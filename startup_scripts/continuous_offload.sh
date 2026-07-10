#!/usr/bin/env bash

set -uo pipefail

UUID0="e429997c-69b5-4c72-84a8-341f11aabdc3"

# Continuously offload flight directories from internal storage to external
# storage.
#
# Usage:
#   ./continuous_offload.sh SOURCE_ROOT DEST_MOUNT
#
# Example:
#   ./continuous_offload.sh /mnt/internal/flights /mnt/storage
#
# Each top-level directory inside SOURCE_ROOT is treated as one flight.
#
# A flight becomes eligible only when no file or directory anywhere inside
# that flight has been modified within the last 24 hours.
#
# A flight is deleted from internal storage only after:
#   1. It has been inactive for at least 24 hours.
#   2. rsync completes successfully.
#   3. The source does not change during transfer.
#   4. Every source file checksum-matches the destination copy.
#   5. The external filesystem is still mounted.
#
# Existing destination data that is not present internally is ignored and
# never deleted.

if (( $# != 2 )); then
    echo "Usage: $0 SOURCE_ROOT DEST_MOUNT"
    exit 2
fi

SOURCE_ROOT="$(readlink -f "$1")"
DEST_MOUNT="$(readlink -f "$2")"
DEST_ROOT="${DEST_MOUNT%/}"

# A flight must have no modifications anywhere in its directory tree for
# this many minutes.
MINIMUM_AGE_MINUTES=$((10))

# Delay between scans of the internal flight directory.
SCAN_DELAY=60

# Delay after an interrupted transfer, disconnected drive, or I/O failure.
RETRY_DELAY=15

# Stop rsync if no data is transferred for this many seconds.
RSYNC_TIMEOUT=60

# Prevent multiple instances of this script from running simultaneously.
LOCK_FILE="/tmp/continuous-flight-offload.lock"


log() {
    printf '%s: %s\n' "$(date '+%F %T')" "$*"
}


external_is_mounted() {
    mountpoint -q "$DEST_MOUNT"
}


flight_is_old_enough() {
    local flight_directory="$1"

    # Return success only if no file, directory, or symbolic link anywhere
    # inside the flight has been modified within the last 24 hours.
    #
    # The flight directory itself is included in this check.

    ! find "$flight_directory" \
        -mmin "-${MINIMUM_AGE_MINUTES}" \
        -print -quit \
        2>/dev/null |
        grep -q .
}


source_fingerprint() {
    local directory="$1"

    # Generate a fingerprint from:
    #   - relative paths
    #   - object types
    #   - file sizes
    #   - modification timestamps
    #   - symbolic-link targets
    #
    # This detects files or directories being added, removed, renamed,
    # resized, or modified during transfer and verification.

    find "$directory" \
        -printf '%P\t%y\t%s\t%T@\t%l\n' \
        2>/dev/null |
        LC_ALL=C sort |
        sha256sum |
        awk '{print $1}'
}


verify_source_exists_on_destination() {
    local source="$1"
    local destination="$2"
    local output
    local status

    # Verification behavior:
    #   --recursive: inspect all source subdirectories
    #   --checksum: compare regular-file contents
    #   --dry-run: make no changes
    #   --itemize-changes: report any missing or different source objects
    #
    # No --delete option is used. Extra destination data is intentionally
    # ignored.

    output="$(
        rsync \
            --recursive \
            --links \
            --perms \
            --times \
            --owner \
            --group \
            --devices \
            --specials \
            --hard-links \
            --acls \
            --xattrs \
            --checksum \
            --dry-run \
            --itemize-changes \
            --out-format='%i %n%L' \
            --timeout="$RSYNC_TIMEOUT" \
            "${source%/}/" \
            "${destination%/}/" \
            2>&1
    )"

    status=$?

    if (( status != 0 )); then
        log "Verification rsync failed with exit code $status:"
        printf '%s\n' "$output"
        return 1
    fi

    if [[ -n "$output" ]]; then
        log "Verification found missing or different source data:"
        printf '%s\n' "$output"
        return 1
    fi

    return 0
}


copy_and_remove_flight() {
    local source_flight="$1"
    local flight_name
    local destination_flight
    local before_fingerprint
    local after_fingerprint
    local final_fingerprint
    local rsync_status

    flight_name="$(basename "$source_flight")"
    destination_flight="${DEST_ROOT}/${flight_name}"

    log "Evaluating flight: $flight_name"

    if [[ ! -d "$source_flight" ]]; then
        log "Flight directory no longer exists: $flight_name"
        return 2
    fi

    if ! flight_is_old_enough "$source_flight"; then
        log "Skipping flight modified within the last 24 hours: $flight_name"
        return 2
    fi

    if ! external_is_mounted; then
        log "External filesystem is not mounted at $DEST_MOUNT"
        return 1
    fi

    before_fingerprint="$(source_fingerprint "$source_flight")"

    if [[ -z "$before_fingerprint" ]]; then
        log "Could not fingerprint flight: $flight_name"
        return 1
    fi

    if ! mkdir -p "$destination_flight"; then
        log "Could not create destination: $destination_flight"
        return 1
    fi

    log "Copying flight: $flight_name"

    rsync \
        --archive \
        --hard-links \
        --acls \
        --xattrs \
        --info=progress2 \
        --partial \
        --partial-dir=.rsync-partial \
        --timeout="$RSYNC_TIMEOUT" \
        "${source_flight%/}/" \
        "${destination_flight%/}/"

    rsync_status=$?

    if (( rsync_status != 0 )); then
        log "Transfer interrupted for $flight_name"
        log "rsync exited with code $rsync_status"
        return 1
    fi

    # Request that pending writes be flushed before verification.
    sync

    if ! external_is_mounted; then
        log "External filesystem disappeared before verification."
        return 1
    fi

    after_fingerprint="$(source_fingerprint "$source_flight")"

    if [[ "$before_fingerprint" != "$after_fingerprint" ]]; then
        log "Source changed during transfer."
        log "Internal copy will not be deleted: $flight_name"
        return 1
    fi

    # Recheck the 24-hour condition before verification.
    if ! flight_is_old_enough "$source_flight"; then
        log "Flight was modified during processing."
        log "Internal copy will not be deleted: $flight_name"
        return 1
    fi

    log "Checksum-verifying every source file: $flight_name"

    if ! verify_source_exists_on_destination \
        "$source_flight" \
        "$destination_flight"
    then
        log "Verification failed."
        log "Internal copy will be retained: $flight_name"
        return 1
    fi

    if ! external_is_mounted; then
        log "External filesystem disappeared after verification."
        log "Internal copy will not be deleted: $flight_name"
        return 1
    fi

    final_fingerprint="$(source_fingerprint "$source_flight")"

    if [[ "$before_fingerprint" != "$final_fingerprint" ]]; then
        log "Source changed during or after verification."
        log "Internal copy will not be deleted: $flight_name"
        return 1
    fi

    # Final age check immediately before deletion.
    if ! flight_is_old_enough "$source_flight"; then
        log "Flight is no longer eligible for deletion: $flight_name"
        return 1
    fi

    log "Flight safely verified on external storage: $flight_name"
    log "Deleting internal copy: $source_flight"

    rm -rf --one-file-system -- "$source_flight"

    if [[ -e "$source_flight" ]]; then
        log "Internal deletion did not complete: $flight_name"
        return 1
    fi

    log "Successfully offloaded and removed: $flight_name"
    return 0
}


main() {
    local found_flight
    local source_flight
    local result

    if [[ ! -d "$SOURCE_ROOT" ]]; then
        log "Source root does not exist: $SOURCE_ROOT"
        exit 1
    fi

    exec 9>"$LOCK_FILE"

    if ! flock -n 9; then
        log "Another offload process is already running."
        exit 1
    fi

    log "Continuous flight offload started."
    log "Internal source:  $SOURCE_ROOT"
    log "External mount:   $DEST_MOUNT"
    log "Minimum age:      24 hours"
    log "Scan interval:    ${SCAN_DELAY} seconds"

    while true; do
        if ! external_is_mounted; then
            log "Waiting for external storage at $DEST_MOUNT"
            sleep "$RETRY_DELAY"
            continue
        fi

        found_flight=false

        while IFS= read -r -d '' source_flight; do
            found_flight=true

            copy_and_remove_flight "$source_flight"
            result=$?

            case "$result" in
                0)
                    # Flight was copied, verified, and removed successfully.
                    ;;

                2)
                    # Flight is too recent or no longer exists.
                    ;;

                *)
                    # The external storage may have disconnected, or another
                    # I/O failure occurred.
                    sleep "$RETRY_DELAY"
                    break
                    ;;
            esac
        done < <(
            find "$SOURCE_ROOT" \
                -mindepth 1 \
                -maxdepth 1 \
                -type d \
                -print0 |
                sort -z
        )

        if [[ "$found_flight" == false ]]; then
            log "No internal flight directories found."
        fi

        sleep "$SCAN_DELAY"
    done
}


main
