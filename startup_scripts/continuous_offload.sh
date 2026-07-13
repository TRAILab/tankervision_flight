#!/usr/bin/env bash

set -uo pipefail

# Continuously offload flight directories from internal storage to an external
# SSD pool mounted via mergerfs, then verify and delete the internal copy.
#
# Source:      /home/<hostname>/storage
# Destination: /mnt/storage  (mergerfs over /mnt/hdd0[:/mnt/hdd1:...])
#
# A flight becomes eligible only when no file or directory anywhere inside it
# has been modified within the last MINIMUM_AGE_MINUTES minutes.
#
# A flight is deleted from internal storage only after:
#   1. It has been inactive for at least MINIMUM_AGE_MINUTES.
#   2. rsync completes successfully.
#   3. The source does not change during transfer.
#   4. Every source file checksum-matches the destination copy.
#   5. The mergerfs pool is still healthy.
#
# If the SSD pool becomes unhealthy at any point, the script kills any
# in-progress rsync within seconds, tears down the mount stack, and waits
# for the SSDs to reconnect before starting over.

# --------------------------------------------------------------------------
# Per-unit SSD UUID configuration (keyed by hostname)
# --------------------------------------------------------------------------
UNIT="${STORAGE_USER:-}"
if [[ -z "$UNIT" ]]; then
    printf '%s: ERROR: STORAGE_USER is not set\n' "$(date '+%F %T')" >&2
    exit 1
fi

case "$UNIT" in
    brigid)
        UUIDS=(
            "e429997c-69b5-4c72-84a8-341f11aabdc3"
        )
        ;;
    argus)
        UUIDS=(
            "a105c0df-1e33-447d-be55-99f1007297c0"
            "e92d0907-42e6-4261-b3da-2d03c7caca3f"
            "64b01f03-168c-42f4-afe4-fa252d927d52"
        )
        ;;
    atlas)
        UUIDS=(
            "399be1e1-3e86-40f2-8da7-0fc2aa5096dc"
            "b1614b8f-8468-4485-bddd-4b5614fe497e"
            "faa4aec5-e19e-4e50-9325-6f14b7ce7368"
        )
        ;;
    helios)
        UUIDS=(
            "b69f553b-b3a0-4ea6-a8c2-aceb2b45f988"
            "d47b873b-8289-4db4-be48-a105dc508622"
            "c55cf816-839b-4f0e-b83f-9535db6711be"
        )
        ;;
    *)
        printf '%s: ERROR: No SSD UUID configuration for hostname "%s"\n' \
            "$(date '+%F %T')" "$UNIT" >&2
        exit 1
        ;;
esac

SOURCE_ROOT="/home/${UNIT}/storage"
DEST_MOUNT="/mnt/storage"

# A flight must have no modifications for this many minutes before it is
# eligible for transfer. (24hrs)
MINIMUM_AGE_MINUTES=$((60 * 24))

# Seconds between scans when the pool is healthy and transfers are complete.
SCAN_DELAY=60

# Seconds to wait before retrying after a failure or mount setup error.
RETRY_DELAY=15

# rsync fallback: stop if no data is transferred for this many seconds.
# The watchdog is the primary protection; this is belt-and-suspenders.
RSYNC_TIMEOUT=30

# Watchdog aggressiveness.
# Detection latency is at most WATCHDOG_PROBE_INTERVAL + WATCHDOG_PROBE_TIMEOUT
# seconds — currently about 5 seconds worst case.
WATCHDOG_PROBE_INTERVAL=2   # seconds between health probes
WATCHDOG_PROBE_TIMEOUT=3    # seconds before a probe is declared hung

LOCK_FILE="/tmp/continuous-flight-offload.lock"

# Temp files scoped to this process.
WATCHDOG_FLAG_FILE="/tmp/offload-watchdog-flag.$$"
RSYNC_PID_FILE="/tmp/offload-rsync-pid.$$"

WATCHDOG_PID=""

# Build /mnt/hdd0, /mnt/hdd1, ... from the UUID list.
MOUNT_POINTS=()
for _i in "${!UUIDS[@]}"; do
    MOUNT_POINTS+=("/mnt/hdd${_i}")
done
unset _i

# --------------------------------------------------------------------------
log() { printf '%s: %s\n' "$(date '+%F %T')" "$*"; }

_cleanup() {
    rm -f "$WATCHDOG_FLAG_FILE" "$RSYNC_PID_FILE"
}
trap _cleanup EXIT

# --------------------------------------------------------------------------
# Mount management
# --------------------------------------------------------------------------

teardown_mounts() {
    log "Tearing down mounts (lazy)..."
    # Lazy unmount avoids blocking if FUSE is already stuck.
    umount -l "$DEST_MOUNT" 2>/dev/null || true
    for _mp in "${MOUNT_POINTS[@]}"; do
        umount -l "$_mp" 2>/dev/null || true
    done
    unset _mp
    sleep 1
}

wait_for_ssds() {
    log "Waiting for all SSDs to appear in /dev/disk/by-uuid..."
    while true; do
        local all_found=true
        for uuid in "${UUIDS[@]}"; do
            if [[ ! -e "/dev/disk/by-uuid/$uuid" ]]; then
                all_found=false
                break
            fi
        done
        [[ "$all_found" == true ]] && break
        log "  Not all SSDs present yet. Checking again in 5s..."
        sleep 5
    done
    log "All ${#UUIDS[@]} SSD(s) present."
}

setup_mounts() {
    mkdir -p "$DEST_MOUNT"
    for mp in "${MOUNT_POINTS[@]}"; do
        mkdir -p "$mp"
    done

    for i in "${!UUIDS[@]}"; do
        local uuid="${UUIDS[$i]}"
        local mp="${MOUNT_POINTS[$i]}"
        if mountpoint -q "$mp" 2>/dev/null; then
            log "  $mp already mounted."
        else
            log "  Mounting $uuid -> $mp"
            if ! mount -t ext4 -o defaults,noatime "/dev/disk/by-uuid/$uuid" "$mp"; then
                log "ERROR: Failed to mount UUID $uuid at $mp"
                return 1
            fi
        fi
    done

    local branches
    branches="$(IFS=':'; printf '%s' "${MOUNT_POINTS[*]}")"
    log "Mounting mergerfs: $branches -> $DEST_MOUNT"
    if ! mergerfs \
        -o allow_other,use_ino,cache.files=off,category.create=epmfs,func.getattr=newest,dropcacheonclose=false,minfreespace=100G \
        "$branches" \
        "$DEST_MOUNT"; then
        log "ERROR: mergerfs mount failed"
        return 1
    fi

    # Confirm mergerfs is immediately responsive with a real write before starting the watchdog.
    if ! timeout "$WATCHDOG_PROBE_TIMEOUT" \
        bash -c "touch '${DEST_MOUNT}/.watchdog_probe' && rm -f '${DEST_MOUNT}/.watchdog_probe'" \
        >/dev/null 2>&1; then
        log "ERROR: mergerfs I/O probe failed immediately after mount"
        return 1
    fi

    log "Storage pool ready (${#UUIDS[@]} SSD(s))."
    return 0
}

# --------------------------------------------------------------------------
# Watchdog (runs as a background subshell)
#
# Every WATCHDOG_PROBE_INTERVAL seconds it:
#   1. Checks that each /mnt/hddN mountpoint is still up (no I/O required).
#   2. Probes mergerfs itself with a hard timeout.
#
# On failure it:
#   a. Touches WATCHDOG_FLAG_FILE so the main loop can detect the problem.
#   b. Reads RSYNC_PID_FILE and kills whatever rsync is running.
#   c. Exits (the main loop is responsible for restarting the mount cycle).
# --------------------------------------------------------------------------

_watchdog_body() {
    while true; do
        sleep "$WATCHDOG_PROBE_INTERVAL"

        local problem=false

        # Fast check: underlying mountpoints (no filesystem I/O).
        for mp in "${MOUNT_POINTS[@]}"; do
            if ! mountpoint -q "$mp" 2>/dev/null; then
                log "[watchdog] SSD mountpoint gone: $mp"
                problem=true
                break
            fi
        done

        # Timed write probe: forces real block device I/O through mergerfs.
        # A plain stat answers from FUSE memory without hitting the SSD.
        if [[ "$problem" == false ]]; then
            if ! timeout "$WATCHDOG_PROBE_TIMEOUT" \
                bash -c "touch '${DEST_MOUNT}/.watchdog_probe' && rm -f '${DEST_MOUNT}/.watchdog_probe'" \
                >/dev/null 2>&1; then
                log "[watchdog] mergerfs I/O probe timed out or failed"
                problem=true
            fi
        fi

        if [[ "$problem" == true ]]; then
            touch "$WATCHDOG_FLAG_FILE"
            local pid
            pid="$(cat "$RSYNC_PID_FILE" 2>/dev/null || true)"
            if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
                log "[watchdog] Sending SIGTERM to rsync PID $pid"
                kill "$pid" 2>/dev/null || true
                sleep 1
                # Escalate to SIGKILL if still alive.
                kill -0 "$pid" 2>/dev/null && kill -9 "$pid" 2>/dev/null || true
            fi
            return
        fi
    done
}

start_watchdog() {
    rm -f "$WATCHDOG_FLAG_FILE"
    printf '' > "$RSYNC_PID_FILE"
    _watchdog_body &
    WATCHDOG_PID=$!
    log "Watchdog started (PID $WATCHDOG_PID): probe every ${WATCHDOG_PROBE_INTERVAL}s, timeout ${WATCHDOG_PROBE_TIMEOUT}s"
}

stop_watchdog() {
    if [[ -n "$WATCHDOG_PID" ]]; then
        kill "$WATCHDOG_PID" 2>/dev/null || true
        wait "$WATCHDOG_PID" 2>/dev/null || true
        WATCHDOG_PID=""
    fi
}

storage_healthy() {
    [[ ! -f "$WATCHDOG_FLAG_FILE" ]]
}

# --------------------------------------------------------------------------
# Flight helpers
# --------------------------------------------------------------------------

flight_is_old_enough() {
    local flight_directory="$1"
    local recent

    recent="$(
        find "$flight_directory" \
            -xdev \
            -mmin "-${MINIMUM_AGE_MINUTES}" \
            -print -quit
    )" || {
        log "ERROR: Could not inspect age of $flight_directory"
        return 1
    }

    [[ -z "$recent" ]]
}

source_fingerprint() {
    local directory="$1"
    # Covers paths, types, sizes, mtimes, symlink targets.
    find "$directory" \
        -printf '%P\t%y\t%s\t%T@\t%l\n' \
        2>/dev/null |
        LC_ALL=C sort |
        sha256sum |
        awk '{print $1}'
}

# --------------------------------------------------------------------------
# Transfer one flight.
# Returns: 0 = offloaded and deleted, 1 = storage/IO failure, 2 = skip
# --------------------------------------------------------------------------
copy_and_remove_flight() {
    local source_flight="$1"
    local flight_name destination_flight
    local before_fp after_fp final_fp
    local rsync_status verify_status
    local verify_tmp verify_output

    flight_name="$(basename "$source_flight")"
    destination_flight="${DEST_MOUNT}/${flight_name}"

    log "Evaluating: $flight_name"

    [[ -d "$source_flight" ]] || { log "No longer exists: $flight_name"; return 2; }
    flight_is_old_enough "$source_flight" || { log "Too recent, skipping: $flight_name"; return 2; }
    storage_healthy || { log "Storage unhealthy before transfer of $flight_name"; return 1; }

    before_fp="$(source_fingerprint "$source_flight")"
    [[ -n "$before_fp" ]] || { log "Could not fingerprint: $flight_name"; return 1; }

    # Use timeout on mkdir — a hung mergerfs will block it indefinitely.
    timeout "$WATCHDOG_PROBE_TIMEOUT" mkdir -p "$destination_flight" 2>/dev/null || {
        log "mkdir on destination failed or timed out: $destination_flight"
        return 1
    }

    # ---- Copy ----
    log "Copying: $flight_name"
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
        "${destination_flight%/}/" &
    local rsync_pid=$!
    printf '%s' "$rsync_pid" > "$RSYNC_PID_FILE"
    wait "$rsync_pid"
    rsync_status=$?
    printf '' > "$RSYNC_PID_FILE"

    if ! storage_healthy; then
        log "Watchdog flagged storage failure during copy of $flight_name"
        return 1
    fi
    if (( rsync_status != 0 )); then
        log "rsync failed (exit $rsync_status) for $flight_name"
        return 1
    fi

    after_fp="$(source_fingerprint "$source_flight")"
    [[ "$before_fp" == "$after_fp" ]] || {
        log "Source changed during transfer; retaining internal copy: $flight_name"
        return 1
    }
    flight_is_old_enough "$source_flight" || {
        log "Flight modified during transfer; retaining internal copy: $flight_name"
        return 1
    }

    # ---- Checksum verification ----
    log "Checksum-verifying: $flight_name"
    verify_tmp="$(mktemp /tmp/offload-verify.XXXXXX)"

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
        --checksum-choice=xxh128 \
        --dry-run \
        --itemize-changes \
        --out-format='%i %n%L' \
        --timeout="$RSYNC_TIMEOUT" \
        "${source_flight%/}/" \
        "${destination_flight%/}/" \
        >"$verify_tmp" 2>&1 &
    local verify_pid=$!
    printf '%s' "$verify_pid" > "$RSYNC_PID_FILE"
    wait "$verify_pid"
    verify_status=$?
    printf '' > "$RSYNC_PID_FILE"
    verify_output="$(cat "$verify_tmp")"
    rm -f "$verify_tmp"

    if ! storage_healthy; then
        log "Watchdog flagged storage failure during verification of $flight_name"
        return 1
    fi
    if (( verify_status != 0 )); then
        log "Verification rsync failed (exit $verify_status) for $flight_name:"
        printf '%s\n' "$verify_output"
        return 1
    fi
    if [[ -n "$verify_output" ]]; then
        log "Verification found discrepancies for $flight_name:"
        printf '%s\n' "$verify_output"
        return 1
    fi

    if ! storage_healthy; then
        log "Storage disappeared after verification of $flight_name"
        return 1
    fi

    final_fp="$(source_fingerprint "$source_flight")"
    [[ "$before_fp" == "$final_fp" ]] || {
        log "Source changed after verification; retaining internal copy: $flight_name"
        return 1
    }
    flight_is_old_enough "$source_flight" || {
        log "No longer eligible for deletion: $flight_name"
        return 1
    }

    # ---- Delete ----
    log "Verified. Deleting internal copy: $source_flight"
    rm -rf --one-file-system -- "$source_flight"
    [[ ! -e "$source_flight" ]] || { log "Deletion incomplete: $flight_name"; return 1; }

    log "Successfully offloaded and removed: $flight_name"
    return 0
}

# --------------------------------------------------------------------------
# One scan pass over all eligible flights.
# Returns 1 immediately if storage fails mid-scan.
# --------------------------------------------------------------------------
run_offload_scan() {
    local source_flight result found=false

    while IFS= read -r -d '' source_flight; do
        found=true

        if ! storage_healthy; then
            log "Storage became unhealthy mid-scan. Aborting."
            return 1
        fi

        copy_and_remove_flight "$source_flight"
        result=$?

        case "$result" in
            0|2) ;;     # success or skip — continue scan
            *)  return 1 ;;
        esac
    done < <(
        find "$SOURCE_ROOT" -mindepth 1 -maxdepth 1 -type d -print0 | sort -z
    )

    [[ "$found" == true ]] || log "No flight directories found."
    return 0
}

# --------------------------------------------------------------------------
# Main
# --------------------------------------------------------------------------
main() {
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
    log "Unit:            $UNIT"
    log "Source:          $SOURCE_ROOT"
    log "Destination:     $DEST_MOUNT"
    log "SSDs:            ${#UUIDS[@]}"
    log "Min flight age:  ${MINIMUM_AGE_MINUTES} minutes"
    log "Scan interval:   ${SCAN_DELAY}s"
    log "Watchdog:        probe every ${WATCHDOG_PROBE_INTERVAL}s, kill if probe takes >${WATCHDOG_PROBE_TIMEOUT}s"

    while true; do
        # Always start a mount cycle clean.
        stop_watchdog
        teardown_mounts
        wait_for_ssds

        if ! setup_mounts; then
            log "Mount setup failed. Retrying in ${RETRY_DELAY}s..."
            sleep "$RETRY_DELAY"
            continue
        fi

        start_watchdog

        # Inner loop: keep scanning until storage health is lost.
        while true; do
            if ! storage_healthy; then
                log "Storage health lost. Restarting mount cycle."
                break
            fi

            if ! run_offload_scan; then
                log "Scan aborted. Restarting mount cycle."
                break
            fi

            log "Scan complete. Next scan in ${SCAN_DELAY}s..."
            sleep "$SCAN_DELAY"
        done

        stop_watchdog
        teardown_mounts
        log "Waiting ${RETRY_DELAY}s before reconnect attempt..."
        sleep "$RETRY_DELAY"
    done
}

main
