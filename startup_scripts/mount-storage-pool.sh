#!/usr/bin/env bash
set -euo pipefail

UUID0="a105c0df-1e33-447d-be55-99f1007297c0"
UUID1="e92d0907-42e6-4261-b3da-2d03c7caca3f"
UUID2="64b01f03-168c-42f4-afe4-fa252d927d52"

STORAGE_USER="${STORAGE_USER:-${SUDO_USER:-}}"
if [[ -z "$STORAGE_USER" ]]; then
    STORAGE_UID=1000
    STORAGE_GID=1000
else
    STORAGE_UID=$(id -u "$STORAGE_USER")
    STORAGE_GID=$(id -g "$STORAGE_USER")
fi

mkdir -p /mnt/hdd0 /mnt/hdd1 /mnt/hdd2 /mnt/storage

echo "Waiting 20 seconds for USB bus to settle..."
sleep 20

echo "Waiting for SSD UUIDs..."
for uuid in "$UUID0" "$UUID1" "$UUID2"; do
    for i in $(seq 1 90); do
        if [[ -e "/dev/disk/by-uuid/$uuid" ]]; then
            echo "Found $uuid"
            break
        fi
        if [[ "$i" -eq 90 ]]; then
            echo "ERROR: UUID $uuid not found"
            exit 1
        fi
        sleep 1
    done
done

echo "Unmounting stale mounts..."
umount /mnt/storage 2>/dev/null || true
umount /mnt/hdd0 2>/dev/null || true
umount /mnt/hdd1 2>/dev/null || true
umount /mnt/hdd2 2>/dev/null || true

echo "Mounting SSDs..."
mount -t ext4 -o defaults,noatime /dev/disk/by-uuid/$UUID0 /mnt/hdd0
mount -t ext4 -o defaults,noatime /dev/disk/by-uuid/$UUID1 /mnt/hdd1
mount -t ext4 -o defaults,noatime /dev/disk/by-uuid/$UUID2 /mnt/hdd2
chown "$STORAGE_UID:$STORAGE_GID" /mnt/hdd0 /mnt/hdd1 /mnt/hdd2

echo "Verifying SSD mounts..."
findmnt /mnt/hdd0 >/dev/null
findmnt /mnt/hdd1 >/dev/null
findmnt /mnt/hdd2 >/dev/null

echo "Mounting mergerfs..."
if ! mergerfs \
    -o allow_other,use_ino,cache.files=off,category.create=epmfs,func.getattr=newest,dropcacheonclose=false,minfreespace=100G \
    /mnt/hdd0:/mnt/hdd1:/mnt/hdd2 \
    /mnt/storage; then
    echo "ERROR: mergerfs mount failed"
    mount | grep -E "/mnt/hdd|/mnt/storage" || true
    ls -la /mnt/storage || true
    exit 1
fi

echo "Verifying mergerfs mount..."
if ! findmnt -T /mnt/storage -t fuse.mergerfs >/dev/null; then
    echo "ERROR: /mnt/storage is not a mergerfs mount"
    findmnt /mnt/storage || true
    mount | grep -E "/mnt/hdd|/mnt/storage" || true
    exit 1
fi
chown "$STORAGE_UID:$STORAGE_GID" /mnt/storage

echo "Mounted storage pool:"
findmnt /mnt/hdd0 /mnt/hdd1 /mnt/hdd2 /mnt/storage || true
df -h /mnt/hdd0 /mnt/hdd1 /mnt/hdd2 /mnt/storage || true
