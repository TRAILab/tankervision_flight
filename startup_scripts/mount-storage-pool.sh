#!/usr/bin/env bash
set -euo pipefail

UUID0="3083-C6EC"
UUID1="C4C6-7FCD"
UUID2="58D2-4F5D"

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
mount -t exfat -o defaults,nofail,noatime,uid=1000,gid=1000,umask=0022 /dev/disk/by-uuid/$UUID0 /mnt/hdd0
mount -t exfat -o defaults,nofail,noatime,uid=1000,gid=1000,umask=0022 /dev/disk/by-uuid/$UUID1 /mnt/hdd1
mount -t exfat -o defaults,nofail,noatime,uid=1000,gid=1000,umask=0022 /dev/disk/by-uuid/$UUID2 /mnt/hdd2

echo "Verifying SSD mounts..."
findmnt /mnt/hdd0 >/dev/null
findmnt /mnt/hdd1 >/dev/null
findmnt /mnt/hdd2 >/dev/null

echo "Mounting mergerfs..."
mergerfs \
  -o allow_other,use_ino,cache.files=off,category.create=epmfs,func.getattr=newest,dropcacheonclose=false,minfreespace=100G \
  /mnt/hdd0:/mnt/hdd1:/mnt/hdd2 \
  /mnt/storage

echo "Mounted storage pool:"
findmnt -R /mnt
df -h /mnt/hdd0 /mnt/hdd1 /mnt/hdd2 /mnt/storage
