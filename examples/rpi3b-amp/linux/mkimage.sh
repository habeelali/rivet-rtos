#!/usr/bin/env bash
# Clone a provisioned card into a single distributable .img.
#
#   sudo ./mkimage.sh /dev/sdX rivet-pi3.img
#
# Reads only to the end of the last partition rather than the whole card,
# so an 8 GB card does not become an 8 GB file when 3 GB of it is in use.
# Shrinking the filesystem further is left to pishrink, which does it
# properly; this deliberately does not try to reimplement that.
set -euo pipefail

DEV="${1:?usage: mkimage.sh <device> <out.img>}"
OUT="${2:?usage: mkimage.sh <device> <out.img>}"

[ -b "$DEV" ] || { echo "$DEV is not a block device" >&2; exit 1; }

# Refuse anything that is not removable, so a typo cannot read the host's
# own disk into a file.
NAME=$(basename "$DEV")
if [ "$(cat "/sys/block/$NAME/removable" 2>/dev/null || echo 0)" != "1" ]; then
    echo "$DEV is not removable; refusing" >&2
    exit 1
fi
if mount | grep -q "^$DEV"; then
    echo "unmount its partitions first:" >&2
    mount | grep "^$DEV" | sed 's/^/  /' >&2
    exit 1
fi

# End of the last partition, in 512-byte sectors.
END=$(partx -g -o END "$DEV" | tail -1 | tr -d ' ')
COUNT=$(( (END + 1) ))
MB=$(( COUNT / 2048 ))
echo "last partition ends at sector $END, copying ${MB} MiB"

dd if="$DEV" of="$OUT" bs=1M count="$MB" status=progress conv=fsync
echo
echo "wrote $OUT ($(du -h "$OUT" | cut -f1))"
echo "shrink further with:  sudo pishrink.sh $OUT"
echo "write it back with:   sudo dd if=$OUT of=/dev/sdX bs=4M conv=fsync status=progress"
