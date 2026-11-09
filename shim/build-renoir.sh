#!/bin/bash
# Build the shim for renoir (Xiaomi Mi 11 Lite 5G, SM7350 "shima") and wrap it
# in a boot.img v3, the format of renoir's stock boot_a (vendor_boot supplies
# the DTB and vendor ramdisk; ABL merges them with this image).
#   ./build-renoir.sh            -> out-renoir/shim-renoir-paint.img (milestone 0)
#   ./build-renoir.sh full       -> out-renoir/shim-renoir.img (normal shim flow)
#   ./build-renoir.sh touch      -> out-renoir/shim-renoir-touch.img (touch probe T1)
#   ./build-renoir.sh full P.bin -> same, booting the xnu payload P.bin (tools/mkpayload.py)
set -euo pipefail
cd "$(dirname "$0")"
CROSS=${CROSS:-aarch64-linux-gnu-}
CC=${CROSS}gcc
OBJCOPY=${CROSS}objcopy
NM=${CROSS}nm
READELF=${CROSS}readelf
MODE=${1:-paint}
DEFS="-DBOARD_RENOIR"
[ "$MODE" = paint ] && DEFS="$DEFS -DRENOIR_PAINT_TEST"
[ "$MODE" = touch ] && DEFS="$DEFS -DRENOIR_TOUCH_TEST"
CFLAGS="-O2 -ffreestanding -nostdlib -fno-pic -mgeneral-regs-only -mstrict-align -fno-stack-protector -Wall -Wextra $DEFS"
O=out-renoir
mkdir -p $O
objs=""
for f in start.S shim.c console.c mmu.c fdt.c loader.c ufs.c touch.c; do
	$CC $CFLAGS -c $f -o $O/${f%.*}.o
	objs="$objs $O/${f%.*}.o"
done
abs=$($READELF -r $O/shim.o $O/console.o $O/mmu.o $O/fdt.o $O/loader.o $O/ufs.o $O/touch.o | grep -c ABS64 || true)
[ "$abs" = 0 ] || { echo "error: $abs absolute relocations in C objects" >&2; exit 1; }
$CC $CFLAGS -static -T shim.ld -Wl,--build-id=none -Wl,--no-warn-rwx-segments -Wl,-N $objs -o $O/shim.elf
$OBJCOPY -O binary $O/shim.elf $O/Image
size=$((16#$($NM $O/shim.elf | awk '$3=="_end"{print $1}')))
truncate -s "$size" $O/Image
# Ramdisk: an xnu payload (tools/mkpayload.py) if given as $2, else an empty
# newc cpio (ABL dislikes a zero-size ramdisk on some builds).
if [ -n "${2:-}" ]; then
	cp "$2" $O/ramdisk.bin
else
	(cd $O && rm -rf rd && mkdir rd && cd rd && find . | cpio -o -H newc --quiet | gzip -9 > ../ramdisk.bin)
fi
case "$MODE" in paint) name=shim-renoir-paint;; touch) name=shim-renoir-touch;; *) name=shim-renoir;; esac
python3 ../tools/mkbootv3.py --kernel $O/Image --ramdisk $O/ramdisk.bin \
	--os_version 13.0.0 --os_patch_level 2035-06 \
	--cmdline "console=null" -o $O/$name.img
ls -l $O/Image $O/$name.img
