#!/usr/bin/env python3
"""Minimal Android boot image v3 packer (no dependencies).

  mkbootv3.py --kernel Image --ramdisk rd.bin [--cmdline STR]
              [--os_version 13.0.0] [--os_patch_level 2035-06] -o boot.img

Layout (AOSP boot_img_hdr_v3): a 4096-byte header page, then the kernel and
the ramdisk, each padded to 4096. The DTB is not part of a v3 boot image: ABL
takes it from vendor_boot.
"""
import argparse, struct

PAGE = 4096
HDR_SIZE = 1580


def os_version(ver, patch):
    a, b, c = (int(x) for x in ver.split('.'))
    y, m = (int(x) for x in patch.split('-')[:2])
    return ((a << 14) | (b << 7) | c) << 11 | ((y - 2000) << 4) | m


def pad(data):
    return data + b'\0' * (-len(data) % PAGE)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--kernel', required=True)
    ap.add_argument('--ramdisk', required=True)
    ap.add_argument('--cmdline', default='')
    ap.add_argument('--os_version', default='13.0.0')
    ap.add_argument('--os_patch_level', default='2035-06')
    ap.add_argument('-o', '--out', required=True)
    a = ap.parse_args()
    kernel = open(a.kernel, 'rb').read()
    ramdisk = open(a.ramdisk, 'rb').read()
    cmd = a.cmdline.encode()
    if len(cmd) >= 1536:
        raise SystemExit('cmdline too long')
    hdr = b'ANDROID!' + struct.pack('<4I', len(kernel), len(ramdisk),
                                    os_version(a.os_version, a.os_patch_level), HDR_SIZE)
    hdr += struct.pack('<4I', 0, 0, 0, 0) + struct.pack('<I', 3)
    hdr += cmd + b'\0' * (1536 - len(cmd))
    assert len(hdr) == HDR_SIZE
    with open(a.out, 'wb') as f:
        f.write(pad(hdr) + pad(kernel) + pad(ramdisk))


if __name__ == '__main__':
    main()
