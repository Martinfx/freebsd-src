# FreeBSD SD card image for the Banana Pi BPI-R64 (MediaTek MT7622)

Everything here builds without root and without a FreeBSD host: the world
and the `MEDIATEK` kernel are cross-compiled on Linux with
`tools/build/make.py`, the firmware with an `aarch64-linux-gnu` GCC, and the
image is assembled with `makefs(8)` and `sgdisk(8)`.  The same steps run on
GitHub Actions in `.github/workflows/bpi-r64-image.yml`.

## Boot chain and card layout

MT7622 boots from SD card as: BootROM -> BL2 (ARM Trusted Firmware) ->
FIP (BL31 + U-Boot) -> U-Boot -> `EFI/BOOT/BOOTAA64.EFI` (FreeBSD
`loader.efi`) -> kernel.  `mkimage.sh` produces this layout (512-byte
sectors), which is what frank-w's `mtk-atf` `build.sh createimg` and OpenWrt
use for `sdmmc`:

| Offset            | Content                                   |
|-------------------|-------------------------------------------|
| LBA 0-33          | protective MBR + GPT                      |
| LBA 1024 (512 KiB)| `bl2.img` (raw, no partition)             |
| LBA 2048 (1 MiB)  | p1 `fip`: `fip.bin` (BL31 + U-Boot)       |
| LBA 6144 (3 MiB)  | p2 `ubootenv`: U-Boot environment         |
| LBA 8192 (4 MiB)  | p3 `efi`: FAT16 ESP, loader + `dtb/`      |
| next MiB          | p4 `rootfs`: UFS2, grown on first boot    |

## Manual build on Linux (Debian/Ubuntu)

```sh
sudo apt install bmake libarchive-dev flex bison time gdisk \
    gcc-aarch64-linux-gnu bison flex swig python3-dev python3-setuptools \
    python3-pyelftools libssl-dev device-tree-compiler uuid-dev libgnutls28-dev bc xxd

git clone -b max-support-mediatek-7622-bananapir64 https://github.com/Martinfx/freebsd-src.git
cd freebsd-src

# 1. firmware: ATF BL2 + FIP with U-Boot (frank-w/u-boot 2026-07-bpi + mtk-atf)
./tools/bpi-r64/build-firmware.sh -o ../firmware -w ../firmware-src

# 2. world + MEDIATEK kernel, cross-built from Linux.
# The compiler must be at least as new as the in-tree LLVM (see
# lib/clang/include/llvm/Config/llvm-config.h, 21.1.8 today); Ubuntu's
# clang-18 fails in libc++.  Use the official release binaries:
V=$(awk '$2 == "LLVM_VERSION_STRING" {gsub(/"/, "", $3); print $3}' lib/clang/include/llvm/Config/llvm-config.h)
sudo mkdir -p /opt/llvm
curl -sSfL "https://github.com/llvm/llvm-project/releases/download/llvmorg-$V/LLVM-$V-Linux-X64.tar.xz" \
    | sudo tar xJ -C /opt/llvm --strip-components=1
export MAKEOBJDIRPREFIX=$PWD/../obj
cat > ../src.conf <<'EOT'
WITHOUT_TOOLCHAIN=yes
WITHOUT_LLVM_TARGET_ALL=yes
WITHOUT_LLDB=yes
WITHOUT_TESTS=yes
WITHOUT_DEBUG_FILES=yes
WITHOUT_PROFILE=yes
WITHOUT_LIB32=yes
WITHOUT_GOOGLETEST=yes
EOT
LLVM=/opt/llvm/bin
# X* binutils must be full paths, otherwise buildworld bootstraps lib/clang
# and the LLVM binutils from source (hours).
ARGS="--cross-bindir=$LLVM TARGET=arm64 TARGET_ARCH=aarch64 \
      -DWITHOUT_CLEAN -DWITH_DISK_IMAGE_TOOLS_BOOTSTRAP SRCCONF=$PWD/../src.conf \
      XAR=$LLVM/llvm-ar XNM=$LLVM/llvm-nm XOBJCOPY=$LLVM/llvm-objcopy \
      XSIZE=$LLVM/llvm-size XSTRINGS=$LLVM/llvm-strings XSTRIPBIN=$LLVM/llvm-strip"
./tools/build/make.py $ARGS buildworld -s -j$(nproc)
./tools/build/make.py $ARGS KERNCONF=MEDIATEK buildkernel -s -j$(nproc)
./tools/build/make.py $ARGS KERNCONF=MEDIATEK DESTDIR=$PWD/../rootfs NO_ROOT=1 \
    -DDB_FROM_SRC installworld installkernel distribution -s -j$(nproc)

# 3. assemble the SD image
OBJ=$MAKEOBJDIRPREFIX$PWD/arm64.aarch64
./tools/bpi-r64/mkimage.sh -d ../rootfs \
    -l $OBJ/stand/efi/loader_lua/loader_lua.efi \
    -m $(find $OBJ/tmp -type f -perm -u+x -name makefs | head -n1) \
    -b ../firmware/bl2.img -f ../firmware/fip.bin -s 2g \
    -o ../bpi-r64-freebsd.img

# 4. write it
sudo dd if=../bpi-r64-freebsd.img of=/dev/sdX bs=1M status=progress conv=fsync
```

On FreeBSD the same `mkimage.sh` works with the base `makefs` and
`sysutils/gdisk`; run steps 2 and 3 with plain `make` (drop
`--cross-bindir`).  `build-firmware.sh` needs the Linux cross GCC (the
MediaTek ATF does not build with clang), so build the firmware on Linux or
take `bl2.img` / `fip.bin` from a frank-w `bpi-r64_sdmmc.img.gz`.

## GitHub Actions

`.github/workflows/bpi-r64-image.yml` runs on every push to
`max-support-mediatek-7622-bananapir64` and on demand (`workflow_dispatch`,
inputs: `ref`, `kernconf`, `root_size`, `modules`).  Two jobs:

* `firmware` (a few minutes): `build-firmware.sh`, uploads `bl2.img`,
  `fip.bin`, `u-boot.bin`, `VERSIONS`.
* `image` (up to 6 h, `timeout-minutes: 360`): buildworld, buildkernel
  `MEDIATEK`, NO_ROOT install, `mkimage.sh`, uploads
  `bpi-r64-freebsd-MEDIATEK-<sha>.img.xz` + `SHA256`.

Artifacts live under the workflow run (Actions -> run -> Artifacts).  To
shorten the kernel step pass `modules`, e.g.
`dtb/mediatek mtkswitch mt7615 if_bridge`.

## Booting

frank-w's U-Boot has `CONFIG_EFI_LOADER`; its default `bootcmd` looks for a
Linux kernel, so point it at the FreeBSD loader once and save it:

```
setenv bootcmd 'load mmc 0:3 ${loadaddr} EFI/BOOT/BOOTAA64.EFI; bootefi ${loadaddr} ${fdtcontroladdr}'
saveenv
```

The loader takes the DTB U-Boot passes via EFI.  To use the in-tree
`mt7622-bananapi-bpi-r64.dtb` instead, load it in U-Boot first
(`load mmc 0:3 ${fdt_addr_r} dtb/mediatek/mt7622-bananapi-bpi-r64.dtb` and
pass `${fdt_addr_r}` to `bootefi`).

The image has no users besides `root` (empty password, console login
only); `sshd` is enabled but refuses root, so set a password and add a
user on the serial console first.  `growfs` expands `rootfs` to the whole
card on the first boot.
