#!/bin/sh
#
# Assemble a bootable SD card image for the Banana Pi BPI-R64 (MT7622) from
# a NO_ROOT installworld/installkernel/distribution tree, the EFI loader
# and the MediaTek boot firmware (bl2.img + fip.bin).
#
# The script only needs makefs(8) (bootstrapped on Linux with
# -DWITH_DISK_IMAGE_TOOLS_BOOTSTRAP), sgdisk(8) from gdisk and dd(1), so it
# runs unprivileged on FreeBSD and on Linux (GitHub Actions).
#
# SD card layout (512-byte sectors), matching what frank-w's mtk-atf
# build.sh createimg and OpenWrt use for MT7622 sdmmc boot:
#
#   LBA    0..  33  protective MBR + GPT header + entries
#   LBA 1024        bl2.img  (raw, read by the BootROM at 512 KiB)
#   LBA 2048..6143  p1 "fip"       fip.bin (BL31 + U-Boot), 2 MiB
#   LBA 6144..7167  p2 "ubootenv"  U-Boot environment (0x300000), 512 KiB
#   LBA 8192..      p3 "efi"       FAT16 ESP: EFI/BOOT/BOOTAA64.EFI + dtb/
#   next 1 MiB      p4 "rootfs"    UFS2 root (grown to the card on first boot)
#
# Usage:
#   mkimage.sh -d DESTDIR -l loader.efi -b bl2.img -f fip.bin -o out.img
#              [-s rootsize] [-e espsize] [-m makefs] [-H hostname]
#

set -eu

DESTDIR=
LOADER=
BL2=
FIP=
OUT=
ROOT_SIZE=${ROOT_SIZE:-2g}
ESP_SIZE=${ESP_SIZE:-64m}
MAKEFS=${MAKEFS:-makefs}
HOSTNAME=${HOSTNAME_IMG:-bpi-r64}

usage() {
	echo "usage: $0 -d DESTDIR -l loader.efi -b bl2.img -f fip.bin -o out.img [-s rootsize] [-e espsize] [-m makefs] [-H hostname]" >&2
	exit 2
}

while getopts "d:l:b:f:o:s:e:m:H:" opt; do
	case ${opt} in
	d) DESTDIR=$OPTARG ;;
	l) LOADER=$OPTARG ;;
	b) BL2=$OPTARG ;;
	f) FIP=$OPTARG ;;
	o) OUT=$OPTARG ;;
	s) ROOT_SIZE=$OPTARG ;;
	e) ESP_SIZE=$OPTARG ;;
	m) MAKEFS=$OPTARG ;;
	H) HOSTNAME=$OPTARG ;;
	*) usage ;;
	esac
done
[ -n "${DESTDIR}" ] && [ -n "${LOADER}" ] && [ -n "${BL2}" ] && \
    [ -n "${FIP}" ] && [ -n "${OUT}" ] || usage

for f in "${LOADER}" "${BL2}" "${FIP}" "${DESTDIR}/METALOG"; do
	[ -f "$f" ] || { echo "$0: missing $f" >&2; exit 1; }
done
command -v sgdisk >/dev/null || { echo "$0: sgdisk(8) not found (install gdisk)" >&2; exit 1; }
command -v "${MAKEFS}" >/dev/null || { echo "$0: makefs not found: ${MAKEFS}" >&2; exit 1; }

DESTDIR=$(cd "${DESTDIR}" && pwd)
WORK=$(mktemp -d "${TMPDIR:-/tmp}/bpi-r64-img.XXXXXX")
trap 'rm -rf "${WORK}"' EXIT

# Record a file/dir we created in DESTDIR in METALOG so that makefs(8)
# with -F picks it up (NO_ROOT installs only log what install(1) wrote).
metalog_add() {
	local file mode type
	file=$1
	if [ -f "${DESTDIR}/${file}" ]; then
		type=file; mode=${2:-0644}
	elif [ -d "${DESTDIR}/${file}" ]; then
		type=dir; mode=${2:-0755}
	else
		echo "metalog_add: ${file} not found" >&2
		return 1
	fi
	echo "${file} type=${type} uname=root gname=wheel mode=${mode}" \
	    >> "${DESTDIR}/METALOG"
}

echo ">>> configuring ${DESTDIR}"
mkdir -p "${DESTDIR}/boot/efi"
metalog_add ./boot/efi

cat > "${DESTDIR}/etc/fstab" <<FEOF
# Custom /etc/fstab for the FreeBSD BPI-R64 image
/dev/ufs/rootfs		/		ufs	rw		1	1
/dev/msdosfs/EFI	/boot/efi	msdosfs	rw,noatime	0	0
tmpfs			/tmp		tmpfs	rw,mode=1777	0	0
FEOF
metalog_add ./etc/fstab

cat > "${DESTDIR}/etc/rc.conf" <<REOF
hostname="${HOSTNAME}"
ifconfig_DEFAULT="DHCP inet6 accept_rtadv"
sshd_enable="YES"
growfs_enable="YES"
REOF
metalog_add ./etc/rc.conf

cat >> "${DESTDIR}/boot/loader.conf" <<LEOF
# Serial console on the MT7622 UART (U-Boot hands us an EFI console on it).
boot_multicons="YES"
boot_serial="YES"
beastie_disable="YES"
loader_color="NO"
LEOF
metalog_add ./boot/loader.conf

: > "${DESTDIR}/firstboot"
metalog_add ./firstboot

# Directories that exist but were not logged (see release/tools/vmimage.subr).
grep type=dir "${DESTDIR}/METALOG" | cut -f 1 -d ' ' | sort -u > "${WORK}/dirs"
( cd "${DESTDIR}" && find . -type d ) | sort | comm -23 - "${WORK}/dirs" > "${WORK}/missing"
while read -r d; do metalog_add "$d"; done < "${WORK}/missing"

# makefs produces directories with 000 permissions if their contents are
# seen before the directories themselves; sort and dedupe.
env -i LC_COLLATE=C sort -u "${DESTDIR}/METALOG" > "${WORK}/METALOG"
cp "${WORK}/METALOG" "${DESTDIR}/METALOG"

echo ">>> building UFS root (${ROOT_SIZE})"
# -N: resolve uname/gname from the image's own passwd/group, not the host's.
( cd "${DESTDIR}" && "${MAKEFS}" -D -B little -s "${ROOT_SIZE}" -t ffs \
    -N "${DESTDIR}/etc" -o label=rootfs -o version=2 -o softupdates=1 \
    "${WORK}/root.img" ./METALOG )

echo ">>> building ESP (${ESP_SIZE})"
ESP=${WORK}/esp
mkdir -p "${ESP}/EFI/BOOT"
cp "${LOADER}" "${ESP}/EFI/BOOT/BOOTAA64.EFI"
if [ -d "${DESTDIR}/boot/dtb" ]; then
	cp -R "${DESTDIR}/boot/dtb" "${ESP}/dtb"
fi
"${MAKEFS}" -t msdos -s "${ESP_SIZE}" -o fat_type=16 -o volume_label=EFI \
    "${WORK}/esp.img" "${ESP}"

bytes() { wc -c < "$1" | tr -d ' '; }
roundup() { echo $(( ($1 + $2 - 1) / $2 * $2 )); }

BL2_SECT=1024
FIP_SECT=2048
FIP_MAX=$(( (6144 - 2048) * 512 ))
ESP_START=8192
ESP_SECT=$(( $(bytes "${WORK}/esp.img") / 512 ))
ROOT_START=$(roundup $(( ESP_START + ESP_SECT )) 2048)
ROOT_SECT=$(( $(bytes "${WORK}/root.img") / 512 ))
IMG_SECT=$(roundup $(( ROOT_START + ROOT_SECT + 34 )) 2048)

if [ "$(bytes "${FIP}")" -gt "${FIP_MAX}" ]; then
	echo "$0: ${FIP} larger than the 2 MiB fip slot" >&2; exit 1
fi
if [ "$(bytes "${BL2}")" -gt $(( (FIP_SECT - BL2_SECT) * 512 )) ]; then
	echo "$0: ${BL2} larger than the 512 KiB bl2 slot" >&2; exit 1
fi

echo ">>> partitioning ${OUT} ($(( IMG_SECT / 2048 )) MiB)"
rm -f "${OUT}"
truncate -s $(( IMG_SECT * 512 )) "${OUT}"
sgdisk -o -a 2048 \
    -n 1:${FIP_SECT}:6143 -c 1:fip -t 1:8300 \
    -n 2:6144:7167 -c 2:ubootenv -t 2:8300 \
    -n 3:${ESP_START}:$(( ESP_START + ESP_SECT - 1 )) -c 3:efi -t 3:EF00 \
    -n 4:${ROOT_START}:$(( ROOT_START + ROOT_SECT - 1 )) -c 4:rootfs -t 4:A503 \
    "${OUT}" >/dev/null

echo ">>> writing firmware and filesystems"
dd if="${BL2}" of="${OUT}" bs=512 seek=${BL2_SECT} conv=notrunc status=none
dd if="${FIP}" of="${OUT}" bs=512 seek=${FIP_SECT} conv=notrunc status=none
dd if="${WORK}/esp.img" of="${OUT}" bs=512 seek=${ESP_START} conv=notrunc status=none
dd if="${WORK}/root.img" of="${OUT}" bs=1M seek=$(( ROOT_START / 2048 )) conv=notrunc status=none

sgdisk -p "${OUT}"
echo ">>> image ready: ${OUT}"
