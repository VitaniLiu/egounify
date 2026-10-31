#!/usr/bin/env bash
# User-authorized destructive operation: only the identified WD partition.
set -Eeuo pipefail
[[ $EUID -eq 0 ]] || { echo 'Run this script with sudo on av-pipeline.' >&2; exit 1; }
[[ $(hostname) == av-pipeline ]] || { echo 'Wrong host'; exit 1; }
project=/home/descfly/apex_n_sharpa
backup="$project/deployment/wd-format-config-backup"
byid=/dev/disk/by-id/usb-WD_My_Passport_2683_575831324133363744314132-0:0
disk=$(readlink -f "$byid")
part=$(readlink -f "$byid-part1")
target=/mnt/apex-data
[[ -b $disk && -b $part ]] || { echo 'WD not connected'; exit 1; }
[[ $(lsblk -dn -o SERIAL "$disk" | xargs) == WD-WX12A367D1A2 ]] || { echo 'Wrong disk serial'; exit 1; }
[[ $(lsblk -dn -o TRAN "$disk" | xargs) == usb ]] || exit 1
[[ $(lsblk -nr -o TYPE "$disk" | wc -l) -eq 2 ]] || { echo 'Unexpected partition layout'; exit 1; }
[[ $(lsblk -dn -o PKNAME "$part" | xargs) == "${disk##*/}" ]] || exit 1
rootdev=$(findmnt -n -o SOURCE /)
if lsblk -snr -o PATH "$rootdev" | grep -Fxq "$disk"; then echo 'Refusing system disk'; exit 1; fi
[[ $(blkid -s TYPE -o value "$part") == exfat ]] || { echo 'Filesystem already changed: inspect manually; do not reformat.'; exit 1; }
[[ $(blkid -s UUID -o value "$part") == 62E3-577C ]] || { echo 'Unexpected original UUID'; exit 1; }
if findmnt -rn -M "$target" >/dev/null; then echo 'Target already mounted: inspect manually'; exit 1; fi
if awk '$1 !~ /^#/ && $2=="/mnt/apex-data" {found=1} END {exit !found}' /etc/fstab; then echo 'fstab target already configured: inspect manually'; exit 1; fi
python3 - "$backup" <<'PY'
from pathlib import Path
import hashlib,json,sys
b=Path(sys.argv[1]); manifest=json.loads((b/'sha256.json').read_text())
assert manifest
for rel,expected in manifest.items():
 p=(b/rel).resolve(); assert b in p.parents
 assert p.is_file() and hashlib.sha256(p.read_bytes()).hexdigest()==expected
print('Project configuration backup verified; other WD files will be erased.')
PY
# Non-forced unmount fails if the filesystem is busy.
if findmnt -rn -S "$part" >/dev/null; then umount "$part"; fi
if findmnt -rn -S "$part" >/dev/null; then echo 'WD is still mounted'; exit 1; fi
[[ $(lsblk -dn -o SERIAL "$disk" | xargs) == WD-WX12A367D1A2 ]] || exit 1
echo "Formatting $part (WD-WX12A367D1A2) as ext4; all existing contents are erased."
mkfs.ext4 -F -L apex-data -m 0 "$part"
udevadm settle
uuid=$(blkid -s UUID -o value "$part")
[[ -n $uuid && $(blkid -s TYPE -o value "$part") == ext4 ]] || exit 1
install -d -o root -g root -m 0755 "$target"
mount -t ext4 "UUID=$uuid" "$target"
[[ $(findmnt -n -M "$target" -o UUID) == "$uuid" ]] || exit 1
install -d -o descfly -g descfly -m 0755 "$target/apex_n_sharpa" "$target/apex_n_sharpa/data"
for name in config references raw_data/apex_data raw_data/sharpa_data hdf5 lerobot test_sources; do
 install -d -o descfly -g descfly -m 0755 "$target/apex_n_sharpa/data/$name"
done
cp -a "$backup/config/." "$target/apex_n_sharpa/data/config/"
cp -a "$backup/references/." "$target/apex_n_sharpa/data/references/"
chown -R descfly:descfly "$target/apex_n_sharpa"
stamp=$(date -u +%Y%m%dT%H%M%SZ)
cp -a /etc/fstab "/etc/fstab.before-apex-data-$stamp"
printf '\n# WD My Passport data disk: WD-WX12A367D1A2\nUUID=%s /mnt/apex-data ext4 defaults,nofail,x-systemd.device-timeout=10s 0 2\n' "$uuid" >> /etc/fstab
systemctl daemon-reload
runuser -u descfly -- env APEX_STORAGE_MOUNT="$target" "$project/.venv/bin/python" - "$project" "$backup" <<'PY'
from pathlib import Path
import sys,hashlib,json,importlib.util,subprocess
project=Path(sys.argv[1]); backup=Path(sys.argv[2]); data=Path('/mnt/apex-data/apex_n_sharpa/data')
for rel,expected in json.loads((backup/'sha256.json').read_text()).items():
 assert hashlib.sha256((data/rel).read_bytes()).hexdigest()==expected
for step in ['read_raw_data','apex2hdf5','hdf52lerobot']:
 spec=importlib.util.spec_from_file_location(step,project/'src'/step/'main.py')
 m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);m.ensure_storage();assert m.DATA==data
probe=data/'.storage-write-test';probe.write_text('test');assert probe.read_text()=='test';probe.unlink()
subprocess.run([sys.executable,'-m','pytest','tests','-q','--junitxml=deployment/pytest-ext4.xml'],cwd=project,check=True)
report={'status':'complete','data_root':str(data),'filesystem':'ext4','serial':'WD-WX12A367D1A2','configuration_restored':True,'tests_report':'deployment/pytest-ext4.xml','real_data_processed':False}
(project/'deployment/format-wd-result.json').write_text(json.dumps(report,indent=2)+'\n')
PY
findmnt -M "$target"
df -hT "$target"
echo 'Complete: /mnt/apex-data/apex_n_sharpa/data'
