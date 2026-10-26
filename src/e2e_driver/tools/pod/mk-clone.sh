#!/usr/bin/env bash
# mk-clone.sh <n>: /root/inst/<n>, a runnable copy of the pod's MORAI for world n (n >= 2). Bundle (assets) and Version
# (account store) are symlinks; Project and SaveFile are real copies (the simulator resolves SaveFile next to its real
# path). Every UDP port in the w1_collect presets and the scene EgoNetwork files shifts by (n-1)*1000; MSC listens on
# 10508+(n-1)*1000 and answers to 10326+(n-1)*1000. Adapted from morai-parallel-collect scripts/linux/mk-clone.sh.
set -euo pipefail
n=$1; off=$(( (n - 1) * 1000 )); src=/root/Workspaces/scratches/morai; d=/root/inst/$n/morai
[[ -e $d ]] && { echo "$d exists" >&2; exit 1; }
mkdir -p "$d"
rsync -a --exclude=/MoraiLauncher_Lin_Data/Bundle --exclude=/MoraiLauncher_Lin_Data/Version --exclude=/unity.lock "$src/" "$d/"
ln -s "$src/MoraiLauncher_Lin_Data/Bundle" "$d/MoraiLauncher_Lin_Data/Bundle"
ln -s "$src/MoraiLauncher_Lin_Data/Version" "$d/MoraiLauncher_Lin_Data/Version"
S=$d/MoraiLauncher_Lin_Data/SaveFile
printf 'Receive Host IP:127.0.0.1\nReceive Host PORT:%d\nRequest Destination IP:127.0.0.1\nRequest Destination PORT:%d\n' $((10508 + off)) $((10326 + off)) > "$S/LinkConnection.txt"
python3 - "$off" "$S/Sensor/25.S4.MolitComp03/w1_collect.json" "$S/Network/25.S4.MolitComp03/w1_collect.json" "$S"/Scenario/R_KR_PR_K-city_2025/EgoNetwork/*.json <<'PY'
import json, sys
off = int(sys.argv[1]); n = 0
def walk(o):
    global n
    if isinstance(o, dict):
        u = o.get('udpConfig')
        if isinstance(u, dict):
            u['hostIP'] = u['destinationIP'] = '127.0.0.1'
            for k in ('hostPort', 'destinationPort'):
                if isinstance(u.get(k), int) and 0 < u[k] and u[k] + off < 65536: u[k] += off; n += 1
        for v in o.values(): walk(v)
    elif isinstance(o, list):
        for v in o: walk(v)
for p in sys.argv[2:]:
    d = json.load(open(p, encoding='utf-8-sig')); walk(d); json.dump(d, open(p, 'w', encoding='utf-8'), ensure_ascii=False, indent=4)
print('shifted', n, 'ports by', off, 'in', len(sys.argv) - 2, 'files')
PY
