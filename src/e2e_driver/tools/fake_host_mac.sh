#!/usr/bin/env bash
# The fake competition MORAI host on the Mac (bc-250 stays the real simulator host): tools/fake_host.py replays a
# collection bag as the simulator's UDP output at 1x simulator time; the winpc WSL stack reaches it through ssh -R
# (wired LAN 192.168.0.1 when the cable answers, else Tailscale) and lan_tunnel.py stack, as with a real simulator.
#   fake_host_mac.sh BAG [fake_host.py args, e.g. --start 20 --secs 120]
# The stack and the model server must already run on winpc (tools/winpc/server.sh up, stack.sh up).
# Measured 10/26: 3.0 MB/s (4 cameras + LiDAR + GPS/IMU/status), RTT LAN 1.3 ms, Tailscale 2.3 ms (direct over the cable).
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
BAG=$1; shift
LAN=(-o HostKeyAlias='[100.98.142.90]:2222' -o HostName=192.168.0.1)       # the Tailscale host key, the cable address
nc -z -G 2 192.168.0.1 2222 2>/dev/null || { LAN=(); echo "cable not answering: Tailscale"; }
W=morai-eval/ws/MORAI-Autonomous-Driving-System/src/e2e_driver/tools
LOG=${TMPDIR:-/tmp}/fake_host_mac.log

uv run --no-project -q --with rosbags --with numpy python -u "$HERE/fake_host.py" "$BAG" --listen 127.0.0.1:30011 "$@" > "$LOG" 2>&1 &
FH=$!
until grep -q "waiting for the stack tunnel" "$LOG" 2>/dev/null; do kill -0 $FH 2>/dev/null || { cat "$LOG"; exit 1; }; sleep 1; done
ssh "${LAN[@]}" -N -o ExitOnForwardFailure=yes -R 30011:127.0.0.1:30011 winpc-wsl &
FWD=$!
trap 'kill $FH $FWD 2>/dev/null; ssh "${LAN[@]}" winpc-wsl "kill \$(cat ~/morai-eval/run/lan_stack.pid) 2>/dev/null"' EXIT
ssh "${LAN[@]}" winpc-wsl "kill \$(cat ~/morai-eval/run/lan_stack.pid 2>/dev/null) 2>/dev/null; until ss -ltn | grep -q 127.0.0.1:30011; do sleep 1; done
  cd $W && (setsid nohup python3 -u lan_tunnel.py stack --connect 127.0.0.1:30011 > ~/morai-eval/run/lan_stack.log 2>&1 < /dev/null & echo \$! > ~/morai-eval/run/lan_stack.pid)"
wait $FH
tail -1 "$LOG"
