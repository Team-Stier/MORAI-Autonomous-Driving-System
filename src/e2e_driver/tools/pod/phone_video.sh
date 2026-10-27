#!/usr/bin/env bash
# phone_video.sh IN.mp4 OUT.mp4 [MiB] [MAX_BYTES]: IN (lap_video.py make, 1x, no dashboard) re-encoded for a phone under MiB
# (27, so under MAX_BYTES 30 MB): 540 p H.264 at the bitrate that fits the duration, faststart; again at 85 % if one pass overshoots.
set -euo pipefail
in=$1; out=$2; mib=${3:-27}; max=${4:-30000000}
d=$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$in")
kb=$(python3 -c "print(int($mib * 8 * 1024 / $d))")
for f in 1 0.85; do
  b=$(python3 -c "print(int($kb * $f))")
  ffmpeg -y -loglevel error -i "$in" -vf scale=-2:540 -c:v libx264 -preset medium -b:v ${b}k -maxrate $((b * 2))k \
    -bufsize $((b * 2))k -pix_fmt yuv420p -movflags +faststart -an "$out"
  (( $(stat -c %s "$out") < max )) && break
done
echo "{\"phone_mp4\": \"$(basename "$out")\", \"mb\": $(python3 -c "print(round($(stat -c %s "$out") / 1e6, 1))"), \"kbps\": $b, \"duration_s\": $d}"
