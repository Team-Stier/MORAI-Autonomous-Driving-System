# Paths on winpc WSL for the closed-loop evaluation stack. Everything this stack writes lives under $EVAL; the
# collection environment ($T: ws-base, build-root, win-1/2) is only read (build-root/upper is a read-only overlay layer).
T=/home/taeyun_kim/.claude-morai-wslros-20261018
ROOTFS=/home/taeyun_kim/.codex/morai-mpc-runsh-wsl-20261010-a1/noetic-rootfs   # Noetic rootfs (python 3.8, cv2 4.2)
LOWER="$T/build-root/upper:$ROOTFS"                                                # + the collection's rosdep/apt layer
EVAL=${EVAL:-$HOME/morai-eval}
WS=${WS:-$EVAL/ws/MORAI-Autonomous-Driving-System}                                 # synced copy of this branch
TRAIN=${MORAI_VLA_TRAIN:-$HOME/morai-student/train}                                 # training code (morai_vla)
VENV=${VENV:-$HOME/claude-bench/venv-teacher}                                       # training venv (torch, transformers)
SHARE=/mnt/d/share                                                                  # bags and step-0 data
