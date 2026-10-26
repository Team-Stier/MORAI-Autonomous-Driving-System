#!/usr/bin/env bash
# create_pod.sh volume DC GB | pod DC VOLUME_ID [NAME]: RunPod network volume / actor pod (RTX 5090 x1). PAID: run only
# with the user's OK. Prints the request and asks before sending unless YES=1. RUNPOD_API_KEY from the environment.
#   pod: GraphQL podFindAndDeployOnDemand with minVcpuCount (REST minVCPUPerGPU said "no instances" while stock was
#   there, 10/26), image ros:noetic-ros-base-focal (ROS Noetic native; MORAI's Vulkan + GLX come in with
#   NVIDIA_DRIVER_CAPABILITIES=all), the volume at /workspace, start command = setup.sh then the idle watchdog.
#   Env: VCPU (32), MEM_GB (60), CLOUD (SECURE | COMMUNITY), DPH_USD (watchdog rate, 1.19), BUDGET_USD (watchdog cap, 10)
set -euo pipefail
: "${RUNPOD_API_KEY:?}"
send() {  # method url json
  echo "$1 $2"; echo "$3" | python3 -m json.tool
  [[ ${YES:-0} == 1 ]] || { read -r -p "send (paid)? [y/N] " ok; [[ $ok == y ]] || exit 1; }
  curl -s -X "$1" "$2" -H 'content-type: application/json' -H "Authorization: Bearer $RUNPOD_API_KEY" -d "$3"; echo
}
case ${1:-} in
  volume)
    send POST https://rest.runpod.io/v1/networkvolumes "{\"name\": \"wowmayo-rl\", \"size\": ${3:-100}, \"dataCenterId\": \"$2\"}" ;;
  pod)
    start="bash /workspace/MORAI-Autonomous-Driving-System/src/e2e_driver/tools/pod/setup.sh > /workspace/setup.log 2>&1; \
DPH_USD=${DPH_USD:-1.19} BUDGET_USD=${BUDGET_USD:-10} nohup bash /workspace/Alpamayo-Cosmos/tools/vast/idle_watchdog.sh > /dev/null 2>&1 & sleep infinity"
    q=$(python3 - "$2" "$3" "${4:-wowmayo-rl-actor}" "$start" <<'PY'
import json, os, sys
dc, vol, name, start = sys.argv[1:5]
inp = dict(cloudType=os.environ.get("CLOUD", "SECURE"), gpuCount=1, gpuTypeId="NVIDIA GeForce RTX 5090",
           minVcpuCount=int(os.environ.get("VCPU", 32)), minMemoryInGb=int(os.environ.get("MEM_GB", 60)), name=name,
           imageName="ros:noetic-ros-base-focal", containerDiskInGb=60, volumeInGb=0, networkVolumeId=vol, dataCenterId=dc,
           volumeMountPath="/workspace", ports="22/tcp", supportPublicIp=True, dockerArgs=f"bash -c {json.dumps(start)}",
           env=[{"key": "NVIDIA_DRIVER_CAPABILITIES", "value": "all"}, {"key": "NVIDIA_VISIBLE_DEVICES", "value": "all"}])
body = "{" + ", ".join(f"{k}: {json.dumps(v) if not isinstance(v, list) else '[' + ', '.join('{key: %s, value: %s}' % (json.dumps(e['key']), json.dumps(e['value'])) for e in v) + ']'}"
                       for k, v in inp.items()).replace('cloudType: "SECURE"', "cloudType: SECURE").replace('cloudType: "COMMUNITY"', "cloudType: COMMUNITY") + "}"
print(json.dumps({"query": "mutation { podFindAndDeployOnDemand(input: %s) { id machineId costPerHr } }" % body}))
PY
)
    send POST https://api.runpod.io/graphql "$q" ;;
  *) sed -n 2,7p "$0"; exit 1 ;;
esac
