#!/usr/bin/env python3
"""bundle.py pack TAG | fetch TAG: the closed-loop pod's files as ~1 GB shards in B2 (b2://morai2026-alpamayo/podimg/TAG/),
so any fresh 5090 host (RunPod, vast, Brev; no network volume) gets them in minutes. Python stdlib only.

pack  (on a pod booted from the network volume, setup.sh done): ROOTS -> regular files bin-packed by size into shards
      (tar | gzip -1; .safetensors / .tar uncompressed), every other entry (symlinks, directories) into links.tgz,
      MANIFEST (file bytes), then b2 sync. Left out: the competition account (MORAI Version/V2AccountInfo.byte and the
      login prefs morai/config.tar: eval_pod.sh copies them from the Mac's ~/.morai-secrets at each boot), the base VLM's
      weights (the engine reads config + tokenizer from it, every weight from the delivery), results, apt, logs.
fetch (a fresh host's start command, eval_pod.sh BUNDLE=TAG): every shard of MANIFEST streamed into tar -x under /,
      SHARDS_AT_ONCE (16) at a time, links.tgz last; B2 keys from the environment or PID 1's (the image has no curl).
"""
import base64, json, os, subprocess, sys, threading, time, urllib.request
from concurrent.futures import ThreadPoolExecutor

BUCKET, SHARD = "morai2026-alpamayo", 1 << 30
HF = "workspace/models/hf/hub/models--Qwen--Qwen3-VL-4B-Instruct"
ROOTS = ["workspace/rootfs", "workspace/MORAI-Autonomous-Driving-System", "workspace/Alpamayo-Cosmos", "workspace/uv-python",
         "workspace/venv-rl", "workspace/venv-tools", "workspace/venv-engine", "workspace/cache", "workspace/models/wowmayo_final",
         f"{HF}/refs", f"{HF}/snapshots", "workspace/morai/files.sha256", "root/Workspaces/scratches/morai"]
SKIP = ("/V2AccountInfo.byte", ".safetensors.lock")


def pack(tag):
    out = "/root/bundle"
    os.makedirs(out, exist_ok=True)
    files, other = [], []
    for root in ROOTS:
        if os.path.isfile("/" + root):
            files.append((os.path.getsize("/" + root), "/" + root))
        for d, dirs, names in os.walk("/" + root):
            other += [os.path.join(d, x) for x in dirs if os.path.islink(os.path.join(d, x))]
            dirs[:] = [x for x in dirs if not os.path.islink(os.path.join(d, x))]
            other.append(d)
            for x in names:
                p = os.path.join(d, x)
                if p.endswith(SKIP) or (root.startswith(HF) and "model-" in x and x.endswith(".safetensors")):
                    continue
                if os.path.islink(p) and not root.startswith(HF):
                    other.append(p)
                else:                                      # the HF snapshot's files are links into blobs: take the file
                    files.append((os.path.getsize(p), p))
    bins = {True: [], False: []}                           # raw (incompressible) / gzip
    for size, p in sorted(files, reverse=True):
        b = bins[p.endswith((".safetensors", ".tar"))]
        tgt = min(b, key=lambda x: x[0]) if b and min(x[0] for x in b) + size <= SHARD else None
        if tgt is None:
            tgt = [0, []]; b.append(tgt)
        tgt[0] += size; tgt[1].append(p)
    jobs = [(f"raw{k:02d}.tar", ps, False) for k, (_, ps) in enumerate(bins[True])] + \
           [(f"gz{k:02d}.tgz", ps, True) for k, (_, ps) in enumerate(bins[False])] + [("links.tgz", other, True)]

    def tar(job):
        name, paths, gz = job
        lst = f"/tmp/bundle.{name}.lst"
        open(lst, "w").write("".join(p.lstrip("/") + "\n" for p in paths))
        deref = name != "links.tgz" and any(p.startswith("/" + HF) for p in paths)    # file bins only hold regular files
        cmd = ["tar", "-C", "/", "--no-recursion", "-T", lst, "-hcf" if deref else "-cf", "-"]
        with open(f"{out}/{name}", "wb") as f:
            if gz:
                t = subprocess.Popen(cmd, stdout=subprocess.PIPE)
                subprocess.run(["gzip", "-1"], stdin=t.stdout, stdout=f, check=True); t.wait()
            else:
                subprocess.run(cmd, stdout=f, check=True)
        return name, os.path.getsize(f"{out}/{name}")
    with ThreadPoolExecutor(24) as ex:
        sizes = list(ex.map(tar, jobs))
    open(f"{out}/MANIFEST", "w").write("".join(f"{n} {s}\n" for n, s in sizes))
    print(f"{len(sizes)} shards, {sum(s for _, s in sizes) / 1e9:.1f} GB (files {sum(s for s, _ in files) / 1e9:.1f} GB)", flush=True)
    env = dict(os.environ, **keys())
    subprocess.run(["/workspace/venv-tools/bin/b2", "sync", "--threads", "32", "--no-progress", "--delete", out + "/",
                    f"b2://{BUCKET}/podimg/{tag}/"], env=env, check=True)


def keys():
    k = {n: v for n, v in os.environ.items() if n.startswith("B2_APPLICATION_KEY")}
    try:
        k.update(l.split("=", 1) for l in open("/proc/1/environ", "rb").read().decode().split("\0") if l.startswith("B2_APPLICATION_KEY"))
    except OSError:
        pass
    return k


def fetch(tag):
    k = keys()
    auth = base64.b64encode(f"{k['B2_APPLICATION_KEY_ID']}:{k['B2_APPLICATION_KEY']}".encode()).decode()
    a = json.load(urllib.request.urlopen(urllib.request.Request(
        "https://api.backblazeb2.com/b2api/v2/b2_authorize_account", headers={"Authorization": "Basic " + auth}), timeout=30))
    url, tok = f"{a['downloadUrl']}/file/{BUCKET}/podimg/{tag}", a["authorizationToken"]

    def get(name):
        for n in range(5):
            try:
                return urllib.request.urlopen(urllib.request.Request(f"{url}/{name}", headers={"Authorization": tok}), timeout=60)
            except OSError as e:
                err = e; time.sleep(2 + 3 * n)
        raise err
    t0, failed = time.time(), []
    manifest = [l.split() for l in get("MANIFEST").read().decode().splitlines() if l.strip()]

    def one(m):
        name, t = m[0], time.time()
        for attempt in range(3):
            tar = subprocess.Popen(["tar", "-xzf" if name.endswith(".tgz") else "-xf", "-", "-C", "/"], stdin=subprocess.PIPE)
            try:
                r = get(name)
                while b := r.read(1 << 22):
                    tar.stdin.write(b)
                tar.stdin.close()
                if tar.wait() == 0:
                    return int(m[1]) / 1e6 / (time.time() - t)
            except OSError as e:
                print(f"[fetch] {name}: {e}", flush=True)
                tar.kill()
        failed.append(name)
    with ThreadPoolExecutor(int(os.environ.get("SHARDS_AT_ONCE", 16))) as ex:
        rates = list(ex.map(one, [m for m in manifest if m[0] != "links.tgz"]))
    one(next(m for m in manifest if m[0] == "links.tgz"))
    total, ok = sum(int(m[1]) for m in manifest), sorted(r for r in rates if r) or [0]
    print(f"[fetch] {len(manifest)} shards, {total / 1e9:.1f} GB in {time.time() - t0:.0f} s ({total / 1e6 / (time.time() - t0):.0f} MB/s"
          f", one stream ~{ok[len(ok) // 2]:.0f} MB/s)" + (f" FAILED {failed}" if failed else ""), flush=True)
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    {"pack": pack, "fetch": fetch}[sys.argv[1]](sys.argv[2])
