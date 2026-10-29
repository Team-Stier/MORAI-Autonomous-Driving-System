#!/usr/bin/env python3
"""campaign_results.py RESULTS_DIR VERSION: the laps of one campaign version (RESULTS_DIR/VERSION/*.json, lap_report.py's
lap_KK.json as eval_pod.sh VERSION=vN collects them) -> RESULTS_DIR/VERSION.json (laps, finished/failed counts,
score_s min/max/mean over finished laps and over all, duration mean, violations per type, per-lap rows) and the
version's row in RESULTS_DIR/../RESULTS.md (one table, a row per version, kept sorted by version number).
score_s = lap time + scorer penalties + returns x 15 s (the rules' total); a lap is failed when it did not finish
(15 min limit, start failure, interrupted) or the scorer found a competition failure."""
import json
import re
import sys
from collections import Counter
from pathlib import Path


def stats(xs):
    return dict(min=round(min(xs), 2), max=round(max(xs), 2), mean=round(sum(xs) / len(xs), 2)) if xs else None


def aggregate(root, version):
    laps = []
    for f in sorted((root / version).glob("*.json")):
        try:
            r = json.loads(f.read_text())
        except ValueError:
            continue
        r["file"] = f.name
        sr, wr = r.get("stack_rate"), r.get("world_rate")   # lap_report.py's clock check (mean rates over the lap)
        if r.get("env") is None and sr is not None and wr and abs(sr / wr - 1) > 0.30:
            r["env"], r["env_reason"] = "invalid", f"stack clock {sr} vs world {wr} sim s per wall s"
        laps.append(r)
    invalid = [r for r in laps if r.get("env") == "invalid"]      # env-invalid laps: listed, never aggregated
    laps = [r for r in laps if r.get("env") != "invalid"]
    fin = [r for r in laps if r.get("finished")]
    counts = Counter()
    for r in laps:
        counts.update(r.get("counts") or {})
    lines = Counter()
    for r in laps:
        lines.update(r.get("signal_by_line") or {})
    # S4 (static obstacle, CP3 -> CP4): laps whose route covers it and how many of them reached CP4
    # (a finished lap passed it; an unfinished lap from before the "cps" field is unknown)
    s4 = [r for r in laps if (r.get("start_cp") or 0) <= 3 and not (0 < (r.get("end_cp") or 0) <= 3)]
    s4_unknown = [r for r in s4 if not r.get("finished") and r.get("cps") is None]
    s4 = [r for r in s4 if r not in s4_unknown]
    s = dict(version=version, laps=len(laps), finished=len(fin), failed=len(laps) - len(fin),
             score_s=stats([r["score_s"] for r in fin if r.get("score_s") is not None]),
             score_s_all=stats([r["score_s"] for r in laps if r.get("score_s") is not None]),
             duration_mean_s=(stats([r["lap_s"] for r in fin if r.get("lap_s") is not None]) or {}).get("mean"),
             penalty_mean_s=(stats([r["penalty_s"] or 0 for r in fin]) or {}).get("mean"),
             violations=dict(sorted(counts.items())),
             s4_laps=len(s4), s4_passed=sum(bool(r.get("finished")) or (r.get("cps") or 0) >= 4 for r in s4), s4_unknown=len(s4_unknown),
             lat_rms_mean_m=(stats([r["lat"]["rms_m"] for r in laps if r.get("lat")]) or {}).get("mean"),
             env_invalid=[r["file"] for r in invalid],
             signal_by_line=dict(sorted(lines.items())),   # "<osm stop line>:<signal_violation|signal_unverified>"
             rows=[{k: r.get(k) for k in ("file", "seed", "scene", "end", "cps", "lap_s", "penalty_s", "returns", "score_s", "counts", "rho_end", "cmd_hz", "signal_by_line", "lat", "env", "stack_rate", "world_rate", "clock_flag")}
                   for r in laps])
    (root / f"{version}.json").write_text(json.dumps(s, ensure_ascii=False, indent=1))
    return s


def table(root):
    head = ("| 버전 | 랩 | 완주 | 실패 | S4 통과 | score_s 평균 (완주) | min | max | 주행시간 평균 | 감점 평균 | 위반 종류별 | 정지선별 신호 |\n"
            "|---|---|---|---|---|---|---|---|---|---|---|---|\n")
    rows = []
    alias = {d.resolve().name: d.name for d in root.glob("v*") if d.is_dir() and d.is_symlink()}   # results/v2 -> v2c: one row "v2"
    for f in sorted(root.glob("v*.json"), key=lambda p: int(re.sub(r"\D", "", p.stem) or 0)):
        if (root / f.stem).is_symlink():
            continue
        s = json.loads(f.read_text()); sc = s.get("score_s") or {}
        if f.stem in alias:
            s["version"] = f"{alias[f.stem]} ({f.stem})"
        g = lambda x: "-" if x is None else f"{x}"
        s4 = f"{s['s4_passed']}/{s['s4_laps']}" + (f" (+{s['s4_unknown']} ?)" if s.get("s4_unknown") else "") if "s4_laps" in s else "-"
        rows.append(f"| {s['version']} | {s['laps']} | {s['finished']} | {s['failed']} | {s4} | {g(sc.get('mean'))} | {g(sc.get('min'))} | "
                    f"{g(sc.get('max'))} | {g(s.get('duration_mean_s'))} | {g(s.get('penalty_mean_s'))} | "
                    f"{', '.join(f'{k} {v}' for k, v in s['violations'].items()) or '-'} | "
                    f"{', '.join(f'{k} {v}' for k, v in (s.get('signal_by_line') or {}).items()) or '-'} |")
    (root.parent / "RESULTS.md").write_text("# 캠페인 버전별 랩 결과 (campaign_results.py가 갱신)\n\n"
                                            "score_s = 실제 주행 시간 + 채점 패널티 + 복귀×15 s. S4 통과 = CP4 도달 랩 / S4가 경로에 있는 랩. 상세: results/vN.json\n\n" + head + "\n".join(rows) + "\n")


if __name__ == "__main__":
    root, version = Path(sys.argv[1]).resolve(), sys.argv[2]
    s = aggregate(root, version)
    table(root)
    print(f"{version}: n={s['laps']} finished={s['finished']} score_s={s['score_s']} violations={s['violations']} signal={s['signal_by_line']}")
