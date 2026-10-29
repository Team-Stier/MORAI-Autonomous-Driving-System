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
        laps.append(r)
    fin = [r for r in laps if r.get("finished")]
    counts = Counter()
    for r in laps:
        counts.update(r.get("counts") or {})
    lines = Counter()
    for r in laps:
        lines.update(r.get("signal_by_line") or {})
    s = dict(version=version, laps=len(laps), finished=len(fin), failed=len(laps) - len(fin),
             score_s=stats([r["score_s"] for r in fin if r.get("score_s") is not None]),
             score_s_all=stats([r["score_s"] for r in laps if r.get("score_s") is not None]),
             duration_mean_s=(stats([r["lap_s"] for r in fin if r.get("lap_s") is not None]) or {}).get("mean"),
             penalty_mean_s=(stats([r["penalty_s"] or 0 for r in fin]) or {}).get("mean"),
             violations=dict(sorted(counts.items())),
             signal_by_line=dict(sorted(lines.items())),   # "<osm stop line>:<signal_violation|signal_unverified>"
             rows=[{k: r.get(k) for k in ("file", "seed", "scene", "end", "lap_s", "penalty_s", "returns", "score_s", "counts", "rho_end", "cmd_hz", "signal_by_line")}
                   for r in laps])
    (root / f"{version}.json").write_text(json.dumps(s, ensure_ascii=False, indent=1))
    return s


def table(root):
    head = ("| 버전 | 랩 | 완주 | 실패 | score_s 평균 (완주) | min | max | 주행시간 평균 | 감점 평균 | 위반 종류별 | 정지선별 신호 |\n"
            "|---|---|---|---|---|---|---|---|---|---|---|\n")
    rows = []
    for f in sorted(root.glob("v*.json"), key=lambda p: int(re.sub(r"\D", "", p.stem) or 0)):
        s = json.loads(f.read_text()); sc = s.get("score_s") or {}
        g = lambda x: "-" if x is None else f"{x}"
        rows.append(f"| {s['version']} | {s['laps']} | {s['finished']} | {s['failed']} | {g(sc.get('mean'))} | {g(sc.get('min'))} | "
                    f"{g(sc.get('max'))} | {g(s.get('duration_mean_s'))} | {g(s.get('penalty_mean_s'))} | "
                    f"{', '.join(f'{k} {v}' for k, v in s['violations'].items()) or '-'} | "
                    f"{', '.join(f'{k} {v}' for k, v in (s.get('signal_by_line') or {}).items()) or '-'} |")
    (root.parent / "RESULTS.md").write_text("# 캠페인 버전별 랩 결과 (campaign_results.py가 갱신)\n\n"
                                            "score_s = 실제 주행 시간 + 채점 패널티 + 복귀×15 s. 상세: results/vN.json\n\n" + head + "\n".join(rows) + "\n")


if __name__ == "__main__":
    root, version = Path(sys.argv[1]).resolve(), sys.argv[2]
    s = aggregate(root, version)
    table(root)
    print(f"{version}: n={s['laps']} finished={s['finished']} score_s={s['score_s']} violations={s['violations']} signal={s['signal_by_line']}")
