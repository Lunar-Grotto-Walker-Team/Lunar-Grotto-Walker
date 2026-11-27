"""分析 lgw_3d_tracker JSONL 跟踪日志。

用法: python3 /tmp/analyze_trace.py <jsonl路径> [--tail N]
输出:
  - 输入/观测统计(每相机)
  - 轨迹生命周期(id 首次/最后出现、最终 source_cameras)
  - 关联统计(match 距离分布、created/unmatched 次数)
  - 跨相机一致性:同一目标 head/arm 观测的 odom 位置差(相邻 <300ms)
"""
import json
import sys
import numpy as np

path = sys.argv[1]
tail = None
if "--tail" in sys.argv:
    tail = int(sys.argv[sys.argv.index("--tail") + 1])

records = []
with open(path, encoding="utf-8") as f:
    for line in f:
        line = line.strip()
        if not line:
            continue
        try:
            r = json.loads(line)
        except json.JSONDecodeError:
            continue
        if "seq" in r:
            records.append(r)
if tail:
    records = records[-tail:]

print(f"ticks: {len(records)}")
if not records:
    sys.exit(0)

t0, t1 = records[0]["t_ms"], records[-1]["t_ms"]
print(f"时间窗: {t0/1000:.1f}s → {t1/1000:.1f}s ({(t1-t0)/1000:.1f}s), "
      f"平均 tick {len(records)/max(1e-9,(t1-t0)/1000):.2f} Hz")

# --- 输入统计 -------------------------------------------------------
inp = {}
dets_hist = {}
for r in records:
    for i in r.get("inputs", []):
        cam = i["cam"]
        inp.setdefault(cam, {"ticks": 0, "dets": 0, "adapted": 0, "zero": 0})
        inp[cam]["ticks"] += 1
        inp[cam]["dets"] += i.get("dets", 0)
        inp[cam]["adapted"] += i.get("adapted", 0)
        if i.get("dets", 0) == 0:
            inp[cam]["zero"] += 1
        dets_hist[i.get("dets", 0)] = dets_hist.get(i.get("dets", 0), 0) + 1
print("\n== 输入(每个相机) ==")
for cam, s in sorted(inp.items()):
    print(f"  {cam:5s} ticks={s['ticks']:5d} dets={s['dets']:5d} "
          f"adapted={s['adapted']:5d} 零检测tick={s['zero']}")
print(f"  检测数分布: {dict(sorted(dets_hist.items()))}")

# --- 观测统计 -------------------------------------------------------
obs_by_cam = {}
both_cam_ticks = 0
for r in records:
    cams = set()
    for o in r.get("obs", []):
        cam = o["cam"]
        cams.add(cam)
        obs_by_cam.setdefault(cam, []).append(o)
    if {"head", "arm"} <= cams:
        both_cam_ticks += 1
print("\n== 观测(成功转到 odom) ==")
for cam, os_ in sorted(obs_by_cam.items()):
    print(f"  {cam:5s} obs={len(os_)}")
print(f"  同一 tick 内同时有 head+arm 观测: {both_cam_ticks} 次")

# --- 关联统计 -------------------------------------------------------
matches, created, unmatched = [], [], []
for r in records:
    for m in r.get("matches", []):
        o = r["obs"][m["obs"]]
        matches.append((o["cam"], m["track_id"], m["dist"], o["conf"]))
    for m in r.get("tentative_matches", []):
        o = r["obs"][m["obs"]]
        matches.append((o["cam"], m["track_id"], m["dist"], o["conf"]))
    for c in r.get("created", []):
        o = r["obs"][c["obs"]]
        created.append((r["seq"], r["t_ms"], o["cam"], c["track_id"]))
    if r.get("unmatched"):
        unmatched.append((r["seq"], len(r["unmatched"])))

print("\n== 关联 ==")
print(f"  matched obs: {len(matches)}  created tracks: {len(created)}  "
      f"最终未匹配 tick: {len(unmatched)}")
if matches:
    d = np.array([m[2] for m in matches])
    print(f"  match 距离 m: p50={np.percentile(d,50):.4f} "
          f"p90={np.percentile(d,90):.4f} max={d.max():.4f}")
    for cam in ("head", "arm"):
        dd = np.array([m[2] for m in matches if m[0] == cam])
        if dd.size:
            print(f"    {cam:5s} n={dd.size:5d} p50={np.percentile(dd,50):.4f} "
                  f"p90={np.percentile(dd,90):.4f} max={dd.max():.4f}")
if created:
    print("  新建轨迹事件(前 20):")
    for c in created[:20]:
        print(f"    seq={c[0]} t={c[1]/1000:.1f}s cam={c[2]} id={c[3]}")

# --- 轨迹 id 时间线 -------------------------------------------------
track_ids = {}
for r in records:
    for t in r.get("tracks", []):
        s = track_ids.setdefault(t["id"], {
            "first": r["t_ms"], "last": r["t_ms"], "max_hits": 0,
            "cams": set(), "statuses": set()})
        s["last"] = r["t_ms"]
        s["max_hits"] = max(s["max_hits"], t["hits"])
        s["cams"].update(t.get("cams") or [])
        s["statuses"].add(t.get("task_state", t.get("status", "?")))
print("\n== 轨迹时间线 ==")
for tid, s in sorted(track_ids.items()):
    print(f"  id={tid} {s['first']/1000:.1f}s→{s['last']/1000:.1f}s "
          f"max_hits={s['max_hits']} cams={sorted(s['cams'])} "
          f"task_state={sorted(s['statuses'])}")

# --- task_state 分布 ------------------------------------------------
status_count = {}
for r in records:
    for t in r.get("tracks", []):
        key = (t["id"], t.get("task_state", t.get("status", "?")))
        status_count[key] = status_count.get(key, 0) + 1
print("\n== task_state 分布(按轨迹) ==")
by_track = {}
for (tid, st), n in status_count.items():
    by_track.setdefault(tid, {})[st] = n
for tid in sorted(by_track):
    total = sum(by_track[tid].values())
    parts = ", ".join(f"{st}={n}({n/total*100:.1f}%)"
                      for st, n in sorted(by_track[tid].items()))
    print(f"  id={tid} 共 {total} tick: {parts}")

# --- 同帧近邻观测抑制 -----------------------------------------------
suppressed = 0
suppressed_ticks = 0
for r in records:
    s = r.get("suppressed") or []
    if s:
        suppressed_ticks += 1
        suppressed += len(s)
print("\n== 同帧近邻观测抑制 ==")
if suppressed:
    print(f"  抑制观测 {suppressed} 条,涉及 {suppressed_ticks} 个 tick"
          f"({suppressed_ticks/len(records)*100:.0f}% 的 tick)")
else:
    print("  无")

# --- 影子轨迹去重 ---------------------------------------------------
dedupe = []
for r in records:
    for d in r.get("dedupe_removed") or []:
        dedupe.append((r["seq"], r["t_ms"], d))
print("\n== 影子轨迹去重 ==")
if dedupe:
    print(f"  删除次数={len(dedupe)}")
    by_pair = {}
    for _, _, d in dedupe:
        key = (d.get("kept_id"), d.get("removed_id"))
        by_pair[key] = by_pair.get(key, 0) + 1
    for (kept, removed), n in sorted(by_pair.items()):
        print(f"    kept={kept} removed={removed} ×{n}")
    print("  前 10 次:")
    for seq, t_ms, d in dedupe[:10]:
        print(f"    seq={seq} t={t_ms/1000:.1f}s {d}")
else:
    print("  无(未出现需要去重的轨迹)")

# --- 跨相机位置一致性 ------------------------------------------------
# 取相邻 head/arm 观测(时间差 ≤300ms),比较 odom 位置
obs = []
for r in records:
    for o in r.get("obs", []):
        obs.append((o["stamp_ms"], o["cam"], np.array([o["x"], o["y"], o["z"]])))
obs.sort(key=lambda x: x[0])
pairs = []
for i, (ts, cam, p) in enumerate(obs):
    for ts2, cam2, p2 in obs[i + 1:]:
        if ts2 - ts > 300:
            break
        if cam != cam2:
            pairs.append(np.linalg.norm(p - p2))
print("\n== 跨相机一致性(相邻 ≤300ms 的 head/arm 观测对) ==")
if pairs:
    a = np.array(pairs)
    print(f"  对数={a.size} 距离 m: p50={np.percentile(a,50):.4f} "
          f"p90={np.percentile(a,90):.4f} p99={np.percentile(a,99):.4f} max={a.max():.4f}")
    for th in (0.05, 0.10, 0.15, 0.30):
        print(f"    ≤{th:.2f}m: {(a<=th).mean()*100:5.1f}%")
else:
    print("  无配对(head/arm 观测未在 300ms 内交替出现)")
