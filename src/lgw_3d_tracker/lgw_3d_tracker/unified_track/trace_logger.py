"""逐 tick 跟踪日志(JSONL + 周期汇总)。

日志目的:回答"同一目标被 head/arm 两相机分别看到时,是否落到同一条
track"。每 tick 一条 JSON,含:

  inputs  每个相机本 tick 的输入(原始检测数 / 成功转到 odom 的观测数)
  obs[]   所有观测(相机、类别、置信度、odom 位置)
  matches[] / tentative_matches[] / created[] / unmatched[]  关联明细
  tracks[] 本 tick 结束后的轨迹快照(id/status/hits/source_cameras/位置)

汇总行(默认每 5s)同时写 console 和 `<base>.summary.log`:
  cross  = source_cameras 同时含两台相机的 track(双相机合并成功)
  single = 只有一台相机贡献的 track(可能重复轨迹 / 单相机可见)

字段语义:
  - `dist` 是 KF update **前**的观测-轨迹 3D 距离(米)
  - `t_ms` 是节点启动后的相对毫秒,与 topic stamp 的绝对时间无关
"""
import json
import os
import time
from typing import Callable, Optional


def cam_label(frame_id: str) -> str:
    """相机 frame_id → 短标签(head / arm),日志里用。"""
    if not frame_id:
        return "unknown"
    if "head" in frame_id:
        return "head"
    if "arm" in frame_id:
        return "arm"
    return frame_id


class TraceLogger:
    """写 JSONL 逐 tick 日志 + 周期汇总。

    Args:
      log_dir: 日志目录(不存在会自动创建)
      tag: 文件名后缀,便于区分测试轮次(如 "_crosscam")
      summary_period_sec: 汇总输出周期(按节点相对时间,秒)
      sink: 可选回调,接收汇总字符串(通常传 node.get_logger().info)
    """

    def __init__(
        self,
        log_dir: str,
        tag: str = "",
        summary_period_sec: float = 5.0,
        sink: Optional[Callable[[str], None]] = None,
    ):
        os.makedirs(log_dir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        base = os.path.join(log_dir, f"lgw_3d_tracker_{stamp}{tag}")
        self.jsonl_path = base + ".jsonl"
        self.summary_path = base + ".summary.log"
        self._fh = open(self.jsonl_path, "a", buffering=1)
        self._summary_fh = open(self.summary_path, "a", buffering=1)
        self._summary_period_sec = float(summary_period_sec)
        self._sink = sink
        self._ticks = 0
        self._obs_total = 0
        self._last_summary_s = 0.0
        self._last_record: Optional[dict] = None

    # ------------------------------------------------------------------
    def log_tick(self, record: dict) -> None:
        """写一条 tick 记录(JSONL,行缓冲,进程被杀也不丢已写行)。"""
        self._fh.write(json.dumps(record, ensure_ascii=False) + "\n")
        self._ticks += 1
        self._obs_total += len(record.get("obs", []))
        self._last_record = record
        t_s = record.get("t_ms", 0) / 1000.0
        if t_s - self._last_summary_s >= self._summary_period_sec:
            self._last_summary_s = t_s
            self._emit_summary(record)

    def log_event(self, event: str, **fields) -> None:
        """写一条非 tick 事件(启动/关闭/参数),便于日志自解释。"""
        rec = {"event": event, "wall": time.strftime("%Y-%m-%dT%H:%M:%S")}
        rec.update(fields)
        self._fh.write(json.dumps(rec, ensure_ascii=False) + "\n")

    # ------------------------------------------------------------------
    def _emit_summary(self, record: dict) -> None:
        cross, single = [], []
        for t in record.get("tracks", []):
            cams = list(t.get("cams") or [])
            item = (f"id={t['id']} cams={'+'.join(cams) if cams else '-'} "
                    f"{t.get('task_state', t.get('status', '?'))} "
                    f"hits={t['hits']} "
                    f"pos=({t['x']:.3f},{t['y']:.3f},{t['z']:.3f})")
            if len(set(cams)) >= 2:
                cross.append(item)
            else:
                single.append(item)

        head = (f"[TRACE_SUMMARY] t={record.get('t_ms', 0) / 1000.0:.1f}s "
                f"seq={record.get('seq')} ticks={self._ticks} "
                f"obs_total={self._obs_total} "
                f"cross={len(cross)} single={len(single)}")
        self._summary_fh.write(head + "\n")
        for it in cross:
            self._summary_fh.write(f"  cross  {it}\n")
        for it in single:
            self._summary_fh.write(f"  single {it}\n")

        if self._sink:
            extra = ""
            if cross:
                extra += f" cross=[{'; '.join(cross)}]"
            if single:
                extra += f" single=[{'; '.join(single)}]"
            self._sink(head + extra)

    # ------------------------------------------------------------------
    def close(self) -> None:
        if self._last_record is not None:
            self._emit_summary(self._last_record)
        self._fh.close()
        self._summary_fh.close()
