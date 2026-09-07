"""Time-weighted trajectory classification; position alone cannot prove wings-level."""
from dataclasses import dataclass, asdict
import csv
import json
from pathlib import Path
import numpy as np


@dataclass
class Thresholds:
    turn_rate_deg_s: float = 3.0
    vertical_speed_m_s: float = 0.5
    bank_deg: float = 5.0
    min_groundspeed_m_s: float = 5.0
    max_gap_s: float = 5.0
    min_pattern_s: float = 8.0

    def validate(self):
        if any(not np.isfinite(v) or v <= 0 for v in asdict(self).values()):
            raise ValueError("All thresholds must be finite and positive")


def read_path(path):
    with open(path, newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        names = reader.fieldnames or []
        required = {"t_s", "x_m", "y_m", "z_m"}
        if not required.issubset(names):
            raise ValueError(f"{Path(path).name}: expected CSV columns {sorted(required)}; coordinates must be local ENU metres")
        rows = list(reader)
    if len(rows) < 3:
        raise ValueError("A flight requires at least three samples")
    data = {k: np.array([float(row[k]) for row in rows]) for k in names if k in required or k == "roll_deg"}
    if not all(np.isfinite(v).all() for v in data.values()):
        raise ValueError("Missing, NaN and infinite samples are not accepted")
    if np.any(np.diff(data["t_s"]) <= 0):
        raise ValueError("Timestamps must be strictly increasing; split separate flights into separate files")
    return data


def analyze(data, thresholds=None):
    cfg = thresholds or Thresholds()
    cfg.validate()
    t = data["t_s"]
    dt = np.diff(t)
    p = np.column_stack([data[k] for k in ("x_m", "y_m", "z_m")])
    if len(t) < 3 or np.any(dt <= 0) or not np.isfinite(p).all() or not np.isfinite(t).all():
        raise ValueError("Expected at least 3 finite samples and strictly increasing time")
    v = np.diff(p, axis=0) / dt[:, None]
    speed = np.linalg.norm(v[:, :2], axis=1)
    headings = np.arctan2(v[:, 1], v[:, 0])
    turn = np.zeros(len(dt))
    # Wrapped differences avoid a false turn when heading crosses +/-180 degrees.
    dhead = np.arctan2(np.sin(np.diff(headings)), np.cos(np.diff(headings)))
    turn[1:] = np.rad2deg(dhead / ((dt[1:] + dt[:-1]) / 2))
    turn[0] = turn[1]
    unknown = (dt > cfg.max_gap_s) | (speed < cfg.min_groundspeed_m_s)
    # Adjacent invalid intervals cannot supply a valid heading-rate estimate.
    unknown[1:] |= (dt[:-1] > cfg.max_gap_s) | (speed[:-1] < cfg.min_groundspeed_m_s)
    unknown[0] |= unknown[1]
    labels = []
    for i in range(len(dt)):
        vertical = "climb" if v[i, 2] > cfg.vertical_speed_m_s else "descent" if v[i, 2] < -cfg.vertical_speed_m_s else "level"
        direction = "left" if turn[i] > cfg.turn_rate_deg_s else "right" if turn[i] < -cfg.turn_rate_deg_s else "straight"
        label = "straight_level" if direction == "straight" and vertical == "level" else f"{direction}_{vertical}"
        if label == "straight_level" and "roll_deg" in data and max(abs(data["roll_deg"][i]), abs(data["roll_deg"][i + 1])) > cfg.bank_deg:
            label = "banked_straight_level"
        labels.append("unknown" if unknown[i] else label)
    duration = float(dt.sum())
    seconds = {label: float(dt[np.array(labels) == label].sum()) for label in sorted(set(labels))}
    segments = []
    start = 0
    for end in range(1, len(labels) + 1):
        if end == len(labels) or labels[end] != labels[start]:
            segments.append({"label": labels[start], "start_s": float(t[start]), "end_s": float(t[end]), "duration_s": float(t[end] - t[start]), "heading_change_deg": float(np.sum(turn[start:end] * dt[start:end]))})
            start = end
    patterns = []
    for seg in segments:
        if seg["duration_s"] < cfg.min_pattern_s:
            continue
        if seg["label"].startswith(("left_", "right_")) and abs(seg["heading_change_deg"]) >= 300:
            patterns.append({**seg, "pattern": "loiter_candidate" if seg["label"].endswith("level") else "spiral_candidate"})
    return {"duration_s": duration, "straight_level_percent": 100 * seconds.get("straight_level", 0) / duration,
            "coverage_percent": 100 * (duration - seconds.get("unknown", 0)) / duration,
            "classification_basis": "trajectory + measured roll" if "roll_deg" in data else "trajectory only; wings-level attitude unverified",
            "seconds": seconds, "percent": {k: 100 * v / duration for k, v in seconds.items()},
            "segments": segments, "patterns": patterns, "thresholds": asdict(cfg)}


def analyze_files(paths, thresholds=None):
    flights = [{"file": str(p), **analyze(read_path(p), thresholds)} for p in paths]
    if not flights:
        raise ValueError("Choose at least one CSV flight")
    total = sum(f["duration_s"] for f in flights)
    seconds = {}
    for flight in flights:
        for k, v in flight["seconds"].items():
            seconds[k] = seconds.get(k, 0) + v
    return {"flights": flights, "duration_s": total, "straight_level_percent": 100 * seconds.get("straight_level", 0) / total,
            "percent": {k: 100 * v / total for k, v in seconds.items()}}


def save_report(report, path):
    Path(path).write_text(json.dumps(report, indent=2, allow_nan=False), encoding="utf-8")
