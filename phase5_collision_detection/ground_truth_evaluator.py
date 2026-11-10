"""
VANET Collision Warning & Traffic Management - Phase 5
Ground Truth Evaluator Module

Evaluates the performance of the Collision Detection algorithm against
an objective physical ground-truth baseline across simulation steps.

Metrics Computed:
-----------------
- True Positives (TP): Hazard physically present AND detected as hazard.
- False Positives (FP): No hazard physically present BUT detected as hazard.
- True Negatives (TN): No hazard physically present AND detected as safe/none.
- False Negatives (FN): Hazard physically present BUT NOT detected as hazard.
- Precision = TP / (TP + FP)
- Recall (Sensitivity) = TP / (TP + FN)
- F1-Score = 2 * (Precision * Recall) / (Precision + Recall)
- False Alarm Rate = FP / (FP + TN)
- Average Detection Latency = Delay from ground-truth hazard onset to detection
"""

import math
from typing import Dict, List, Tuple
import pandas as pd


class GroundTruthEvaluator:
    def __init__(
        self,
        gt_ttc_threshold: float = 6.0,
        gt_gap_threshold: float = 35.0,
        gt_closing_threshold: float = 1.0
    ):
        """
        Ground Truth Criteria for an Objective Hazard:
        An objective rear-end collision hazard physically exists between two vehicles
        in the same lane if:
           (0 < TTC <= gt_ttc_threshold) OR (gap <= gt_gap_threshold AND closing_speed >= gt_closing_threshold)
        """
        self.gt_ttc_threshold = gt_ttc_threshold
        self.gt_gap_threshold = gt_gap_threshold
        self.gt_closing_threshold = gt_closing_threshold

        self.step_evaluations: List[Dict] = []
        self.hazard_start_times: Dict[Tuple[str, str], float] = {}
        self.detection_latencies: List[float] = []

    def evaluate_step(
        self,
        current_time: float,
        pair: Tuple[str, str],
        lane_id: str,
        gap_m: float,
        closing_speed_mps: float,
        ttc_s: float,
        detected_risk: str,
        warning_generated: bool
    ):
        follower_id, leader_id = pair

        # 1. Determine physical ground truth
        has_closing = closing_speed_mps >= self.gt_closing_threshold
        is_close = gap_m <= self.gt_gap_threshold
        is_critical_ttc = (0 < ttc_s <= self.gt_ttc_threshold)

        ground_truth_hazard = is_critical_ttc or (is_close and has_closing)

        # 2. Determine system detection classification
        # Hazardous if detected as CRITICAL, HIGH, or MODERATE
        system_detected_hazard = detected_risk in ("CRITICAL", "HIGH", "MODERATE")

        # 3. Classify TP, FP, TN, FN
        tp = fp = tn = fn = 0
        if ground_truth_hazard and system_detected_hazard:
            tp = 1
        elif not ground_truth_hazard and system_detected_hazard:
            fp = 1
        elif not ground_truth_hazard and not system_detected_hazard:
            tn = 1
        elif ground_truth_hazard and not system_detected_hazard:
            fn = 1

        # 4. Track Detection Latency
        if ground_truth_hazard:
            if pair not in self.hazard_start_times:
                self.hazard_start_times[pair] = current_time
            if system_detected_hazard and pair in self.hazard_start_times:
                latency = current_time - self.hazard_start_times[pair]
                self.detection_latencies.append(latency)
        else:
            if pair in self.hazard_start_times:
                del self.hazard_start_times[pair]

        self.step_evaluations.append({
            "timestamp": round(current_time, 2),
            "follower_id": follower_id,
            "leader_id": leader_id,
            "lane_id": lane_id,
            "gap_m": round(gap_m, 2),
            "closing_speed_mps": round(closing_speed_mps, 2),
            "ttc_s": round(ttc_s, 2) if ttc_s != float("inf") else -1.0,
            "gt_hazard": int(ground_truth_hazard),
            "detected_risk": detected_risk,
            "system_hazard": int(system_detected_hazard),
            "warning_generated": int(warning_generated),
            "TP": tp,
            "FP": fp,
            "TN": tn,
            "FN": fn
        })

    def compute_metrics(self) -> Dict[str, float]:
        df = pd.DataFrame(self.step_evaluations)
        if df.empty:
            return {
                "TP": 0, "FP": 0, "TN": 0, "FN": 0,
                "Precision": 1.0, "Recall": 1.0, "F1": 1.0,
                "FalseAlarmRate": 0.0, "MeanLatency_s": 0.0
            }

        tp = df["TP"].sum()
        fp = df["FP"].sum()
        tn = df["TN"].sum()
        fn = df["FN"].sum()

        # Handle zero denominators: report NaN / N/A, not 1.0 (100%)
        precision = (tp / (tp + fp)) if (tp + fp) > 0 else float("nan")
        recall = (tp / (tp + fn)) if (tp + fn) > 0 else (1.0 if tn > 0 and fp == 0 else float("nan"))
        if not math.isnan(precision) and not math.isnan(recall) and (precision + recall) > 0:
            f1 = (2 * precision * recall) / (precision + recall)
        else:
            f1 = float("nan")

        far = (fp / (fp + tn)) if (fp + tn) > 0 else 0.0
        mean_latency = (
            sum(self.detection_latencies) / len(self.detection_latencies)
            if self.detection_latencies else 0.0
        )

        return {
            "TP": int(tp),
            "FP": int(fp),
            "TN": int(tn),
            "FN": int(fn),
            "Precision": round(precision, 4) if not math.isnan(precision) else float("nan"),
            "Recall": round(recall, 4) if not math.isnan(recall) else float("nan"),
            "F1": round(f1, 4) if not math.isnan(f1) else float("nan"),
            "FalseAlarmRate": round(far, 4),
            "MeanLatency_s": round(mean_latency, 3)
        }

    def export_csv(self, filepath: str):
        df = pd.DataFrame(self.step_evaluations)
        df.to_csv(filepath, index=False)
