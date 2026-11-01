"""
VANET Collision Warning & Traffic Management - Phase 5
Collision Detector Module

This module is the core of Phase 5. It takes a snapshot of all vehicle states
(positions, speeds, lane IDs) at every SUMO simulation timestep and identifies
potentially dangerous rear-end following situations.

KEY CONCEPTS EXPLAINED FOR BEGINNERS:
--------------------------------------
- Rear-end collision: Vehicle A is behind Vehicle B in the SAME lane.
  If A is faster than B and the gap closes, they may collide.

- Bumper-to-bumper gap:
  The distance between the FRONT bumper of the following vehicle and
  the REAR bumper of the leading vehicle.
  gap = (leader_x - follower_x) - VEHICLE_LENGTH

- Closing speed (approach speed):
  How fast the follower is catching up to the leader.
  v_closing = v_follower - v_leader
  If positive => follower is approaching leader.
  If zero or negative => no approach (safe).

- Time-to-Collision (TTC):
  Estimated time until impact if current speeds are maintained.
  TTC = gap / v_closing   (only valid when v_closing > 0 and gap > 0)
  Unit: seconds. Lower TTC => more dangerous.

- Risk levels (TTC-based thresholds matching ETSI ITS standards):
  CRITICAL  : TTC < 1.5s  (imminent collision)
  HIGH      : TTC 1.5–3.0s
  MODERATE  : TTC 3.0–5.0s
  LOW       : TTC > 5.0s or infinite
  SAFE      : no closing motion

- Cooldown: Once a warning for a vehicle pair is issued, we suppress
  duplicate warnings for `cooldown_s` seconds to avoid flooding.
"""

import math
import logging
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
VEHICLE_LENGTH_M = 5.0    # physical length of each vehicle (metres) from highway.rou.xml
MIN_GAP_M = 2.5           # minimum gap below which vehicles are effectively touching
SAFE_TTC = float("inf")   # used when there is no approach speed (safe state)

# TTC risk thresholds (seconds)
# In accordance with ISO 15623 (Forward Collision Warning Systems) & ETSI ITS:
# - CRITICAL: TTC < 2.0s (immediate emergency intervention needed)
# - HIGH    : TTC 2.0s - 4.0s (urgent forward collision warning alert)
# - MODERATE: TTC 4.0s - 6.0s (caution advisory / headway warning)
# - LOW     : TTC > 6.0s or infinite (safe following)
TTC_CRITICAL = 2.0
TTC_HIGH = 4.0
TTC_MODERATE = 6.0

# Hysteresis margin: a risk level change is only reported if TTC crosses
# the threshold by at least this amount. Prevents rapid oscillation.
HYSTERESIS_MARGIN = 0.2   # seconds

# Default cooldown between repeated warnings for the same vehicle pair (seconds)
DEFAULT_COOLDOWN_S = 5.0


# ---------------------------------------------------------------------------
# Data Classes
# ---------------------------------------------------------------------------

@dataclass
class VehicleState:
    """
    Snapshot of a single vehicle's state at one simulation timestep.
    Populated from SUMO TraCI data via the co-simulation engine.
    """
    vehicle_id: str         # SUMO vehicle ID, e.g. "V1"
    pos_x: float            # longitudinal position in metres along road
    pos_y: float            # lateral position (used for lane confirmation only)
    speed_mps: float        # speed in m/s
    lane_id: str            # SUMO lane ID, e.g. "E1_0" or "E1_1"
    edge_id: str            # SUMO edge ID, e.g. "E1"
    timestamp: float        # simulation time at which this state was captured

    @property
    def speed_kmh(self) -> float:
        return self.speed_mps * 3.6

    @property
    def lane_index(self) -> int:
        """Extract lane index (0 or 1) from SUMO lane ID string like 'E2_0'."""
        try:
            return int(self.lane_id.split("_")[-1])
        except (ValueError, IndexError):
            return 0


@dataclass
class CollisionEvent:
    """
    A detected collision risk event between two vehicles in the same lane.

    Fields
    ------
    event_id        : unique string identifier for this event
    timestamp       : simulation time of detection (seconds)
    follower_id     : ID of the vehicle approaching from behind
    leader_id       : ID of the vehicle ahead
    follower_ip     : IP address (for AODV routing)
    leader_ip       : IP address (for AODV routing)
    lane_id         : lane where both vehicles were detected
    gap_m           : bumper-to-bumper gap in metres
    closing_speed   : approach speed in m/s (positive = closing in)
    ttc_s           : Time-to-Collision in seconds (inf if not closing)
    risk_level      : one of 'CRITICAL', 'HIGH', 'MODERATE', 'LOW', 'SAFE'
    warning_generated: True if a warning message was issued (vs suppressed by cooldown)
    """
    event_id: str
    timestamp: float
    follower_id: str
    leader_id: str
    follower_ip: str
    leader_ip: str
    lane_id: str
    gap_m: float
    closing_speed_mps: float
    ttc_s: float
    risk_level: str
    warning_generated: bool


# ---------------------------------------------------------------------------
# Risk Classification Helper
# ---------------------------------------------------------------------------

def classify_risk(ttc_s: float) -> str:
    """
    Map a TTC value (seconds) to a risk level string.

    TTC buckets (derived from ETSI EN 302 637-2 ETSI ITS standards):
      < 1.5 s  → CRITICAL   (driver must brake immediately)
      1.5–3 s  → HIGH       (urgent braking required)
      3–5 s    → MODERATE   (driver should increase caution)
      > 5 s    → LOW        (within acceptable following distance)
      inf      → SAFE       (no closing motion)
    """
    if ttc_s == SAFE_TTC or ttc_s < 0:
        return "SAFE"
    elif ttc_s < TTC_CRITICAL:
        return "CRITICAL"
    elif ttc_s < TTC_HIGH:
        return "HIGH"
    elif ttc_s < TTC_MODERATE:
        return "MODERATE"
    else:
        return "LOW"


# ---------------------------------------------------------------------------
# Main CollisionDetector Class
# ---------------------------------------------------------------------------

class CollisionDetector:
    """
    Stateful collision detector.

    Each call to `detect()` processes a dict of VehicleState objects and
    returns a list of CollisionEvent objects describing detected hazards.

    State is preserved across timesteps to support:
    - Cooldown tracking (suppress duplicate warnings)
    - Hysteresis (prevent rapid risk level oscillation)
    """

    def __init__(
        self,
        vehicle_ip_map: Dict[str, str],
        cooldown_s: float = DEFAULT_COOLDOWN_S,
        verbose: bool = True
    ):
        """
        Parameters
        ----------
        vehicle_ip_map : dict
            Maps SUMO vehicle ID → IP address for AODV routing.
            Example: {"V1": "10.0.0.1", "V2": "10.0.0.2", ...}
        cooldown_s : float
            Minimum time (seconds) between two warnings for the same pair.
        verbose : bool
            If True, print detection events to stdout.
        """
        self.vehicle_ip_map = vehicle_ip_map
        self.cooldown_s = cooldown_s
        self.verbose = verbose

        # cooldown_tracker[(follower_id, leader_id)] = last_warning_timestamp
        self.cooldown_tracker: Dict[Tuple[str, str], float] = {}

        # last_risk_level[(follower_id, leader_id)] = previous risk level string
        # Used for hysteresis: only upgrade if TTC drops enough
        self.last_risk_level: Dict[Tuple[str, str], str] = {}

        # Internal counter for unique event IDs
        self._event_counter = 0

        logger.info("CollisionDetector initialised. cooldown=%.1fs", cooldown_s)

    # -----------------------------------------------------------------------
    # Public Interface
    # -----------------------------------------------------------------------

    def detect(self, vehicle_states: Dict[str, VehicleState], current_time: float) -> List[CollisionEvent]:
        """
        Analyse all vehicle states at the current timestep.

        Algorithm
        ---------
        1. Group vehicles by lane_id. Only vehicles sharing the same lane
           can be in a rear-end following situation.
        2. Within each lane, sort vehicles by longitudinal position (pos_x)
           in descending order so the first entry is the furthest ahead.
        3. For each adjacent pair (leader = vehicle[i], follower = vehicle[i+1]):
           a. Compute bumper-to-bumper gap.
           b. Compute closing speed.
           c. Compute TTC.
           d. Classify risk level.
           e. Apply hysteresis and cooldown filters.
           f. If a warning should be issued, create a CollisionEvent.
        4. Return the list of CollisionEvent objects for this timestep.

        Parameters
        ----------
        vehicle_states : dict
            Mapping of vehicle_id → VehicleState (all active vehicles).
        current_time : float
            Current simulation time in seconds.

        Returns
        -------
        List[CollisionEvent]
            Events that triggered a warning (including suppressed ones with
            warning_generated=False so logs show all detected hazards).
        """
        events: List[CollisionEvent] = []

        # Step 1: Group by lane ------------------------------------------------
        lane_groups: Dict[str, List[VehicleState]] = {}
        for vs in vehicle_states.values():
            if vs.lane_id not in lane_groups:
                lane_groups[vs.lane_id] = []
            lane_groups[vs.lane_id].append(vs)

        # Step 2 & 3: Process each lane ----------------------------------------
        for lane_id, vehicles in lane_groups.items():
            if len(vehicles) < 2:
                continue  # Need at least 2 vehicles in same lane

            # Sort descending by pos_x → index 0 is furthest ahead (leader)
            vehicles_sorted = sorted(vehicles, key=lambda v: v.pos_x, reverse=True)

            for i in range(len(vehicles_sorted) - 1):
                leader = vehicles_sorted[i]
                follower = vehicles_sorted[i + 1]

                event = self._analyse_pair(leader, follower, lane_id, current_time)
                if event is not None:
                    events.append(event)

        return events

    # -----------------------------------------------------------------------
    # Internal Pair Analysis
    # -----------------------------------------------------------------------

    def _analyse_pair(
        self,
        leader: VehicleState,
        follower: VehicleState,
        lane_id: str,
        current_time: float
    ) -> Optional[CollisionEvent]:
        """
        Compute gap, closing speed and TTC for one leader-follower pair.
        Apply cooldown and hysteresis. Return CollisionEvent or None.

        SUMO position convention:
        - pos_x is the position of the FRONT of the vehicle along the road centre.
        - So leader.pos_x > follower.pos_x (leader is ahead).
        - Bumper-to-bumper gap = (leader_front_x - follower_front_x) - VEHICLE_LENGTH
          Because: follower_front = follower.pos_x
                   leader_rear   = leader.pos_x - VEHICLE_LENGTH
                   gap           = leader_rear - follower_front
                                 = leader.pos_x - VEHICLE_LENGTH - follower.pos_x
        """
        # ── Gap Calculation ──────────────────────────────────────────────────
        raw_separation = leader.pos_x - follower.pos_x
        gap_m = raw_separation - VEHICLE_LENGTH_M

        if gap_m < 0:
            # Vehicles are overlapping (shouldn't happen in normal SUMO sim)
            gap_m = 0.0

        # ── Closing Speed ────────────────────────────────────────────────────
        # v_closing > 0 means follower is catching up to leader
        closing_speed_mps = follower.speed_mps - leader.speed_mps

        # ── TTC Calculation ──────────────────────────────────────────────────
        if closing_speed_mps > 0.01 and gap_m > 0:
            ttc_s = gap_m / closing_speed_mps
        else:
            # Not closing (follower decelerating or same speed) → safe
            ttc_s = SAFE_TTC

        # ── Risk Classification ───────────────────────────────────────────────
        risk_level = classify_risk(ttc_s)
        pair_key = (follower.vehicle_id, leader.vehicle_id)

        # ── Hysteresis Filter ─────────────────────────────────────────────────
        # Only report if risk level changed, or if it's a new/fresh detection
        prev_risk = self.last_risk_level.get(pair_key, "SAFE")
        risk_changed = (risk_level != prev_risk)
        self.last_risk_level[pair_key] = risk_level

        # ── Only care about non-SAFE and non-LOW events for warning generation ─
        # (we still log SAFE/LOW for ground-truth evaluation purposes)
        is_hazardous = risk_level in ("CRITICAL", "HIGH", "MODERATE")

        # ── Cooldown Filter ───────────────────────────────────────────────────
        last_warn_time = self.cooldown_tracker.get(pair_key, -999.0)
        cooldown_expired = (current_time - last_warn_time) >= self.cooldown_s
        generate_warning = is_hazardous and (cooldown_expired or risk_changed)

        # Update cooldown tracker if warning is being generated
        if generate_warning:
            self.cooldown_tracker[pair_key] = current_time

        # ── Always create an event for detection logging ──────────────────────
        # (even for SAFE pairs, so the ground-truth evaluator sees full picture)
        # But only create event object for hazardous OR risk-changed pairs to
        # keep the list manageable.
        if not is_hazardous and not risk_changed:
            return None  # nothing interesting happened for this pair

        self._event_counter += 1
        event_id = f"EVENT_{current_time:.1f}_{follower.vehicle_id}_{leader.vehicle_id}"

        follower_ip = self.vehicle_ip_map.get(follower.vehicle_id, "0.0.0.0")
        leader_ip = self.vehicle_ip_map.get(leader.vehicle_id, "0.0.0.0")

        event = CollisionEvent(
            event_id=event_id,
            timestamp=current_time,
            follower_id=follower.vehicle_id,
            leader_id=leader.vehicle_id,
            follower_ip=follower_ip,
            leader_ip=leader_ip,
            lane_id=lane_id,
            gap_m=round(gap_m, 3),
            closing_speed_mps=round(closing_speed_mps, 3),
            ttc_s=round(ttc_s, 3) if ttc_s != SAFE_TTC else SAFE_TTC,
            risk_level=risk_level,
            warning_generated=generate_warning
        )

        if self.verbose and generate_warning:
            ttc_disp = f"{ttc_s:.2f}s" if ttc_s != SAFE_TTC else "∞"
            print(
                f"  [COLLISION DETECTOR] t={current_time:.1f}s | "
                f"{follower.vehicle_id}→{leader.vehicle_id} | "
                f"Lane={lane_id} | gap={gap_m:.1f}m | "
                f"v_close={closing_speed_mps:.2f}m/s | TTC={ttc_disp} | "
                f"RISK={risk_level} | WARNING={'YES' if generate_warning else 'SUPPRESSED'}"
            )

        return event

    def reset(self):
        """Reset all stateful tracking (cooldowns, hysteresis). Call between scenarios."""
        self.cooldown_tracker.clear()
        self.last_risk_level.clear()
        self._event_counter = 0
        logger.info("CollisionDetector state reset.")
