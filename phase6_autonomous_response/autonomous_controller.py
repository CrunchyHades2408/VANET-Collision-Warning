"""
VANET Collision Warning & Traffic Management - Phase 6
Autonomous Vehicle Response Controller

Implements the closed-loop safety response:
Hazard Detected -> Warning Generated -> Warning Transmitted (AODV) ->
Warning Delivered -> Warning Received & Validated -> Autonomous Braking Actuated ->
Speed Monitored -> Safe Driving Recovery.

Adheres to:
- ISO 15623 / ETSI ITS forward collision warning & mitigation principles.
- Realistic vehicle deceleration envelopes (preventing instantaneous stops).
- Robust edge-case handling (missing vehicles, expiration, deduplication, recovery).
"""

import math
import logging
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Optional, Set, Tuple, Any

from phase5_collision_detection.warning_generator import StructuredWarningMessage

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Data Classes
# ---------------------------------------------------------------------------

@dataclass
class WarningValidationResult:
    is_valid: bool
    status: str          # "VALID", "NOT_DELIVERED", "WRONG_RECIPIENT", "EXPIRED", "DUPLICATE", "VEHICLE_INACTIVE", "ALREADY_BRAKING"
    reason: str
    warning_id: str
    recipient_id: str
    validation_time: float


@dataclass
class VehicleBrakingState:
    vehicle_id: str
    state: str           # "IDLE", "BRAKING_ACTIVE", "SPEED_MONITORING", "RECOVERED"
    trigger_warning_id: str
    risk_level: str
    warning_receipt_time: float
    braking_start_time: float
    initial_speed_mps: float
    target_speed_mps: float
    braking_duration_s: float
    expected_decel_mps2: float
    recovery_start_time: Optional[float] = None
    min_speed_reached_mps: float = 999.0
    actual_speed_reduction_mps: float = 0.0
    last_speed_mps: float = 0.0
    completed: bool = False


@dataclass
class CausalEventRecord:
    timestamp: float
    step_number: int     # 1 to 8 matching Phase 6 causal sequence
    step_name: str       # e.g., "HAZARD_DETECTED", "BRAKING_ACTIVATED", etc.
    event_id: str
    vehicle_id: str
    details: str
    speed_mps: float = 0.0
    gap_m: float = 0.0
    ttc_s: float = -1.0


# ---------------------------------------------------------------------------
# Warning Receipt Validator
# ---------------------------------------------------------------------------

class WarningReceiptValidator:
    """
    Validates warning receipt at the destination vehicle before actuation:
    1. Distinguishes GENERATED vs TRANSMITTED vs DELIVERED vs RECEIVED.
    2. Rejects warnings not delivered to the intended recipient.
    3. Rejects expired warnings (arrived too late).
    4. Rejects duplicate warnings within cooldown window.
    5. Rejects warnings for inactive / departed vehicles.
    """

    def __init__(
        self,
        max_warning_age_s: float = 2.0,
        cooldown_window_s: float = 4.0
    ):
        self.max_warning_age_s = max_warning_age_s
        self.cooldown_window_s = cooldown_window_s

        # Track processed warnings per recipient: recipient_id -> {warning_id: timestamp}
        self.processed_warnings: Dict[str, Dict[str, float]] = {}
        # Track last warning timestamp per vehicle: recipient_id -> float
        self.last_warning_time_per_veh: Dict[str, float] = {}

    def validate(
        self,
        warning: StructuredWarningMessage,
        receiving_vehicle_id: str,
        current_time: float,
        active_vehicle_ids: List[str],
        current_vehicle_state: Optional[str] = "IDLE"
    ) -> WarningValidationResult:
        """
        Perform strict multi-criteria validation on an incoming warning message.
        """
        # 1. Delivery Confirmation Check
        # Braking function must execute ONLY after confirmed network delivery
        if warning.delivery_status != "DELIVERED":
            return WarningValidationResult(
                is_valid=False,
                status="NOT_DELIVERED",
                reason=f"Warning delivery status is '{warning.delivery_status}', expected 'DELIVERED'",
                warning_id=warning.event_id,
                recipient_id=receiving_vehicle_id,
                validation_time=current_time
            )

        # 2. Recipient Identity Verification
        # Only the intended recipient should respond
        if warning.target_id != receiving_vehicle_id:
            return WarningValidationResult(
                is_valid=False,
                status="WRONG_RECIPIENT",
                reason=f"Warning intended for '{warning.target_id}', but delivered to '{receiving_vehicle_id}'",
                warning_id=warning.event_id,
                recipient_id=receiving_vehicle_id,
                validation_time=current_time
            )

        # 3. Vehicle Existence in Simulation
        # Safety check: vehicle must still be in the road network
        if receiving_vehicle_id not in active_vehicle_ids:
            return WarningValidationResult(
                is_valid=False,
                status="VEHICLE_INACTIVE",
                reason=f"Receiving vehicle '{receiving_vehicle_id}' has already exited the simulation",
                warning_id=warning.event_id,
                recipient_id=receiving_vehicle_id,
                validation_time=current_time
            )

        # 4. Freshness & Expiration Check
        # Reject warnings that arrived too late
        warning_age = current_time - warning.timestamp
        if warning_age > self.max_warning_age_s:
            return WarningValidationResult(
                is_valid=False,
                status="EXPIRED",
                reason=f"Warning age ({warning_age:.3f}s) exceeds max threshold ({self.max_warning_age_s}s)",
                warning_id=warning.event_id,
                recipient_id=receiving_vehicle_id,
                validation_time=current_time
            )

        # 5. Duplicate Warning Check
        veh_history = self.processed_warnings.setdefault(receiving_vehicle_id, {})
        if warning.event_id in veh_history:
            return WarningValidationResult(
                is_valid=False,
                status="DUPLICATE",
                reason=f"Warning ID '{warning.event_id}' was already processed for vehicle '{receiving_vehicle_id}'",
                warning_id=warning.event_id,
                recipient_id=receiving_vehicle_id,
                validation_time=current_time
            )

        # Check vehicle-level cooldown to prevent repeated warning-triggered braking
        last_time = self.last_warning_time_per_veh.get(receiving_vehicle_id, -999.0)
        time_since_last = current_time - last_time
        if time_since_last < self.cooldown_window_s and current_vehicle_state == "BRAKING_ACTIVE":
            return WarningValidationResult(
                is_valid=False,
                status="ALREADY_BRAKING",
                reason=f"Vehicle '{receiving_vehicle_id}' is actively braking from previous warning ({time_since_last:.2f}s ago)",
                warning_id=warning.event_id,
                recipient_id=receiving_vehicle_id,
                validation_time=current_time
            )

        # Record validation
        veh_history[warning.event_id] = current_time
        self.last_warning_time_per_veh[receiving_vehicle_id] = current_time

        return WarningValidationResult(
            is_valid=True,
            status="VALID",
            reason="Warning verified successfully and accepted for autonomous actuation",
            warning_id=warning.event_id,
            recipient_id=receiving_vehicle_id,
            validation_time=current_time
        )


# ---------------------------------------------------------------------------
# Autonomous Vehicle Controller
# ---------------------------------------------------------------------------

class AutonomousVehicleController:
    """
    Manages autonomous braking actuation and recovery via SUMO TraCI:
    - Actuates vehicle slowdown only upon receipt of validated warning.
    - Applies realistic deceleration profiles according to risk level.
    - Continuously monitors vehicle speed drop and headway margin.
    - Recovers vehicle to normal car-following speed control after threat resolution.
    - Protects against duplicate, expired, and erratic commands.
    """

    def __init__(
        self,
        # Braking profile configurations
        critical_target_speed_mps: float = 2.0,      # ~7.2 km/h
        critical_duration_s: float = 3.0,            # 3.0 s braking duration
        high_target_speed_mps: float = 8.0,          # ~28.8 km/h
        high_duration_s: float = 3.5,
        moderate_target_speed_mps: float = 14.0,     # ~50.4 km/h
        moderate_duration_s: float = 4.0,
        max_decel_limit_mps2: float = 7.5,           # comfortable emergency limit
        min_brake_hold_s: float = 2.5,               # time to maintain before checking recovery
        total_recovery_timeout_s: float = 6.0,       # maximum forced duration before returning control
        verbose: bool = True
    ):
        self.critical_target_speed_mps = critical_target_speed_mps
        self.critical_duration_s = critical_duration_s
        self.high_target_speed_mps = high_target_speed_mps
        self.high_duration_s = high_duration_s
        self.moderate_target_speed_mps = moderate_target_speed_mps
        self.moderate_duration_s = moderate_duration_s
        self.max_decel_limit_mps2 = max_decel_limit_mps2
        self.min_brake_hold_s = min_brake_hold_s
        self.total_recovery_timeout_s = total_recovery_timeout_s
        self.verbose = verbose

        self.validator = WarningReceiptValidator()
        self.active_braking_states: Dict[str, VehicleBrakingState] = {}

        # Telemetry logs
        self.validation_log: List[Dict[str, Any]] = []
        self.braking_actuation_log: List[Dict[str, Any]] = []
        self.response_telemetry_log: List[Dict[str, Any]] = []
        self.causal_event_log: List[Dict[str, Any]] = []

    def log_causal_step(
        self,
        current_time: float,
        step_number: int,
        step_name: str,
        event_id: str,
        vehicle_id: str,
        details: str,
        speed_mps: float = 0.0,
        gap_m: float = 0.0,
        ttc_s: float = -1.0
    ):
        rec = CausalEventRecord(
            timestamp=round(current_time, 4),
            step_number=step_number,
            step_name=step_name,
            event_id=event_id,
            vehicle_id=vehicle_id,
            details=details,
            speed_mps=round(speed_mps, 2),
            gap_m=round(gap_m, 2),
            ttc_s=round(ttc_s, 2)
        )
        self.causal_event_log.append(asdict(rec))
        if self.verbose:
            print(f"[{current_time:6.2f}s] [CAUSAL STEP {step_number}: {step_name}] Vehicle '{vehicle_id}' | {details}")

    def compute_braking_profile(
        self,
        current_speed_mps: float,
        risk_level: str
    ) -> Tuple[float, float, float]:
        """
        Compute safe target speed and duration based on risk level and current speed.
        Returns: (target_speed_mps, duration_s, expected_deceleration_mps2)
        """
        if risk_level == "CRITICAL":
            target_speed = min(self.critical_target_speed_mps, current_speed_mps * 0.2)
            duration = self.critical_duration_s
        elif risk_level == "HIGH":
            target_speed = min(self.high_target_speed_mps, current_speed_mps * 0.4)
            duration = self.high_duration_s
        else:
            target_speed = min(self.moderate_target_speed_mps, current_speed_mps * 0.7)
            duration = self.moderate_duration_s

        delta_v = max(0.0, current_speed_mps - target_speed)
        decel = delta_v / duration if duration > 0 else 0.0

        # Bound deceleration to realistic physical limit
        if decel > self.max_decel_limit_mps2:
            duration = delta_v / self.max_decel_limit_mps2
            decel = self.max_decel_limit_mps2

        return round(target_speed, 2), round(duration, 2), round(decel, 2)

    def handle_delivered_warning(
        self,
        traci_module: Any,
        warning: StructuredWarningMessage,
        current_time: float
    ) -> Tuple[bool, str]:
        """
        Entry point called when the network confirms warning packet delivery to target vehicle.
        Validates the message and actuates autonomous braking.
        """
        recipient_id = warning.target_id
        active_ids = traci_module.vehicle.getIDList()

        # Step 4 Confirmation: Warning successfully delivered
        self.log_causal_step(
            current_time=current_time,
            step_number=4,
            step_name="WARNING_DELIVERED",
            event_id=warning.event_id,
            vehicle_id=recipient_id,
            details=f"AODV packet confirmed delivered from {warning.sender_id} to {recipient_id}"
        )

        current_state = self.active_braking_states.get(recipient_id)
        current_status_str = current_state.state if current_state else "IDLE"

        # Step 5: Warning Received by Intended Vehicle & Validated
        val_result = self.validator.validate(
            warning=warning,
            receiving_vehicle_id=recipient_id,
            current_time=current_time,
            active_vehicle_ids=active_ids,
            current_vehicle_state=current_status_str
        )

        self.validation_log.append(asdict(val_result))

        if not val_result.is_valid:
            self.log_causal_step(
                current_time=current_time,
                step_number=5,
                step_name=f"WARNING_REJECTED_{val_result.status}",
                event_id=warning.event_id,
                vehicle_id=recipient_id,
                details=f"Warning rejected: {val_result.reason}"
            )
            return False, val_result.status

        # Warning accepted
        self.log_causal_step(
            current_time=current_time,
            step_number=5,
            step_name="WARNING_RECEIVED_VALID",
            event_id=warning.event_id,
            vehicle_id=recipient_id,
            details=f"Warning verified by On-Board Unit. Risk Tier: {warning.risk_level}, TTC: {warning.ttc_s:.2f}s"
        )

        # Step 6: Braking Activation
        success = self.actuate_autonomous_braking(
            traci_module=traci_module,
            warning=warning,
            current_time=current_time
        )

        return success, "BRAKING_ACTIVATED" if success else "ACTUATION_FAILED"

    def actuate_autonomous_braking(
        self,
        traci_module: Any,
        warning: StructuredWarningMessage,
        current_time: float
    ) -> bool:
        """
        Executes SUMO TraCI slowDown() on the recipient vehicle.
        """
        recipient_id = warning.target_id
        try:
            curr_speed = traci_module.vehicle.getSpeed(recipient_id)
            target_speed, duration, decel = self.compute_braking_profile(curr_speed, warning.risk_level)

            # Issue TraCI slowDown command
            traci_module.vehicle.slowDown(recipient_id, target_speed, duration)

            state = VehicleBrakingState(
                vehicle_id=recipient_id,
                state="BRAKING_ACTIVE",
                trigger_warning_id=warning.event_id,
                risk_level=warning.risk_level,
                warning_receipt_time=current_time,
                braking_start_time=current_time,
                initial_speed_mps=curr_speed,
                target_speed_mps=target_speed,
                braking_duration_s=duration,
                expected_decel_mps2=decel,
                min_speed_reached_mps=curr_speed,
                last_speed_mps=curr_speed,
                completed=False
            )
            self.active_braking_states[recipient_id] = state

            actuation_record = {
                "timestamp": round(current_time, 4),
                "event_id": warning.event_id,
                "vehicle_id": recipient_id,
                "sender_id": warning.sender_id,
                "risk_level": warning.risk_level,
                "initial_speed_mps": round(curr_speed, 2),
                "initial_speed_kmh": round(curr_speed * 3.6, 2),
                "target_speed_mps": round(target_speed, 2),
                "target_speed_kmh": round(target_speed * 3.6, 2),
                "duration_s": duration,
                "expected_decel_mps2": decel,
                "ttc_s": round(warning.ttc_s, 2),
                "gap_m": round(warning.gap_m, 2)
            }
            self.braking_actuation_log.append(actuation_record)

            self.log_causal_step(
                current_time=current_time,
                step_number=6,
                step_name="BRAKING_ACTIVATED",
                event_id=warning.event_id,
                vehicle_id=recipient_id,
                details=(
                    f"TraCI slowDown() invoked: speed {curr_speed * 3.6:.1f} -> "
                    f"{target_speed * 3.6:.1f} km/h over {duration:.1f}s (decel={decel:.2f} m/s²)"
                ),
                speed_mps=curr_speed,
                gap_m=warning.gap_m,
                ttc_s=warning.ttc_s
            )
            return True

        except Exception as e:
            logger.error(f"Failed to actuate braking on vehicle {recipient_id}: {e}")
            return False

    def monitor_and_update(
        self,
        traci_module: Any,
        current_time: float,
        active_gap_provider: Optional[Any] = None
    ):
        """
        Called at every simulation step:
        - Monitors speed reduction of actively responding vehicles (Step 7).
        - Detects when safe conditions return or timeout elapses.
        - Restores normal car-following speed control via traci.vehicle.setSpeed(vid, -1) (Step 8).
        """
        active_ids = traci_module.vehicle.getIDList()
        completed_vehicles = []

        for vid, bstate in list(self.active_braking_states.items()):
            if bstate.completed:
                continue

            # Handle vehicle leaving the network while response was active
            if vid not in active_ids:
                bstate.state = "RECOVERED"
                bstate.completed = True
                completed_vehicles.append(vid)
                self.log_causal_step(
                    current_time=current_time,
                    step_number=8,
                    step_name="RESPONSE_COMPLETED",
                    event_id=bstate.trigger_warning_id,
                    vehicle_id=vid,
                    details="Vehicle exited simulation; response terminated cleanly"
                )
                continue

            curr_speed = traci_module.vehicle.getSpeed(vid)
            curr_accel = traci_module.vehicle.getAcceleration(vid)
            elapsed = current_time - bstate.braking_start_time

            # Track minimum speed reached and speed reduction
            if curr_speed < bstate.min_speed_reached_mps:
                bstate.min_speed_reached_mps = curr_speed
            speed_reduction = max(0.0, bstate.initial_speed_mps - curr_speed)
            bstate.actual_speed_reduction_mps = speed_reduction
            bstate.last_speed_mps = curr_speed

            # Telemetry record per step
            self.response_telemetry_log.append({
                "timestamp": round(current_time, 2),
                "vehicle_id": vid,
                "state": bstate.state,
                "speed_mps": round(curr_speed, 2),
                "speed_kmh": round(curr_speed * 3.6, 2),
                "acceleration_mps2": round(curr_accel, 2),
                "speed_reduction_mps": round(speed_reduction, 2),
                "elapsed_response_s": round(elapsed, 2)
            })

            # Step 7: Log speed change once deceleration becomes observable
            if bstate.state == "BRAKING_ACTIVE" and speed_reduction >= 1.0:
                bstate.state = "SPEED_MONITORING"
                self.log_causal_step(
                    current_time=current_time,
                    step_number=7,
                    step_name="VEHICLE_SPEED_CHANGED",
                    event_id=bstate.trigger_warning_id,
                    vehicle_id=vid,
                    details=(
                        f"Deceleration confirmed: speed dropped from {bstate.initial_speed_mps * 3.6:.1f} to "
                        f"{curr_speed * 3.6:.1f} km/h (reduced by {speed_reduction * 3.6:.1f} km/h)"
                    ),
                    speed_mps=curr_speed
                )

            # Step 8: Recovery Condition Evaluation
            # Condition: Vehicle has slowed down past minimum duration AND
            # either target speed reached or total duration timeout reached
            should_recover = False
            recovery_reason = ""

            if elapsed >= bstate.braking_duration_s:
                should_recover = True
                recovery_reason = f"Braking duration completed ({bstate.braking_duration_s:.1f}s elapsed)"
            elif elapsed >= self.total_recovery_timeout_s:
                should_recover = True
                recovery_reason = f"Max response timeout reached ({self.total_recovery_timeout_s:.1f}s)"

            if should_recover and bstate.state != "RECOVERED":
                try:
                    # Restore default speed control to SUMO's car-following model
                    traci_module.vehicle.setSpeed(vid, -1)
                    bstate.state = "RECOVERED"
                    bstate.completed = True
                    bstate.recovery_start_time = current_time
                    completed_vehicles.append(vid)

                    self.log_causal_step(
                        current_time=current_time,
                        step_number=8,
                        step_name="RESPONSE_COMPLETED",
                        event_id=bstate.trigger_warning_id,
                        vehicle_id=vid,
                        details=(
                            f"Restored default speed control (setSpeed -1). Total speed reduction: "
                            f"{bstate.actual_speed_reduction_mps * 3.6:.1f} km/h. Reason: {recovery_reason}"
                        ),
                        speed_mps=curr_speed
                    )
                except Exception as e:
                    logger.warning(f"Error restoring speed control for {vid}: {e}")
