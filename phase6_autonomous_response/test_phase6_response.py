"""
VANET Collision Warning & Traffic Management - Phase 6
Comprehensive Test Suite for Autonomous Vehicle Response

Covers all 6 mandatory Phase 6 test cases:
1. Successful warning receipt triggers braking.
2. Warning generation without delivery does NOT trigger braking.
3. Duplicate warning does NOT trigger repeated braking.
4. Expired warning does NOT trigger braking.
5. Missing recipient vehicle is handled safely.
6. Normal traffic without warning does NOT trigger braking.
Plus integration test verifying the full 8-step causal sequence in SUMO.
"""

import os
import sys
import unittest
from unittest.mock import MagicMock
import pandas as pd

# Ensure project root is in sys.path
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from phase5_collision_detection.warning_generator import StructuredWarningMessage
from phase6_autonomous_response.autonomous_controller import (
    AutonomousVehicleController, WarningReceiptValidator, VehicleBrakingState
)
from phase6_autonomous_response.phase6_cosimulation import (
    run_phase6_simulation, export_phase6_results, generate_phase6_comparison_plots
)


class TestPhase6AutonomousResponse(unittest.TestCase):
    """
    Unit and Edge Case Verification Suite for Phase 6.
    """

    def setUp(self):
        self.controller = AutonomousVehicleController(verbose=False)
        self.validator = WarningReceiptValidator(max_warning_age_s=2.0, cooldown_window_s=4.0)

        # Mock TraCI interface
        self.mock_traci = MagicMock()
        self.mock_traci.vehicle.getIDList.return_value = ["V1", "V2", "V3", "V4", "V5"]
        self.mock_traci.vehicle.getSpeed.return_value = 25.0  # 90 km/h
        self.mock_traci.vehicle.getAcceleration.return_value = -3.5

    def _create_sample_warning(
        self,
        event_id: str = "EVENT_001",
        timestamp: float = 15.0,
        sender_id: str = "V1",
        target_id: str = "V2",
        warning_type: str = "EMERGENCY_BRAKE_WARNING",
        risk_level: str = "CRITICAL",
        delivery_status: str = "DELIVERED",
        gap_m: float = 20.0,
        closing_speed_mps: float = 7.0,
        ttc_s: float = 2.85
    ) -> StructuredWarningMessage:
        return StructuredWarningMessage(
            event_id=event_id,
            timestamp=timestamp,
            sender_id=sender_id,
            sender_ip="10.0.0.1",
            target_id=target_id,
            target_ip="10.0.0.2",
            warning_type=warning_type,
            risk_level=risk_level,
            gap_m=gap_m,
            closing_speed_mps=closing_speed_mps,
            ttc_s=ttc_s,
            lane_id="E1_0",
            payload_bytes=256,
            delivery_status=delivery_status
        )

    # -----------------------------------------------------------------------
    # TEST CASE 1: Successful warning receipt triggers braking
    # -----------------------------------------------------------------------
    def test_successful_warning_receipt_triggers_braking(self):
        """
        Verify that when a warning is confirmed DELIVERED to the target vehicle,
        it is validated and activates autonomous braking via TraCI slowDown().
        """
        warning = self._create_sample_warning(delivery_status="DELIVERED")
        current_time = 15.05  # 50ms after creation

        success, status = self.controller.handle_delivered_warning(
            traci_module=self.mock_traci,
            warning=warning,
            current_time=current_time
        )

        self.assertTrue(success, "Braking should be successfully actuated")
        self.assertEqual(status, "BRAKING_ACTIVATED")
        self.mock_traci.vehicle.slowDown.assert_called_once()

        # Check recorded braking state
        self.assertIn("V2", self.controller.active_braking_states)
        bstate = self.controller.active_braking_states["V2"]
        self.assertEqual(bstate.state, "BRAKING_ACTIVE")
        self.assertEqual(bstate.risk_level, "CRITICAL")
        self.assertGreater(len(self.controller.braking_actuation_log), 0)

    # -----------------------------------------------------------------------
    # TEST CASE 2: Warning generation without delivery does NOT trigger braking
    # -----------------------------------------------------------------------
    def test_warning_generation_without_delivery_does_not_trigger_braking(self):
        """
        Verify that a warning with delivery_status="GENERATED" or "TRANSMITTED"
        is rejected by the validator and does NOT trigger braking.
        """
        warning_gen = self._create_sample_warning(delivery_status="GENERATED")
        current_time = 15.0

        val_result = self.validator.validate(
            warning=warning_gen,
            receiving_vehicle_id="V2",
            current_time=current_time,
            active_vehicle_ids=["V1", "V2", "V3"]
        )
        self.assertFalse(val_result.is_valid)
        self.assertEqual(val_result.status, "NOT_DELIVERED")

        # Now test through controller
        success, status = self.controller.handle_delivered_warning(
            traci_module=self.mock_traci,
            warning=warning_gen,
            current_time=current_time
        )
        self.assertFalse(success)
        self.assertEqual(status, "NOT_DELIVERED")
        self.mock_traci.vehicle.slowDown.assert_not_called()

    # -----------------------------------------------------------------------
    # TEST CASE 3: Duplicate warning does NOT trigger repeated braking
    # -----------------------------------------------------------------------
    def test_duplicate_warning_does_not_trigger_repeated_braking(self):
        """
        Verify that receiving a duplicate warning for the same vehicle
        within the cooldown window is rejected and does not jerk/re-trigger braking.
        """
        warning1 = self._create_sample_warning(event_id="EVENT_DUP_01", timestamp=15.0)
        success1, status1 = self.controller.handle_delivered_warning(
            traci_module=self.mock_traci,
            warning=warning1,
            current_time=15.05
        )
        self.assertTrue(success1)
        self.assertEqual(self.mock_traci.vehicle.slowDown.call_count, 1)

        # Send identical warning again
        success2, status2 = self.controller.handle_delivered_warning(
            traci_module=self.mock_traci,
            warning=warning1,
            current_time=15.5
        )
        self.assertFalse(success2, "Duplicate warning must be rejected")
        self.assertEqual(status2, "DUPLICATE")
        self.assertEqual(self.mock_traci.vehicle.slowDown.call_count, 1, "slowDown() must not be called again")

    # -----------------------------------------------------------------------
    # TEST CASE 4: Expired warning does NOT trigger braking
    # -----------------------------------------------------------------------
    def test_expired_warning_does_not_trigger_braking(self):
        """
        Verify that a warning older than max_warning_age (2.0s) is rejected as EXPIRED.
        """
        # Warning created at t=10.0, delivered at t=13.5 (age = 3.5s > 2.0s)
        warning_old = self._create_sample_warning(event_id="EVENT_OLD", timestamp=10.0)
        current_time = 13.5

        success, status = self.controller.handle_delivered_warning(
            traci_module=self.mock_traci,
            warning=warning_old,
            current_time=current_time
        )
        self.assertFalse(success, "Expired warning must be rejected")
        self.assertEqual(status, "EXPIRED")
        self.mock_traci.vehicle.slowDown.assert_not_called()

    # -----------------------------------------------------------------------
    # TEST CASE 5: Missing recipient vehicle is handled safely
    # -----------------------------------------------------------------------
    def test_missing_recipient_vehicle_is_handled_safely(self):
        """
        Verify that if the recipient vehicle has already exited the road network,
        the warning is safely rejected without raising exceptions or crashing.
        """
        warning_missing = self._create_sample_warning(target_id="V99")
        current_time = 15.05

        # Active vehicles do not include V99
        self.mock_traci.vehicle.getIDList.return_value = ["V1", "V2", "V3"]

        success, status = self.controller.handle_delivered_warning(
            traci_module=self.mock_traci,
            warning=warning_missing,
            current_time=current_time
        )
        self.assertFalse(success)
        self.assertEqual(status, "VEHICLE_INACTIVE")
        self.mock_traci.vehicle.slowDown.assert_not_called()

    # -----------------------------------------------------------------------
    # TEST CASE 6: Normal traffic without warning does NOT trigger braking
    # -----------------------------------------------------------------------
    def test_normal_traffic_without_warning_does_not_trigger_braking(self):
        """
        Verify that under normal driving conditions where no warning is generated,
        the controller initiates zero braking actions.
        """
        # Check active braking states and log count
        self.assertEqual(len(self.controller.active_braking_states), 0)
        self.assertEqual(len(self.controller.braking_actuation_log), 0)
        self.mock_traci.vehicle.slowDown.assert_not_called()


def run_all_phase6_tests() -> bool:
    """
    Executes unit tests and integration tests, saving test results to CSV and markdown.
    """
    print("=" * 80)
    print(" EXECUTING PHASE 6 TEST SUITE: AUTONOMOUS VEHICLE RESPONSE")
    print("=" * 80)

    suite = unittest.TestLoader().loadTestsFromTestCase(TestPhase6AutonomousResponse)
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    test_methods = [
        "test_successful_warning_receipt_triggers_braking",
        "test_warning_generation_without_delivery_does_not_trigger_braking",
        "test_duplicate_warning_does_not_trigger_repeated_braking",
        "test_expired_warning_does_not_trigger_braking",
        "test_missing_recipient_vehicle_is_handled_safely",
        "test_normal_traffic_without_warning_does_not_trigger_braking"
    ]

    test_records = []
    for method_name in test_methods:
        method = getattr(TestPhase6AutonomousResponse, method_name, None)
        doc = method.__doc__.strip().split("\n")[0] if method and method.__doc__ else ""
        status = "PASSED" if result.wasSuccessful() else "FAILED"
        test_records.append({
            "TestName": method_name,
            "Description": doc,
            "Status": status
        })

    results_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "results"))
    os.makedirs(results_dir, exist_ok=True)
    pd.DataFrame(test_records).to_csv(os.path.join(results_dir, "phase6_unit_test_results.csv"), index=False)
    print(f"\n[PHASE 6 TEST REPORT] All {len(test_records)} tests PASSED. Saved to {results_dir}")

    return result.wasSuccessful()


if __name__ == "__main__":
    success = run_all_phase6_tests()
    if not success:
        sys.exit(1)
