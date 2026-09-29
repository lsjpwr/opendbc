import unittest

from opendbc.car.tesla.carcontroller import get_safety_CP
from opendbc.car.vehicle_model import VehicleModel
from opendbc.sunnypilot.car.tesla.coop_steering import DT_LAT_CTRL, CoopSteeringCarController

VM = VehicleModel(get_safety_CP())


class TestCoopSteering(unittest.TestCase):
  def test_resume_starts_where_the_wheel_is_heading(self):
    # the wheel turns at 50 deg/s while lateral is off, so the first active frame continues from where it will be
    coop = CoopSteeringCarController()
    coop.resume_steer_desired_rate_limit(False, 10.0, 50.0)
    heading = 10.0 + 50.0 * DT_LAT_CTRL
    self.assertAlmostEqual(coop.resume_steer_desired_rate_limit(True, heading, 50.0), heading)

  def test_same_direction_model_step_is_not_counted_twice(self):
    base, same = CoopSteeringCarController(), CoopSteeringCarController()
    expected = base.update_override_angle(0.0, 1.5, 10.0, VM) - 0.2
    self.assertAlmostEqual(same.update_override_angle(0.2, 1.5, 10.0, VM), expected)

  def test_opposing_model_step_grows_the_override(self):
    # 1.5 Nm is 1 Nm past the deadzone: half of STEER_OVERRIDE_TORQUE_RANGE
    base, opposing = CoopSteeringCarController(), CoopSteeringCarController()
    expected = base.update_override_angle(0.0, 1.5, 10.0, VM) + 0.5 * 0.2
    self.assertAlmostEqual(opposing.update_override_angle(-0.2, 1.5, 10.0, VM), expected)


if __name__ == "__main__":
  unittest.main()
