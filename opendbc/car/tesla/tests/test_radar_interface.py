import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint
from opendbc.car.tesla.interface import CarInterface
from opendbc.car.tesla.radar_interface import DAS_POINT_TIMEOUT
from opendbc.car.tesla.values import CANBUS, CAR, DAS_CUTIN_TRACK_ID_BASE, DBC

PACKER = CANPacker(DBC[CAR.TESLA_MODEL_Y][Bus.adas])


def radar_interface(hw4_gen2=True, vehicle_bus=True):
  fingerprint = gen_empty_fingerprint()
  if hw4_gen2:
    fingerprint[CANBUS.autopilot_party][0x399] = 8
  if vehicle_bus:
    fingerprint[CANBUS.vehicle][0x3F5] = 8
  CP = CarInterface.get_params(CAR.TESLA_MODEL_Y, fingerprint, [], False, False, False)
  CP_SP = CarInterface.get_params_sp(CP, CAR.TESLA_MODEL_Y, fingerprint, [], False, False, False)
  return CarInterface.RadarInterface(CP, CP_SP)


def das_object(object_id=0, prefix="DAS_leadVeh", dx=30.0, dy=1.4, vx_rel=-2.0, veh_id=5, relevant=1):
  return PACKER.make_can_msg("DAS_object", CANBUS.vehicle, {
    "DAS_objectId": object_id,
    f"{prefix}Type": 2,
    f"{prefix}RelevantForControl": relevant,
    f"{prefix}Dx": dx,
    f"{prefix}VxRel": vx_rel,
    f"{prefix}Dy": dy,
    f"{prefix}Id": veh_id,
  })


def run(ri, frames_by_call, calls):
  """Calls update() `calls` times with frames_by_call.get(i, []) on call i. Returns what it published."""
  published = []
  for i in range(calls):
    rr = ri.update([(i * 10_000_000, frames_by_call.get(i, []))])
    if rr is not None:
      published.append(rr)
  return published


class TestDasObjectPoints(unittest.TestCase):
  def test_without_das_object_empty_points_go_out_at_20hz(self):
    published = run(radar_interface(), {}, 100)
    assert len(published) == 20
    for rr in published:
      assert len(rr.points) == 0
      assert not any(rr.errors.to_dict().values())

  def test_lead_frame_becomes_a_point(self):
    rr, = run(radar_interface(), {0: [das_object(dx=30.0, dy=1.4, vx_rel=-2.0, veh_id=5)]}, 5)
    pt, = rr.points
    assert pt.trackId == 5
    self.assertAlmostEqual(pt.dRel, 30.0, places=4)
    self.assertAlmostEqual(pt.yRel, -1.4, places=4)  # DAS Dy is positive to the left, yRel to the right
    self.assertAlmostEqual(pt.vRel, -2.0, places=4)

  def test_cutin_frame_gets_its_own_track_id_range(self):
    rr, = run(radar_interface(), {0: [das_object(object_id=3, prefix="DAS_cutinVeh", veh_id=7)]}, 5)
    pt, = rr.points
    assert pt.trackId == DAS_CUTIN_TRACK_ID_BASE + 7

  def test_frames_in_one_call_are_demultiplexed(self):
    frames = [das_object(dx=40.0), das_object(object_id=3, prefix="DAS_cutinVeh", dx=15.0), das_object(object_id=1, dx=60.0)]
    rr, = run(radar_interface(), {0: frames}, 5)
    assert sorted(round(pt.dRel) for pt in rr.points) == [15, 40]

  def test_invalid_frames_are_not_points(self):
    for bad in ({"dx": 127.5}, {"vx_rel": 30.0}, {"veh_id": 127}, {"relevant": 0}):
      with self.subTest(**bad):
        rr, = run(radar_interface(), {0: [das_object(**bad)]}, 5)
        assert len(rr.points) == 0

  def test_invalid_frame_clears_the_slot(self):
    rr, = run(radar_interface(), {0: [das_object()], 1: [das_object(dx=127.5)]}, 5)
    assert len(rr.points) == 0

  def test_point_expires_half_a_second_after_its_last_frame(self):
    published = run(radar_interface(), {0: [das_object()]}, DAS_POINT_TIMEOUT + 5)
    assert [len(rr.points) for rr in published] == [1] * (DAS_POINT_TIMEOUT // 5) + [0]

  def test_other_teslas_keep_the_old_path(self):
    for hw4_gen2, vehicle_bus in ((True, False), (False, True), (False, False)):
      with self.subTest(hw4_gen2=hw4_gen2, vehicle_bus=vehicle_bus):
        ri = radar_interface(hw4_gen2, vehicle_bus)
        assert ri.das_cp is None
        published = run(ri, {i: [das_object()] for i in range(20)}, 20)
        assert len(published) == 4
        assert all(len(rr.points) == 0 for rr in published)


if __name__ == "__main__":
  unittest.main()
