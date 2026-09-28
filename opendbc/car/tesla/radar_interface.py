from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.tesla.values import CANBUS, DAS_CUTIN_TRACK_ID_BASE, DBC, TeslaFlags

RADAR_START_ADDR = 0x410
RADAR_MSG_COUNT = 80  # 40 points * 2 messages each

# DAS_object (0x309) is Tesla's own vision object list on the VEHICLE bus, multiplexed by DAS_objectId.
# Its lead and cut-in slots go out as points that radard only uses to confirm the model's leads.
# The layout is from the community Model 3 DBC and unverified on 2026+ Model Y.
DAS_OBJECT_SIGNAL_PREFIX = {0: "DAS_leadVeh", 3: "DAS_cutinVeh"}
DAS_TRACK_ID_BASE = {0: 0, 3: DAS_CUTIN_TRACK_ID_BASE}
DAS_DX_SNA = 127.5      # m, raw 255
DAS_VX_REL_SNA = 30.0   # m/s, raw 15
DAS_ID_SNA = 127
DAS_POINT_TIMEOUT = 50  # update() calls, 0.5 s at card's 100 Hz


def get_radar_can_parser(CP):
  if Bus.radar not in DBC[CP.carFingerprint]:
    return None

  messages = [('RadarStatus', 16)]
  for i in range(RADAR_MSG_COUNT // 2):
    messages.extend([
      (f'RadarPoint{i}_A', 16),
      (f'RadarPoint{i}_B', 16),
    ])

  return CANParser(DBC[CP.carFingerprint][Bus.radar], messages, 1)


def get_das_can_parser(CP):
  if not CP.flags & TeslaFlags.HW4_GEN2_VEHICLE_BUS:
    return None

  # nan frequency turns the alive check off: a firmware without DAS_object must not raise CAN errors
  return CANParser(DBC[CP.carFingerprint][Bus.adas], [("DAS_object", float("nan"))], CANBUS.vehicle)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.updated_messages = set()
    self.trigger_msg = RADAR_START_ADDR + RADAR_MSG_COUNT - 1

    self.radar_off_can = CP.radarUnavailable
    self.rcp = get_radar_can_parser(CP)

    self.das_cp = get_das_can_parser(CP)
    self.das_objects: dict[int, tuple[int, int, float, float, float]] = {}  # objectId: (frame, trackId, dRel, yRel, vRel)

  def update(self, can_strings):
    if self.das_cp is not None:
      return self._update_das(self.das_cp, can_strings)

    if self.radar_off_can or self.rcp is None:
      return super().update(None)

    vls = self.rcp.update(can_strings)
    self.updated_messages.update(vls)

    if self.trigger_msg not in self.updated_messages:
      return None

    rr = self._update(self.updated_messages)
    self.updated_messages.clear()

    return rr

  def _update_das(self, das_cp, can_strings):
    self.frame += 1
    das_cp.update(can_strings)

    frames = das_cp.vl_all["DAS_object"]
    for i, object_id in enumerate(frames["DAS_objectId"]):
      object_id = int(object_id)
      prefix = DAS_OBJECT_SIGNAL_PREFIX.get(object_id)
      if prefix is None:
        continue

      d_rel = frames[f"{prefix}Dx"][i]
      v_rel = frames[f"{prefix}VxRel"][i]
      veh_id = int(frames[f"{prefix}Id"][i])
      if d_rel < DAS_DX_SNA and v_rel < DAS_VX_REL_SNA and veh_id != DAS_ID_SNA and frames[f"{prefix}RelevantForControl"][i] == 1:
        # DAS Dy is taken as positive to the left (ISO 8855); yRel is positive to the right
        self.das_objects[object_id] = (self.frame, DAS_TRACK_ID_BASE[object_id] + veh_id, d_rel, -frames[f"{prefix}Dy"][i], v_rel)
      else:
        self.das_objects.pop(object_id, None)

    if self.frame % 5 != 0:  # 20 Hz, like RadarInterfaceBase
      return None

    ret = structs.RadarData()
    points = []
    for frame, track_id, d_rel, y_rel, v_rel in self.das_objects.values():
      if self.frame - frame < DAS_POINT_TIMEOUT:
        pt = structs.RadarData.RadarPoint()
        pt.trackId = track_id
        pt.dRel = d_rel
        pt.yRel = y_rel
        pt.vRel = v_rel
        points.append(pt)
    ret.points = points
    return ret

  def _update(self, updated_messages):
    ret = structs.RadarData()
    if self.rcp is None:
      return ret

    if not self.rcp.can_valid:
      ret.errors.canError = True

    radar_status = self.rcp.vl['RadarStatus']
    if radar_status['shortTermUnavailable']:
      ret.errors.radarUnavailableTemporary = True
    if radar_status['sensorBlocked'] or radar_status['vehDynamicsError']:
      ret.errors.radarFault = True

    for i in range(RADAR_MSG_COUNT // 2):
      msg_a = self.rcp.vl[f'RadarPoint{i}_A']
      msg_b = self.rcp.vl[f'RadarPoint{i}_B']

      # Make sure msg A and B are together
      if msg_a['Index'] != msg_b['Index2']:
        continue

      if not msg_a['Tracked']:
        if i in self.pts:
          del self.pts[i]
        continue

      if i not in self.pts:
        self.pts[i] = structs.RadarData.RadarPoint()
        self.pts[i].trackId = self.track_id
        self.track_id += 1

      self.pts[i].dRel = msg_a['LongDist']
      self.pts[i].yRel = msg_a['LatDist']
      self.pts[i].vRel = msg_a['LongSpeed']

    ret.points = list(self.pts.values())
    return ret
