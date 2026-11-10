#!/usr/bin/env python3
"""
sunnypilot: compare the Tesla safety decisions of two opendbc trees on the same CAN input.

The same events go into each tree's libsafety, every decision and the state after it is recorded:
steering (0x488) and DAS_control (0x2b9) TX, forwarding, controls_allowed, controls_allowed_lateral,
the MADS state and the angle / torque disengage inputs.

  # in the tree that makes the input (its DBCs and packer), once
  python -m opendbc.safety.tests.tesla_equivalence gen events.json
  # in each tree
  PYTHONPATH=<tree> python -m opendbc.safety.tests.tesla_equivalence run events.json out.json
  # anywhere
  python -m opendbc.safety.tests.tesla_equivalence compare base.json new.json
"""
import json
import random
import sys

TESLA = 10  # CarParams.SafetyModel.tesla
SP_VEHICLE_BUS, SP_MADS_3_FINGER = 1, 2
LONGITUDINAL, LEGACY_DAS_STEERING, HW4_GEN2 = 1, 2, 4

# safety param, safety param SP, MADS (enable, disengage lateral on brake, pause lateral on brake)
CONFIGS = [(p, sp, mads)
           for p in (0, LONGITUDINAL, LEGACY_DAS_STEERING, HW4_GEN2)
           for sp in (0, SP_VEHICLE_BUS | SP_MADS_3_FINGER)
           for mads in ((False, False, False), (True, False, False), (True, True, False), (True, False, True))]


def gen(path, steps=2500, seed=0):
  from opendbc.can import CANPacker
  # the packer keeps one counter per address, openpilot's 0x488 / 0x2b9 TX gets its own
  party, party_tx, vehicle = CANPacker("tesla_model3_party"), CANPacker("tesla_model3_party"), CANPacker("tesla_model3_vehicle")
  rnd = random.Random(seed)
  events = []

  def can(kind, packer, name, bus, values, corrupt=False):
    addr, dat, bus = packer.make_can_msg(name, bus, values)
    assert addr != 0, name
    dat = bytearray(dat)
    if corrupt:
      dat[rnd.randrange(len(dat))] ^= 1 << rnd.randrange(8)
    events.append([kind, bus, addr, dat.hex()])

  for param, param_sp, mads in CONFIGS:
    events.append(["setup", param, param_sp, list(mads)])
    t = 0
    speed, angle, cruise, gas, brake, override, summon, stock_steer, aeb, touch, hb_mads = 10., 0., False, False, False, 0, 0, 0, False, 0, True
    for step in range(steps):
      t += 10000
      events.append(["timer", t])

      # driver and car, slow random walk with bursts
      speed = min(max(speed + rnd.gauss(0, 0.3), 0.), 40.)
      angle = min(max(angle + rnd.gauss(0, 1.5), -90.), 90.)
      if rnd.random() < 0.01:
        cruise = not cruise
      gas = rnd.random() < 0.05 if rnd.random() < 0.1 else gas
      brake = rnd.random() < 0.08 if rnd.random() < 0.1 else brake
      override = rnd.choice((0, 0, 0, 1, 2)) if rnd.random() < 0.05 else override  # 1: hands on level, 2: torsion torque
      summon = rnd.choice((0, 0, 0, 0, 3, 5, 9)) if rnd.random() < 0.003 else summon
      stock_steer = rnd.choice((0, 0, 0, 0, 2, 4)) if rnd.random() < 0.01 else stock_steer
      aeb = (rnd.random() < 0.2) if rnd.random() < 0.005 else aeb
      touch = rnd.choice((0, 0, 3, 4)) if rnd.random() < 0.02 else touch
      drop = rnd.random() < 0.002  # a silent stretch makes the RX checks lag
      bad = rnd.random() < 0.003   # one corrupted frame

      if not drop:
        # TESLA_STEERING_DISENGAGE_TORQUE is 5 Nm, torque near it too
        if override == 2:
          torque = rnd.choice((rnd.uniform(-4.9, 4.9), rnd.uniform(4.95, 5.05), rnd.uniform(5.1, 8.))) * rnd.choice((-1, 1))
        else:
          torque = rnd.uniform(-1., 1.)
        can("rx", party, "EPAS3S_sysStatus", 0, {"EPAS3S_internalSAS": angle, "EPAS3S_handsOnLevel": 3 if override == 1 else rnd.choice((0, 1)),
                                               "EPAS3S_torsionBarTorque": torque, "EPAS3S_eacStatus": 1, "EPAS3S_eacErrorCode": 0}, bad)
        can("rx", party, "DI_speed", 0, {"DI_vehicleSpeed": speed * 3.6, "DI_accelPedalPressed": gas})
        can("rx", party, "ESP_B", 0, {"ESP_vehicleSpeed": (speed + rnd.uniform(-0.5, 0.5) + (3. if rnd.random() < 0.002 else 0.)) * 3.6,
                                      "ESP_wheelSpeedsQF": 1, "ESP_vehicleStandstillSts": 1 if speed < 0.1 else 0})
        can("rx", party, "DI_systemStatus", 0, {"DI_gear": 4})
        can("rx", party, "ESP_status", 0, {"ESP_driverBrakeApply": 2 if brake else 1})
        if step % 10 == 0:
          can("rx", party, "DI_state", 0, {"DI_cruiseState": 2 if cruise else 0, "DI_autoparkState": summon})
          can("rx", party, "UI_warning", 0, {"buckleStatus": 1})
        if step % 2 == 0:
          can("rx", party, "DAS_control", 2, {"DAS_aebEvent": 1 if aeb else 0, "DAS_accState": 4})
        can("rx", party, "DAS_steeringControl", 2, {"DAS_steeringControlType": stock_steer, "DAS_steeringAngleRequest": angle})
        if step % 50 == 0:
          can("rx", vehicle, "UI_status2", 1, {"UI_activeTouchPoints": touch})

      # openpilot: angle commands near and past the limits, any control type
      cmd = angle + rnd.choice((0., rnd.uniform(-1., 1.), rnd.uniform(-20., 20.)))
      # the control type is a 3-bit field, the legacy 2-bit one is its top bits
      can("tx", party_tx, "DAS_steeringControl", 0, {"DAS_steeringAngleRequest": cmd, "DAS_steeringControlType": rnd.choice((0, 1, 1, 1, 2, 2, 3, 4, 5))})
      if step % 4 == 0:
        accel = [rnd.choice((0., rnd.uniform(-3.48, 2.), rnd.uniform(-6., 4.))) for _ in range(2)]
        can("tx", party_tx, "DAS_control", 0, {"DAS_setSpeed": 40, "DAS_accState": rnd.choice((4, 4, 13)), "DAS_aebEvent": 1 if rnd.random() < 0.01 else 0,
                                            "DAS_accelMin": min(accel), "DAS_accelMax": max(accel)})
      if step % 25 == 0:
        events.append(["fwd", 2, rnd.choice((0x488, 0x27d, 0x2b9, 0x123))])
        events.append(["fwd", 0, rnd.choice((0x488, 0x27d, 0x2b9))])
      if step % 100 == 0:
        # openpilot's MADS heartbeat, false stretches of a few seconds
        if rnd.random() < 0.15:
          hb_mads = not hb_mads
        events.append(["hb_mads", hb_mads])
        events.append(["tick"])

  with open(path, "w") as f:
    json.dump(events, f)
  print(f"{len(events)} events, {len(CONFIGS)} configs")


def run(path, out_path):
  from opendbc.safety.tests.libsafety import libsafety_py
  s = libsafety_py.libsafety
  dual = hasattr(s, "safety_heartbeat_check")  # merged tree: two copies
  with open(path) as f:
    events = json.load(f)

  def state():
    return [bool(s.get_controls_allowed()), bool(s.get_controls_allowed_lateral()), bool(s.get_controls_requested_lateral()),
            int(s.mads_get_current_disengage_reason()), int(s.get_mads_button_press()), bool(s.get_vehicle_moving()),
            bool(s.get_gas_pressed_prev()), bool(s.get_brake_pressed_prev()), bool(s.get_steering_disengage_prev()),
            bool(s.get_cruise_engaged_prev()), int(s.get_angle_meas_min()), int(s.get_angle_meas_max()), bool(s.get_relay_malfunction())]

  out = []
  for e in events:
    kind = e[0]
    if kind == "setup":
      param, param_sp, mads = e[1], e[2], e[3]
      s.set_current_safety_param_sp(param_sp)
      s.set_safety_hooks(TESLA, param)
      s.init_tests()  # clears the param SP (the config is built) and the startup summon assumption, like the safety tests
      s.set_mads_params(*mads)
      if dual:
        s.safety_heartbeat(True)  # openpilot heartbeat, the old libsafety has no controls_allowed heartbeat check
      s.set_heartbeat_engaged_mads(True)
      r = None
    elif kind == "timer":
      s.set_timer(e[1])
      continue
    elif kind == "rx":
      r = bool(s.safety_rx_hook(libsafety_py.make_CANPacket(e[2], e[1], bytes.fromhex(e[3]))))
    elif kind == "tx":
      r = bool(s.safety_tx_hook(libsafety_py.make_CANPacket(e[2], e[1], bytes.fromhex(e[3]))))
    elif kind == "fwd":
      r = int(s.safety_fwd_hook(e[1], e[2]))
    elif kind == "hb_mads":
      s.set_heartbeat_engaged_mads(e[1])
      r = None
    elif kind == "tick":
      if dual:
        s.safety_heartbeat_check()
      else:
        s.mads_heartbeat_engaged_check()
      s.safety_tick()
      r = None
    else:
      raise ValueError(kind)
    out.append([r, *state()])
    if dual:
      assert not s.safety_redundancy_fault, f"redundancy fault at {e}"
  with open(out_path, "w") as f:
    json.dump(out, f)
  print(f"{len(out)} records")


FIELDS = ["decision", "controls_allowed", "controls_allowed_lateral", "controls_requested_lateral", "disengage_reason", "mads_button",
          "vehicle_moving", "gas_pressed_prev", "brake_pressed_prev", "steering_disengage_prev", "cruise_engaged_prev",
          "angle_meas_min", "angle_meas_max", "relay_malfunction"]


def compare(a_path, b_path, events_path=None):
  with open(a_path) as f:
    a = json.load(f)
  with open(b_path) as f:
    b = json.load(f)
  assert len(a) == len(b), (len(a), len(b))
  diffs = [i for i in range(len(a)) if a[i] != b[i]]
  # how much of each decision the input covers
  counts = {}
  if events_path:
    with open(events_path) as f:
      events = [e for e in json.load(f) if e[0] != "timer"]
    for e, r in zip(events, a, strict=True):
      key = f"{e[0]} {hex(e[2]) if e[0] in ('rx', 'tx', 'fwd') else ''}".strip()
      c = counts.setdefault(key, {})
      c[str(r[0])] = c.get(str(r[0]), 0) + 1
    print("decisions:", json.dumps(counts))
    print("controls_allowed true:", sum(r[1] for r in a), "controls_allowed_lateral true:", sum(r[2] for r in a), "of", len(a))
  print(f"{len(a)} records, {len(diffs)} differ")
  for i in diffs[:10]:
    print(i, {FIELDS[k]: (a[i][k], b[i][k]) for k in range(len(FIELDS)) if a[i][k] != b[i][k]})
  return not diffs


if __name__ == "__main__":
  cmd = sys.argv[1]
  if cmd == "gen":
    gen(sys.argv[2])
  elif cmd == "run":
    run(sys.argv[2], sys.argv[3])
  elif cmd == "compare":
    sys.exit(0 if compare(*sys.argv[2:]) else 1)
