/**
 * Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.
 *
 * This file is part of sunnypilot and is licensed under the MIT License.
 * See the LICENSE.md file in the root directory for more details.
 */

#pragma once

#include "opendbc/safety/declarations.h"

// MADS state is safety.mads and the other MADS fields of SafetyState, the settings are safety_setup.mads

// ===============================
// State Update Helpers
// ===============================

inline EdgeTransition m_get_edge_transition(const bool current, const bool last) {
  EdgeTransition state;

  if (current && !last) {
    state = MADS_EDGE_RISING;
  } else if (!current && last) {
    state = MADS_EDGE_FALLING;
  } else {
    state = MADS_EDGE_NO_CHANGE;
  }

  return state;
}

inline void m_mads_state_init(void) {
  safety.mads.is_vehicle_moving = NULL;
  safety.mads.acc_main.current = NULL;
  safety.mads.mads_button.current = MADS_BUTTON_UNAVAILABLE;

  safety.mads.acc_main.previous = false;
  safety.mads.acc_main.transition = MADS_EDGE_NO_CHANGE;

  safety.mads.mads_button.last = MADS_BUTTON_UNAVAILABLE;
  safety.mads.mads_button.transition = MADS_EDGE_NO_CHANGE;


  safety.mads.current_disengage.active_reason = MADS_DISENGAGE_REASON_NONE;
  safety.mads.current_disengage.pending_reasons = MADS_DISENGAGE_REASON_NONE;

  safety.mads.controls_requested_lateral = false;
  safety.controls_allowed_lateral = false;
}

inline void m_update_button_state(ButtonStateTracking *button_state) {
  if (button_state->current != MADS_BUTTON_UNAVAILABLE) {
    button_state->transition = m_get_edge_transition(button_state->current == MADS_BUTTON_PRESSED, button_state->last == MADS_BUTTON_PRESSED);
    button_state->last = button_state->current;
  }
}

inline void m_update_binary_state(BinaryStateTracking *state) {
  state->transition = m_get_edge_transition(state->current, state->previous);
  state->previous = state->current;
}

/**
 * @brief Updates the MADS control state based on current system conditions
 *
 * @return void
 */
inline void m_update_control_state(void) {
  bool allowed = true;

  // Initial control requests from button or ACC transitions
  if ((safety.mads.acc_main.transition == MADS_EDGE_RISING) ||
      (safety.mads.mads_button.transition == MADS_EDGE_RISING) ||
      (safety.mads.op_controls_allowed.transition == MADS_EDGE_RISING)) {
    safety.mads.controls_requested_lateral = true;
  }

  // Primary control blockers - these prevent any further control processing
  if (safety.mads.acc_main.transition == MADS_EDGE_FALLING) {
    mads_exit_controls(MADS_DISENGAGE_REASON_ACC_MAIN_OFF);
    allowed = false;  // No matter what, no further control processing on this cycle
  }

  if (safety.mads.mads_steering_disengage.transition == MADS_EDGE_RISING) {
    mads_exit_controls(MADS_DISENGAGE_REASON_STEERING_DISENGAGE);
    allowed = false;  // No matter what, no further control processing on this cycle
  }

  if (safety_setup.mads.disengage_lateral_on_brake && (safety.mads.braking.transition == MADS_EDGE_RISING)) {
    mads_exit_controls(MADS_DISENGAGE_REASON_BRAKE);
    allowed = false;
  }

  // Secondary control conditions - only checked if primary conditions don't block further control processing
  if (allowed && safety_setup.mads.pause_lateral_on_brake) {
    // Brake rising edge immediately blocks controls
    // Brake release might request controls if brake was the ONLY reason for disengagement
    if (safety.mads.braking.transition == MADS_EDGE_RISING) {
      mads_exit_controls(MADS_DISENGAGE_REASON_BRAKE);
      allowed = false;
    } else if ((safety.mads.braking.transition == MADS_EDGE_FALLING) &&
               (safety.mads.current_disengage.active_reason == MADS_DISENGAGE_REASON_BRAKE) &&
               (safety.mads.current_disengage.pending_reasons == MADS_DISENGAGE_REASON_BRAKE)) {
      safety.mads.controls_requested_lateral = true;
    } else if (safety.mads.braking.current) {
      allowed = false;
    } else {
    }
  }

  // Process control request if conditions allow. Gate the write on system_enabled so that
  // stock safety builds (MADS disabled) never flip the global on.
  if (allowed && safety_setup.mads.system_enabled && safety.mads.controls_requested_lateral && !safety.controls_allowed_lateral) {
    safety.mads.controls_requested_lateral = false;
    safety.controls_allowed_lateral = true;
    safety.mads.current_disengage.active_reason = MADS_DISENGAGE_REASON_NONE;
    safety.mads.current_disengage.pending_reasons = MADS_DISENGAGE_REASON_NONE;
  }
}

inline void mads_heartbeat_engaged_check(void) {
  if (safety.controls_allowed_lateral && !safety.heartbeat_engaged_mads) {
    safety.heartbeat_engaged_mads_mismatches += 1U;
    if (safety.heartbeat_engaged_mads_mismatches >= 3U) {
      mads_exit_controls(MADS_DISENGAGE_REASON_HEARTBEAT_ENGAGED_MISMATCH);
    }
  } else {
    safety.heartbeat_engaged_mads_mismatches = 0U;
  }
}

// ===============================
// Function Implementations
// ===============================

// cppcheck-suppress misra-c2012-8.7; called from libsafety test harness
inline void mads_set_alternative_experience(const int *mode) {
  const bool mads_enabled = (*mode & ALT_EXP_ENABLE_MADS) != 0;
  const bool disengage_lateral_on_brake = (*mode & ALT_EXP_MADS_DISENGAGE_LATERAL_ON_BRAKE) != 0;
  const bool pause_lateral_on_brake = (*mode & ALT_EXP_MADS_PAUSE_LATERAL_ON_BRAKE) != 0;

  mads_set_system_state(mads_enabled, disengage_lateral_on_brake, pause_lateral_on_brake);
}

extern inline void mads_set_system_state(const bool enabled, const bool disengage_lateral_on_brake, const bool pause_lateral_on_brake) {
  m_mads_state_init();
  safety_setup.mads.system_enabled = enabled;
  safety_setup.mads.disengage_lateral_on_brake = disengage_lateral_on_brake;
  safety_setup.mads.pause_lateral_on_brake = pause_lateral_on_brake;
}

inline void mads_exit_controls(const DisengageReason reason) {
  // Always track this as a pending reason
  safety.mads.current_disengage.pending_reasons |= reason;

  if (safety.controls_allowed_lateral) {
    safety.mads.current_disengage.active_reason = reason;
    safety.mads.controls_requested_lateral = false;
    safety.controls_allowed_lateral = false;
  }
}

inline void mads_state_update(const bool op_vehicle_moving, const bool op_acc_main, const bool op_allowed, const bool is_braking, const bool _steering_disengage) {
  safety.mads.is_vehicle_moving = op_vehicle_moving;
  safety.mads.acc_main.current = op_acc_main;
  safety.mads.op_controls_allowed.current = op_allowed;
  safety.mads.mads_button.current = safety.mads_button_press;
  safety.mads.braking.current = is_braking;
  safety.mads.mads_steering_disengage.current = _steering_disengage;

  m_update_binary_state(&safety.mads.acc_main);
  m_update_binary_state(&safety.mads.op_controls_allowed);
  m_update_binary_state(&safety.mads.braking);
  m_update_binary_state(&safety.mads.mads_steering_disengage);
  m_update_button_state(&safety.mads.mads_button);

  m_update_control_state();
}
