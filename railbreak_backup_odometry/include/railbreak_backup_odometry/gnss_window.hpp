#pragma once

namespace railbreak {

// When the start window closes. Stamps only: no wall clock.
//
// A fix past the window closes immediately, and only from that fix. The first
// wheel or command stamp past the window must not: at --rate 10 it is delivered
// before GNSS still sitting in the queue, and unsubscribing there drops the window.
//
// If the antennas stop inside the window (the case allows GNSS for only the
// first seconds), close after kQuietInputs wheel/command callbacks with no GNSS
// callback between them. Those callbacks are the drain. An antenna that never
// sent a fix is not waited on. An antenna that already delivered a stamp past
// the window does not keep the other one open.
struct GnssWindow {
  static constexpr int kQuietInputs = 8;

  double window_s = 3.0;
  double wait_s = 10.0;
  double t_first_fix = -1.0;
  double t_first_rover = -1.0;
  double t_first_input = -1.0;
  bool master_past = false;
  bool rover_past = false;
  bool closed = false;

  enum class Action { kWait, kFinish, kRelative };

  Action on_fix(bool master, double t, bool valid) {
    if (closed) return Action::kWait;
    if (master) ++n_master_cb_;
    else ++n_rover_cb_;
    if (!valid) return Action::kWait;
    if (master) {
      if (t_first_fix < 0.0) t_first_fix = t;
    } else if (t_first_rover < 0.0) {
      t_first_rover = t;
    }
    const double t_open = t_first_fix >= 0.0 ? t_first_fix : t_first_rover;
    if (t_open >= 0.0 && t > t_open + window_s) {
      if (master) master_past = true;
      else rover_past = true;
      const bool master_done = t_first_fix < 0.0 || master_past;
      const bool rover_done = t_first_rover < 0.0 || rover_past;
      if (master_done && rover_done) return Action::kFinish;
    }
    return Action::kWait;
  }

  Action on_input(double t) {
    if (closed) return Action::kWait;
    if (t_first_input < 0.0) t_first_input = t;
    if (t_first_fix < 0.0 && t_first_rover < 0.0 && t > t_first_input + wait_s)
      return Action::kRelative;
    const double t_open = t_first_fix >= 0.0 ? t_first_fix : t_first_rover;
    if (t_open < 0.0 || t <= t_open + window_s) return Action::kWait;

    const bool pending_master = t_first_fix >= 0.0 && !master_past;
    const bool pending_rover = t_first_rover >= 0.0 && !rover_past;
    const bool draining = (pending_master && n_master_cb_ != master_mark_) ||
                          (pending_rover && n_rover_cb_ != rover_mark_);
    if (draining) {
      master_mark_ = n_master_cb_;
      rover_mark_ = n_rover_cb_;
      quiet_ = 0;
      return Action::kWait;
    }
    if (++quiet_ >= kQuietInputs) return Action::kFinish;
    return Action::kWait;
  }

 private:
  int n_master_cb_ = 0;
  int n_rover_cb_ = 0;
  int master_mark_ = -1;
  int rover_mark_ = -1;
  int quiet_ = 0;
};

}  // namespace railbreak
