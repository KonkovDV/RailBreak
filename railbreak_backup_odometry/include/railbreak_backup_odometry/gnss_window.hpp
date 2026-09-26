#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace railbreak {

// A fix that may open the window or enter the start sample. Status is compared
// with the caller's STATUS_FIX. A NaN stamp must not become t_open. A NaN
// altitude, or a latitude/longitude outside the closed geographic bounds, must
// not enter the median.
inline bool gnss_fix_ok(double t, double lat, double lon, double alt, int status, int status_fix) {
  return status >= status_fix && std::isfinite(t) && std::isfinite(alt) && lat >= -90.0 &&
         lat <= 90.0 && lon >= -180.0 && lon <= 180.0;
}

// One fix taken from a subscription, before the two queues are merged.
// index is the position in the caller's holding array.
struct QueuedStamp {
  bool master = false;
  double t = 0.0;
  std::size_t index = 0;
};

// Finite stamps first, ascending. Equal stamps keep the order they were taken
// (masters, then rovers, each queue in take order). A non-finite stamp sorts
// after every finite one and does not disturb that order.
inline std::vector<QueuedStamp> order_queued_stamps(std::vector<QueuedStamp> q) {
  std::stable_sort(q.begin(), q.end(), [](const QueuedStamp& a, const QueuedStamp& b) {
    const bool fa = std::isfinite(a.t);
    const bool fb = std::isfinite(b.t);
    if (fa != fb) return fa;
    if (!fa) return false;
    return a.t < b.t;
  });
  return q;
}

// When the start window closes. Stamps only: no wall clock.
//
// A fix past the window closes immediately, and only from that fix. The first
// wheel or command stamp past the window must not: at --rate 10 it is delivered
// before GNSS still sitting in the queue, and unsubscribing there drops the window.
//
// If the antennas stop inside the window (the case allows GNSS for only the
// first seconds), the quiet count asks the node to close. Eight wheel/command
// callbacks are not a proof that a GNSS message is absent from the middleware
// queue: the node must take() those subscriptions before unsubscribing.
// A STATUS_NO_FIX stream is not a fix and must not hold the window open.
// Valid fixes still in the queue do reset the drain. An antenna that never
// sent a valid fix is not waited on. An antenna that already delivered a stamp
// past the window does not keep the other one open.
struct GnssWindow {
  static constexpr int kQuietInputs = 8;

  double window_s = 3.0;
  double wait_s = 10.0;
  // t_open is the window origin: the first valid fix of either antenna.
  // It does not move when the other antenna arrives later.
  // t_first_fix is only the first valid master, t_first_rover only the first
  // valid rover. They say which antenna is still waited on. Neither is the start.
  double t_first_fix = -1.0;
  double t_first_rover = -1.0;
  double t_open = -1.0;
  double t_first_input = -1.0;
  bool master_past = false;
  bool rover_past = false;
  bool closed = false;

  enum class Action { kWait, kFinish, kRelative };

  Action on_fix(bool master, double t, bool valid) {
    if (closed) return Action::kWait;
    if (!valid) return Action::kWait;
    // The sample set starts at the first valid fix of either antenna and does not move.
    if (t_open < 0.0) t_open = t;
    if (master) ++n_master_cb_;
    else ++n_rover_cb_;
    if (master) {
      if (t_first_fix < 0.0) t_first_fix = t;
    } else if (t_first_rover < 0.0) {
      t_first_rover = t;
    }
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

  int master_fixes() const { return n_master_cb_; }
  int rover_fixes() const { return n_rover_cb_; }

 private:
  int n_master_cb_ = 0;
  int n_rover_cb_ = 0;
  int master_mark_ = -1;
  int rover_mark_ = -1;
  int quiet_ = 0;
};

}  // namespace railbreak
