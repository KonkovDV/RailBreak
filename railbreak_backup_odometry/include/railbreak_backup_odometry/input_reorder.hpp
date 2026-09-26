#pragma once

// Hold vehicle samples until a newer header is `hold_s` ahead, then release
// them in header-stamp order. rosbag play delivers the two bogies and the
// notch in receive order; those headers step backwards by tens of milliseconds.
// A stamp that is still behind the filter after this wait is not released here.
// The node counts it and does not publish it.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

namespace railbreak {

template <class Payload>
struct HeldSample {
  double t = 0.0;
  std::uint64_t seq = 0;
  Payload payload{};
};

template <class Payload>
class InputReorder {
 public:
  void set_hold(double hold_s) {
    hold_s_ = std::isfinite(hold_s) && hold_s > 0.0 ? hold_s : 0.0;
  }

  double hold_s() const { return hold_s_; }

  void push(double t, Payload payload) {
    if (!std::isfinite(t)) return;
    q_.push_back(HeldSample<Payload>{t, seq_++, std::move(payload)});
    if (!have_max_ || t > t_max_) t_max_ = t;
    have_max_ = true;
  }

  // Samples whose stamp is at least hold_s behind the newest stamp seen.
  std::vector<HeldSample<Payload>> drain() {
    std::vector<HeldSample<Payload>> ready;
    if (!have_max_ || q_.empty()) return ready;
    const double limit = t_max_ - hold_s_;
    std::vector<HeldSample<Payload>> keep;
    keep.reserve(q_.size());
    for (auto& item : q_) {
      if (item.t <= limit) ready.push_back(std::move(item));
      else keep.push_back(std::move(item));
    }
    q_.swap(keep);
    std::stable_sort(ready.begin(), ready.end(), [](const HeldSample<Payload>& a,
                                                     const HeldSample<Payload>& b) {
      if (a.t < b.t) return true;
      if (b.t < a.t) return false;
      return a.seq < b.seq;
    });
    return ready;
  }

  std::size_t pending() const { return q_.size(); }

 private:
  double hold_s_ = 0.0;
  double t_max_ = 0.0;
  bool have_max_ = false;
  std::uint64_t seq_ = 0;
  std::vector<HeldSample<Payload>> q_;
};

}  // namespace railbreak
