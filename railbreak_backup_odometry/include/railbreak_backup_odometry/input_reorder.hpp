#pragma once

// Event-driven order for the three vehicle streams. There is no timer and no
// /clock: each callback pushes a sample and releases only what is already
// covered by every live stream.
//
//   t_watermark = min(t_latest of live streams, cap) - hold_s
//
// A stream that has not spoken yet is not in the min, so one topic can start
// the ride. A stream whose latest stamp is more than stall_s behind the
// freshest is left out of the min. That is ORDER_NOT_RESTORED. While the lead
// is at most kAbandonS, the cap is that stream's latest plus stall_s: one
// early stamp must not publish over samples the other streams have not
// delivered yet. A larger lead no longer caps the watermark, so a stream that
// has stopped cannot hold the output for the rest of the ride. A sample at or
// under the watermark is still released in stamp order, and the node counts it in
// n_behind_out if the output has already passed it. Samples are not discarded
// inside this queue.
//
// A push without a stream id uses one anonymous stream and the same hold
// against the newest stamp. That is the single-topic path.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace railbreak {

template <class Payload>
struct HeldSample {
  double t = 0.0;
  std::uint64_t seq = 0;
  int stream = -1;
  Payload payload{};
};

template <class Payload>
struct ReorderDrain {
  std::vector<HeldSample<Payload>> ready;
  double watermark = std::numeric_limits<double>::quiet_NaN();
  const char* reason = "";
};

template <class Payload>
class InputReorder {
 public:
  static constexpr int kMaxStreams = 4;
  static constexpr double kDefaultStallS = 1.0;
  // Larger than the 2.3 s stamp lead at the start of 30618_e9a34502. A dead
  // stream is released after this lead; the start-of-bag reorder is not.
  static constexpr double kAbandonS = 5.0;

  void set_hold(double hold_s) {
    hold_s_ = std::isfinite(hold_s) && hold_s > 0.0 ? hold_s : 0.0;
  }

  void set_stall(double stall_s) {
    stall_s_ = std::isfinite(stall_s) && stall_s > 0.0 ? stall_s : kDefaultStallS;
  }

  double hold_s() const { return hold_s_; }
  double stall_s() const { return stall_s_; }

  // stream < 0 is the anonymous single-topic queue.
  void push(double t, Payload payload, int stream = -1) {
    if (!std::isfinite(t)) return;
    if (stream >= kMaxStreams) stream = -1;
    q_.push_back(HeldSample<Payload>{t, seq_++, stream, std::move(payload)});
    if (!have_max_ || t > t_max_) t_max_ = t;
    have_max_ = true;
    if (stream >= 0) {
      if (!seen_[stream] || t > latest_[stream]) latest_[stream] = t;
      seen_[stream] = true;
      named_ = true;
    }
  }

  ReorderDrain<Payload> drain() {
    ReorderDrain<Payload> out;
    if (!have_max_ || q_.empty()) return out;
    out.watermark = watermark(out.reason);
    std::vector<HeldSample<Payload>> keep;
    keep.reserve(q_.size());
    for (auto& item : q_) {
      if (item.t <= out.watermark) out.ready.push_back(std::move(item));
      else keep.push_back(std::move(item));
    }
    q_.swap(keep);
    std::stable_sort(out.ready.begin(), out.ready.end(),
                     [](const HeldSample<Payload>& a, const HeldSample<Payload>& b) {
                       if (a.t < b.t) return true;
                       if (b.t < a.t) return false;
                       return a.seq < b.seq;
                     });
    return out;
  }

  std::size_t pending() const { return q_.size(); }

 private:
  double watermark(const char*& reason) const {
    reason = "";
    if (!named_) return t_max_ - hold_s_;
    double freshest = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < kMaxStreams; ++i) {
      if (seen_[i]) freshest = std::max(freshest, latest_[i]);
    }
    double slow = std::numeric_limits<double>::infinity();
    double cap = std::numeric_limits<double>::infinity();
    bool excluded = false;
    for (int i = 0; i < kMaxStreams; ++i) {
      if (!seen_[i]) continue;
      if (freshest - latest_[i] > stall_s_) {
        excluded = true;
        // Do not follow the early stamp past what this stream could still deliver.
        // A lead past kAbandonS is a stopped stream: leave the cap off.
        if (freshest - latest_[i] <= kAbandonS)
          cap = std::min(cap, latest_[i] + stall_s_);
        continue;
      }
      slow = std::min(slow, latest_[i]);
    }
    if (excluded) reason = "ORDER_NOT_RESTORED";
    if (!std::isfinite(slow)) slow = freshest;
    slow = std::min(slow, cap);
    return slow - hold_s_;
  }

  double hold_s_ = 0.0;
  double stall_s_ = kDefaultStallS;
  double t_max_ = 0.0;
  bool have_max_ = false;
  bool named_ = false;
  bool seen_[kMaxStreams] = {};
  double latest_[kMaxStreams] = {};
  std::uint64_t seq_ = 0;
  std::vector<HeldSample<Payload>> q_;
};

}  // namespace railbreak
