// Host replay of the velocity the node publishes. No ROS and no GNSS.
//   replay_velocity <assets_dir> <events.csv> <out.csv>
// events.csv: t,stream,value
//   stream 0 front bogie, 1 rear bogie, 2 driver notch
//   bogie value is the bag field, km/h; the core applies wheel_unit_scale
// The command stream does not hold the watermark. Samples behind the last
// applied input still update the filter and are not published.
// GNSS snaps and the integrity "velocity NONE" gate are not in this replay.

#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>

#include "railbreak_backup_odometry/extrap_stamp.hpp"
#include "railbreak_backup_odometry/input_reorder.hpp"
#include "railbreak_backup_odometry/track_odometer.hpp"
#include "railbreak_backup_odometry/velocity_output.hpp"

namespace {

struct Sample {
  bool cmd = false;
  bool front = false;
  double value = 0.0;
};

struct Publisher {
  double offset_s = 0.105;
  double fade_s = 0.5;
  double step_s = 0.04;
  double fade = 0.0;
  double fade_t = 0.0;
  bool have_fade_t = false;
  double pair_t = 0.0;
  double pair_v = 0.0;
  bool have_pair = false;
  double last_header = 0.0;
  bool have_header = false;
  int n_out = 0;
  int n_untrusted = 0;
  int n_partial_fade = 0;
  std::ofstream* out = nullptr;

  void emit(double t_sample, double v) {
    const double header = railbreak::velocity_output_stamp(t_sample, offset_s);
    if (!std::isfinite(header) || !std::isfinite(v)) return;
    if (have_header && !(header > last_header)) return;
    last_header = header;
    have_header = true;
    ++n_out;
    (*out) << header << ',' << v << '\n';
  }

  void on_input(double t_stamp, double wheels, double filter_v, bool trusted, bool from_bogie,
                bool mates) {
    double dt = 0.0;
    if (have_fade_t) dt = t_stamp - fade_t;
    if (!std::isfinite(dt) || dt < 0.0) dt = 0.0;
    fade_t = t_stamp;
    have_fade_t = true;
    const double v_out =
        railbreak::blend_velocity(wheels, filter_v, trusted, dt, fade_s, fade);
    const bool emit_vel = !trusted || (from_bogie && mates);
    if (!emit_vel) return;
    if (!trusted) ++n_untrusted;
    if (fade < 0.999) ++n_partial_fade;
    const bool grid = from_bogie && mates && trusted && step_s > 0.0;
    const double gap = have_pair ? t_stamp - pair_t : 0.0;
    const bool gap_ok = grid && gap > 0.0 && gap <= 0.35;
    double ts[32];
    double vs[32];
    const int n = railbreak::velocity_resample(gap_ok ? pair_t : std::numeric_limits<double>::quiet_NaN(),
                                               gap_ok ? pair_v : std::numeric_limits<double>::quiet_NaN(),
                                               t_stamp, v_out, step_s, ts, vs, 32);
    for (int i = 0; i < n; ++i) emit(ts[i], vs[i]);
    if (grid) {
      pair_t = t_stamp;
      pair_v = v_out;
      have_pair = true;
    } else if (!trusted) {
      have_pair = false;
    }
  }
};

railbreak::Params node_params() {
  railbreak::Params p;
  p.unit = 1.0 / 3.6;
  p.sigma_k0 = 0.0015;
  p.q_v = 0.05;
  p.q_s = 0.0001;
  p.q_k = 1.0e-9;
  p.q_ba = 1.0e-5;
  p.nis_gate = 16.0;
  p.fr_sigma_gate = 4.0;
  p.fr_floor = 0.3;
  p.zupt_v = 0.05;
  p.zupt_hold_s = 1.0;
  p.zupt_v_max = 0.5;
  p.stop_gate = 3.0;
  p.load_factor = 1.0;
  return p;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: replay_velocity <assets_dir> <events.csv> <out.csv>\n";
    return 2;
  }
  const railbreak::Assets assets = railbreak::load_assets(argv[1]);
  railbreak::TrackOdometer od(&assets, node_params());
  railbreak::InputReorder<Sample> reorder;
  reorder.set_hold(0.0);
  reorder.set_stall(1.0);
  reorder.ignore_watermark(2);

  std::ifstream in(argv[2]);
  std::ofstream out(argv[3]);
  if (!in || !out) {
    std::cerr << "cannot open events or output\n";
    return 2;
  }
  out.setf(std::ios::fixed);
  out.precision(9);
  out << "t,v\n";
  Publisher pub;
  pub.out = &out;

  std::string line;
  std::getline(in, line);
  bool have_out = false;
  double t_out = 0.0;
  int n_in = 0;
  int n_behind = 0;
  while (std::getline(in, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::string cell;
    double fields[3] = {0, 0, 0};
    for (int i = 0; i < 3 && std::getline(ss, cell, ','); ++i) fields[i] = std::stod(cell);
    const double t = fields[0];
    const int stream = static_cast<int>(fields[1]);
    Sample sample;
    sample.cmd = stream == 2;
    sample.front = stream == 0;
    sample.value = fields[2];
    if (!std::isfinite(t)) continue;
    ++n_in;
    reorder.push(t, sample, stream);
    const auto drained = reorder.drain();
    for (const auto& item : drained.ready) {
      const bool behind = have_out && item.t < t_out;
      if (item.payload.cmd) od.on_cmd(item.t, static_cast<int>(item.payload.value));
      else od.on_bogie(item.t, item.payload.front, item.payload.value);
      if (behind) {
        ++n_behind;
        continue;
      }
      if (!item.payload.cmd) {
        const bool mates = std::fabs(od.front_t() - od.rear_t()) < 1e-3;
        pub.on_input(item.t, od.wheels_mean_mps(), od.v(), od.wheels_trusted(), true, mates);
      } else if (!od.wheels_trusted()) {
        pub.on_input(item.t, od.wheels_mean_mps(), od.v(), false, false, false);
      }
      t_out = item.t;
      have_out = true;
    }
  }
  std::cerr << "events " << n_in << " behind " << n_behind << " published " << pub.n_out
            << " untrusted_inputs " << pub.n_untrusted << " partial_fade " << pub.n_partial_fade
            << "\n";
  return 0;
}
