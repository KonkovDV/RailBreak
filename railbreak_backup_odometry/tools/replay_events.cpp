// Offline replay of an event CSV through the same core as the ROS node.
//   replay_events <assets_dir> <events.csv> <out.csv>
// events.csv: t,kind,value,s0,sigma_s0  (kind: 0 front, 1 rear, 2 cmd; first row carries init)
#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "railbreak_backup_odometry/track_odometer.hpp"

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: replay_events <assets_dir> <events.csv> <out.csv>\n";
    return 2;
  }
  const railbreak::Assets assets = railbreak::load_assets(argv[1]);
  railbreak::TrackOdometer od(&assets, railbreak::Params{});
  std::ifstream in(argv[2]);
  std::ofstream out(argv[3]);
  out.precision(10);
  out << "t,s,v,k,sigma_s,mode\n";
  std::string line;
  std::getline(in, line);
  bool first = true;
  while (std::getline(in, line)) {
    std::stringstream ss(line);
    std::string c;
    double f[5] = {0, 0, 0, 0, 0};
    for (int i = 0; i < 5 && std::getline(ss, c, ','); ++i) f[i] = std::stod(c);
    const double t = f[0];
    const int kind = static_cast<int>(f[1]);
    if (first) {
      od.init(f[3], f[4]);
      od.set_time(t);
      first = false;
    }
    if (kind == 2) {
      od.on_cmd(t, static_cast<int>(f[2]));
    } else {
      od.on_bogie(t, kind == 0, f[2]);
    }
    out << t << ',' << od.s() << ',' << od.v() << ',' << od.k() << ',' << od.sigma_s() << ','
        << static_cast<int>(od.mode()) << '\n';
  }
  return 0;
}
