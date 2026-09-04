#include "tram_dr_localization/map.hpp"

#include <algorithm>
#include <cmath>

namespace tram_dr {
namespace {

// Fallback = OSM stop vertices of route 10 (same numbers as route_10.yaml).
// Not a survey. Overridden by route_* params when they parse.

constexpr MapNode kDefaultNodes[] = {
    {0.0, 55.810004, 37.461373},
    {174.7, 55.808538, 37.460364},
    {408.4, 55.806844, 37.458153},
    {2794.0, 55.804351, 37.420237},
    {3222.8, 55.805250, 37.413564},
    {3711.1, 55.802060, 37.408193},
    {4196.0, 55.799240, 37.402275},
    {4605.4, 55.799361, 37.395728},
    {4832.3, 55.800089, 37.392337},
};

}  // namespace

bool project_s(double s_m, double& lat_deg, double& lon_deg,
               const std::vector<MapNode>& nodes) {
  if (nodes.size() < 2) {
    return false;
  }
  const double s = std::max(0.0, s_m);
  if (s <= nodes.front().s_m) {
    lat_deg = nodes.front().lat_deg;
    lon_deg = nodes.front().lon_deg;
    return true;
  }
  for (std::size_t i = 0; i + 1 < nodes.size(); ++i) {
    if (s <= nodes[i + 1].s_m) {
      const double t =
          (s - nodes[i].s_m) / std::max(nodes[i + 1].s_m - nodes[i].s_m, 1e-6);
      lat_deg = nodes[i].lat_deg + t * (nodes[i + 1].lat_deg - nodes[i].lat_deg);
      lon_deg = nodes[i].lon_deg + t * (nodes[i + 1].lon_deg - nodes[i].lon_deg);
      return true;
    }
  }
  lat_deg = nodes.back().lat_deg;
  lon_deg = nodes.back().lon_deg;
  return true;
}

bool project_s(double s_m, double& lat_deg, double& lon_deg) {
  return project_s(s_m, lat_deg, lon_deg,
                   std::vector<MapNode>(std::begin(kDefaultNodes), std::end(kDefaultNodes)));
}

}  // namespace tram_dr
