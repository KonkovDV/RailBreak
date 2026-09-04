#pragma once

#include <vector>

namespace tram_dr {

struct MapNode {
  double s_m;
  double lat_deg;
  double lon_deg;
};

// Polyline s → lat/lon on explicit nodes (≥2, s strictly increasing).
// Returns false on a degenerate node list. No GNSS receiver.
bool project_s(double s_m, double& lat_deg, double& lon_deg,
               const std::vector<MapNode>& nodes);

// Fallback polyline: OSM stop vertices of route 10 (not a survey).
// Used when the node receives no route_* parameters.
bool project_s(double s_m, double& lat_deg, double& lon_deg);

}  // namespace tram_dr
