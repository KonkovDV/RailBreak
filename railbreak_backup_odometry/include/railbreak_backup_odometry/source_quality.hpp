// Per-source input quality. This is not an integrity status and not a probability.
// missing, stale, outlier, impossible and disagree stay different kinds.
// A forgotten unit that still falls under v_max is not labelled here.
#pragma once

#include <cstring>

namespace railbreak {

struct SourceQuality {
  double timestamp = 0.0;
  double age_s = 1.0e9;
  bool valid = false;
  double unit_scale = 1.0;
  int dropout_counter = 0;
  int outlier_counter = 0;
  int impossible_counter = 0;
  // 1 when kind is ok, otherwise 0. Not a calibrated score.
  double quality_score = 0.0;
  const char* kind = "missing";
};

inline SourceQuality classify_source(bool have, double timestamp, double age_s, double stale_s,
                                    double unit_scale, int dropout_counter, int outlier_counter,
                                    int impossible_counter, const char* reject, bool disagree) {
  SourceQuality q;
  q.timestamp = timestamp;
  q.age_s = have ? age_s : 1.0e9;
  q.unit_scale = unit_scale;
  q.dropout_counter = dropout_counter;
  q.outlier_counter = outlier_counter;
  q.impossible_counter = impossible_counter;
  const bool impossible = reject != nullptr && std::strcmp(reject, "impossible") == 0;
  const bool outlier = reject != nullptr && std::strcmp(reject, "outlier") == 0;
  if (!have && !impossible && !outlier) q.kind = "missing";
  else if (impossible) q.kind = "impossible";
  else if (!have) q.kind = "outlier";
  else if (age_s > stale_s) q.kind = "stale";
  else if (outlier) q.kind = "outlier";
  else if (disagree) q.kind = "disagree";
  else q.kind = "ok";
  q.valid = have && std::strcmp(q.kind, "stale") != 0 && std::strcmp(q.kind, "impossible") != 0 &&
            std::strcmp(q.kind, "missing") != 0;
  q.quality_score = std::strcmp(q.kind, "ok") == 0 ? 1.0 : 0.0;
  return q;
}

}  // namespace railbreak
