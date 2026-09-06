#include "tram_dr_localization/modes.hpp"

namespace tram_dr {

const char* confidence_label(Confidence c) {
  switch (c) {
    case Confidence::kUninitialized:
      return "UNINITIALIZED";
    case Confidence::kOk:
      return "OK";
    case Confidence::kDegraded:
      return "DEGRADED";
    case Confidence::kLost:
      return "LOST";
  }
  return "UNINITIALIZED";
}

const char* mode_label(Mode m) {
  switch (m) {
    case Mode::kNormal:
      return "normal";
    case Mode::kSlip:
      return "slip";
    case Mode::kSlide:
      return "slide";
    case Mode::kSensorFault:
      return "sensor_fault";
    case Mode::kStandstill:
      return "standstill";
  }
  return "normal";
}

}  // namespace tram_dr
