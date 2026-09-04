#pragma once

namespace tram_dr {

enum class Mode { kNormal, kSlip, kSlide, kSensorFault, kStandstill };

enum class Confidence { kUninitialized, kOk, kDegraded, kLost };

const char* confidence_label(Confidence c);
const char* mode_label(Mode m);

}  // namespace tram_dr
