#pragma once
#include <stdint.h>

// Time-based confirmation: high sampling rates must not shorten debounce.
class PageConfirmation {
 public:
  void reset() { active = false; samples = 0; }
  bool update(uint32_t now, bool eligible, bool matchesCandidate, bool firstPage) {
    if (!eligible) { reset(); return false; }
    if (!active || !matchesCandidate) {
      active = true; since = now; samples = 1; return false;
    }
    if (samples < 255) ++samples;
    return samples >= 2 && static_cast<uint32_t>(now - since) >=
           (firstPage ? 250U : 200U);
  }
 private:
  bool active = false;
  uint32_t since = 0;
  uint8_t samples = 0;
};
