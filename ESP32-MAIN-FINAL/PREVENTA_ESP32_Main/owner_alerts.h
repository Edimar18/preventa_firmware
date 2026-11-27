#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>

namespace preventa {
constexpr uint32_t pairingDurationMs = 300000UL;
inline bool pairingWindowOpen(uint32_t now, uint32_t started) {
    return static_cast<uint32_t>(now - started) < pairingDurationMs;
}
inline void formatOwnerSms(char* output, size_t capacity, int tier,
                           const char* location, const char* deviceId, bool test) {
    const char* place = location && location[0] ? location : "Location not set";
    if (test) {
        snprintf(output, capacity, "PREVENTA TEST T%d | %.40s | ID %.20s. Fire source N/A. Test only; no emergency action required.", tier, place, deviceId);
        return;
    }
    const char* level = tier >= 5 ? "EMERGENCY" : tier == 4 ? "DANGER" : "WARNING";
    const char* action = tier >= 5 ? "Leave if unsafe. Contact emergency services."
        : tier == 4 ? "Potential fire risk. Leave the affected area if unsafe."
        : "Potential fire risk. Check PREVENTA and assess safely.";
    snprintf(output, capacity, "PREVENTA T%d %s | %.40s | ID %.20s. %s", tier, level, place, deviceId, action);
}
}  // namespace preventa
