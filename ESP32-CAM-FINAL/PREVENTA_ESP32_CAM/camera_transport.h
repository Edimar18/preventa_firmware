#pragma once
#include <stdint.h>
#include <stddef.h>

namespace preventa {
constexpr uint32_t cameraRequestTimeoutMs = 45000UL;
constexpr uint32_t cameraStreamStallMs = 10000UL;
constexpr size_t cameraMaxLineBytes = 127;
inline bool cameraDeadlineExpired(uint32_t now, uint32_t started) {
    return static_cast<uint32_t>(now - started) >= cameraRequestTimeoutMs;
}
inline uint32_t cameraPreviewChecksum(const uint8_t* bytes, size_t length) {
    uint32_t result = 2166136261UL;
    for (size_t i = 0; i < length; ++i) result = (result ^ bytes[i]) * 16777619UL;
    return result;
}
} // namespace preventa
