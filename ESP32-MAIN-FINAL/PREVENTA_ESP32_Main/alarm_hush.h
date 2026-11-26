#pragma once

#include <stdint.h>

namespace preventa {

enum class ShortPressAction { ToggleStandby, HushBuzzer };

inline ShortPressAction shortPressAction(bool standby, int effectiveTier) {
    return !standby && effectiveTier >= 3
        ? ShortPressAction::HushBuzzer : ShortPressAction::ToggleStandby;
}

// Local audible acknowledgement only; never changes detection or relay state.
class AlarmHush {
public:
    static constexpr uint32_t durationMs = 120000UL;

    bool acknowledge(int tier, uint32_t now) {
        if (tier < 3) return false;
        active_ = true;
        tier_ = tier;
        started_ = now;
        return true;
    }

    bool active(int tier, uint32_t now) {
        if (active_ && (tier < 3 || tier > tier_
                || static_cast<uint32_t>(now - started_) >= durationMs)) {
            clear();
        }
        return active_;
    }

    uint32_t remainingMs(int tier, uint32_t now) {
        return active(tier, now)
            ? durationMs - static_cast<uint32_t>(now - started_) : 0;
    }

    void clear() { active_ = false; }

private:
    bool active_ = false;
    int tier_ = 0;
    uint32_t started_ = 0;
};

}  // namespace preventa
