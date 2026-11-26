"""Exercise the firmware's actual hush policy and button/output integration."""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SKETCH = ROOT / "ESP32-MAIN-FINAL/PREVENTA_ESP32_Main"


class AlarmHushTests(unittest.TestCase):
    def test_button_timer_escalation_and_physical_output(self):
        source = (SKETCH / "PREVENTA_ESP32_Main.ino").read_text()
        helpers = re.search(
            r"int effectiveAlertTier\(\) \{.*?\n\}\n\nbool updateBuzzer\(bool active\) \{.*?\n\}",
            source, re.S,
        ).group()
        harness = r'''
#include <cassert>
#include <algorithm>
#include <cstdint>
#include "alarm_hush.h"
using std::max;
preventa::AlarmHush g_alarm_hush;
bool g_buzzer_active = false, g_standby_active = false, ENABLE_BUZZER = true;
int g_ui_tier = 0, g_tier_test_override = -1, output = -1;
constexpr int HIGH = 1, LOW = 0, BUZZER_PIN = 26;
uint32_t clockMs = 0;
uint32_t millis() { return clockMs; }
void digitalWrite(int pin, int value) { assert(pin == 26); output = value; }
''' + helpers + r'''
int main() {
    using preventa::ShortPressAction;
    for (int tier = 0; tier <= 5; ++tier) {
        assert(preventa::shortPressAction(true, tier) == ShortPressAction::ToggleStandby);
        assert(preventa::shortPressAction(false, tier) == (tier >= 3
            ? ShortPressAction::HushBuzzer : ShortPressAction::ToggleStandby));
    }
    assert(!g_alarm_hush.acknowledge(2, 0));
    g_ui_tier = 4;
    assert(updateBuzzer(true) && output == HIGH);
    assert(g_alarm_hush.acknowledge(4, 100));
    clockMs = 100;
    assert(!updateBuzzer(true) && output == LOW);
    clockMs = 120099;
    assert(!updateBuzzer(true));
    assert(g_alarm_hush.remainingMs(4, clockMs) == 1);
    clockMs = 120100;
    assert(updateBuzzer(true) && output == HIGH);
    assert(g_alarm_hush.remainingMs(4, clockMs) == 0);
    // A new higher tier immediately restores the audible alarm.
    g_ui_tier = 3; clockMs = 200000;
    g_alarm_hush.acknowledge(3, clockMs);
    assert(!updateBuzzer(false));
    g_ui_tier = 4;
    assert(updateBuzzer(true));
    g_alarm_hush.acknowledge(4, clockMs);
    g_ui_tier = 5;
    assert(updateBuzzer(true));
    // A lower warning tier keeps the acknowledgement until recovery below 3.
    g_alarm_hush.acknowledge(5, clockMs);
    assert(g_alarm_hush.active(4, clockMs + 1));
    assert(!g_alarm_hush.active(2, clockMs + 2));
    assert(!g_alarm_hush.active(4, clockMs + 3));
    // Repeated press starts a fresh two minutes; no timeout accumulation.
    g_alarm_hush.acknowledge(4, 0);
    g_alarm_hush.acknowledge(4, 100000);
    assert(g_alarm_hush.remainingMs(4, 120000) == 100000);
    assert(!g_alarm_hush.active(4, 220000));
    // uint32 millis rollover.
    const uint32_t start = UINT32_MAX - 500;
    g_alarm_hush.acknowledge(4, start);
    assert(g_alarm_hush.remainingMs(4, start + uint32_t(119999)) == 1);
    assert(!g_alarm_hush.active(4, start + uint32_t(120000)));
    // A test cannot hide a higher live tier; standby always drives LOW.
    g_ui_tier = 5; g_tier_test_override = 3;
    assert(effectiveAlertTier() == 5 && updateBuzzer(true));
    g_ui_tier = 0; g_tier_test_override = 4;
    assert(effectiveAlertTier() == 4 && updateBuzzer(true));
    g_standby_active = true;
    assert(!updateBuzzer(true) && output == LOW);
    g_standby_active = false; ENABLE_BUZZER = false;
    assert(!updateBuzzer(true) && output == LOW);
}
'''
        with tempfile.TemporaryDirectory(prefix="preventa-hush-") as temp:
            cpp, binary = Path(temp) / "test.cpp", Path(temp) / "test"
            cpp.write_text(harness)
            subprocess.run([
                "g++", "-std=c++11", "-Wall", "-Wextra", "-Werror",
                "-I", str(SKETCH), str(cpp), "-o", str(binary),
            ], check=True)
            subprocess.run([str(binary)], check=True)


    def test_real_button_handler_routes_release_and_preserves_long_press(self):
        source = (SKETCH / "PREVENTA_ESP32_Main.ino").read_text()
        handler = re.search(r"void handleDiagnosticButton\(\) \{.*?\n\}", source, re.S).group()
        harness = r'''
#include <cassert>
#include <cstdint>
#include "alarm_hush.h"
preventa::AlarmHush g_alarm_hush;
bool g_standby_active = false, pressed = false;
uint32_t clockMs = 0;
int tier = 0, standbyCalls = 0, portalCalls = 0, buzzerCalls = 0;
constexpr int LOW = 0, HIGH = 1, DIAG_BUTTON_PIN = 33, SYSTEM_LED_PIN = 14;
struct Log { void println(const char*) {} template<class... A> void printf(const char*, A...) {} } Serial;
int digitalRead(int) { return pressed ? LOW : HIGH; }
void digitalWrite(int, int) {}
void delay(unsigned ms) { clockMs += ms; }
uint32_t millis() { return clockMs; }
int effectiveAlertTier() { return tier; }
void updateBuzzer(bool) { ++buzzerCalls; }
void toggleStandbyMode() { ++standbyCalls; g_standby_active = !g_standby_active; }
void toggleSetupPortal() { ++portalCalls; }
''' + handler + r'''
void press(uint32_t duration, bool heldFeedback = false) {
    pressed = true; handleDiagnosticButton();
    clockMs += duration;
    if (heldFeedback) handleDiagnosticButton();
    pressed = false; handleDiagnosticButton();
    clockMs += 10;
}
int main() {
    press(79); assert(standbyCalls == 0);
    tier = 2; press(80); assert(standbyCalls == 1 && g_standby_active);
    tier = 5; press(100); assert(standbyCalls == 2 && !g_standby_active);
    for (tier = 3; tier <= 5; ++tier) {
        const int before = buzzerCalls;
        press(100);
        assert(standbyCalls == 2 && !g_standby_active);
        assert(g_alarm_hush.active(tier, clockMs) && buzzerCalls == before + 1);
    }
    press(2000); press(2999);
    assert(standbyCalls == 2 && portalCalls == 0);
    press(3000, true);
    assert(portalCalls == 1 && standbyCalls == 2);
    tier = 0; press(1999);
    assert(standbyCalls == 3 && g_standby_active);
}
'''
        with tempfile.TemporaryDirectory(prefix="preventa-button-") as temp:
            cpp, binary = Path(temp) / "test.cpp", Path(temp) / "test"
            cpp.write_text(harness)
            subprocess.run(["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror",
                            "-I", str(SKETCH), str(cpp), "-o", str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    unittest.main()
