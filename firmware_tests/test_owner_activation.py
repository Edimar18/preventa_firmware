"""Host checks for the actual remote-config parser, pairing clock and SMS formatter."""
from pathlib import Path
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SKETCH = ROOT / 'ESP32-MAIN-FINAL/PREVENTA_ESP32_Main'
ARDUINO_JSON = Path.home() / 'Arduino/libraries/ArduinoJson/src'


class OwnerActivationTests(unittest.TestCase):
    def test_activation_parser_pairing_expiry_and_room_sms(self):
        if not ARDUINO_JSON.exists():
            self.skipTest('Install ArduinoJson 7 to exercise the real firmware parser')
        harness = r'''
#include <cassert>
#include <cstring>
#include <cstdint>
#include "remote_config.h"
#include "owner_alerts.h"
int main() {
    JsonDocument doc;
    doc["device_id"] = "PREVENTA-TEST";
    doc["config_version"] = "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
    doc["activation_generation"] = "01234567-89ab-cdef-0123-456789abcdef";
    doc["activation_state"] = "unclaimed";
    doc["name"] = "Kitchen monitor";
    doc["location"] = "Kitchen";
    doc["camera_enabled"] = true;
    doc["sms_recipient"] = nullptr;
    RemoteSettings settings;
    assert(parseRemoteSettings(doc.as<JsonObjectConst>(), "PREVENTA-TEST", settings));
    assert(strcmp(settings.activationState, "unclaimed") == 0 && !settings.smsRecipient[0]);
    assert(!parseRemoteSettings(doc.as<JsonObjectConst>(), "OTHER-DEVICE", settings));
    doc["activation_state"] = "active";
    doc["sms_recipient"] = "+12025550123";
    assert(parseRemoteSettings(doc.as<JsonObjectConst>(), "PREVENTA-TEST", settings));
    assert(strcmp(settings.location, "Kitchen") == 0 && strcmp(settings.activationState,"active") == 0);
    doc.remove("activation_state");
    assert(!parseRemoteSettings(doc.as<JsonObjectConst>(), "PREVENTA-TEST", settings));
    doc["activation_state"] = "active";
    doc["activation_generation"] = "missing";
    assert(!parseRemoteSettings(doc.as<JsonObjectConst>(), "PREVENTA-TEST", settings));
    uint32_t start = UINT32_MAX - 500;
    assert(preventa::pairingWindowOpen(start+299999U,start));
    assert(!preventa::pairingWindowOpen(start+300000U,start));
    char sms[161];
    for (int tier = 3; tier <= 5; ++tier) {
        preventa::formatOwnerSms(sms,sizeof(sms),tier,"Upstairs bedroom","PREVENTA-TEST0001",false);
        assert(strstr(sms,"Upstairs bedroom") && strstr(sms,"PREVENTA-TEST0001"));
        assert(strlen(sms) <= 160);
        preventa::formatOwnerSms(sms,sizeof(sms),tier,"Kitchen","PREVENTA-TEST0001",true);
        assert(strstr(sms,"TEST") && strstr(sms,"Kitchen") && strstr(sms,"N/A"));
    }
}
'''
        with tempfile.TemporaryDirectory(prefix='preventa-owner-tests-') as temp:
            cpp, binary = Path(temp)/'test.cpp', Path(temp)/'test'
            cpp.write_text(harness)
            subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Wno-deprecated-declarations',
                            '-I',str(SKETCH),'-I',str(ARDUINO_JSON),str(cpp),'-o',str(binary)],check=True)
            subprocess.run([str(binary)],check=True)

    def test_no_sms_fallback_and_all_send_boundaries_are_gated(self):
        source = (SKETCH/'PREVENTA_ESP32_Main.ino').read_text()
        for signature in ['bool sendSmsOnModem(', 'bool sendReadingToBackend(', 'bool postReadingPayloadToBackend(',
                          'bool uploadCaptureToBackend(', 'SetupResult sendSetupTestPush(']:
            # Find the definition rather than its declaration.
            start = source.index(signature, source.index('// ── RUNTIME GLOBAL STATE'))
            while source.find(';', start) < source.find('{', start):
                start = source.index(signature, start+1)
            body = source[source.index('{',start):source.index('{',start)+850]
            self.assertIn('g_owner_active.load()',body,signature)
        start=source.index('const char* smsRecipient() {')
        self.assertNotIn('SMS_RECIPIENT',source[start:source.index('void startCaptivePortal()',start)])
        self.assertIn('preferences.remove("remote_cfg")',source)
        self.assertNotIn('preferences.clear()',source)


if __name__ == '__main__':
    unittest.main()
