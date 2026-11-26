#pragma once

#include <ArduinoJson.h>
#include <math.h>
#include <string.h>

// The deployed API uses a hostname certificate chaining to ISRG Root X1.
constexpr char CLOUD_HTTPS_ORIGIN[] = "https://preventa.duckdns.org";
constexpr char CLOUD_CA_PEM[] = R"PEM(-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)PEM";

struct RemoteSettings {
    char version[65] = {};
    char smsRecipient[17] = {};
    float ppmAdvisory = 50.0f;
    float tempAdvisory = 38.0f;
    float currentAdvisory = 8.0f;
    float humidityAdvisory = NAN;
    unsigned int reportIntervalSec = 2;
    bool cameraEnabled = true;
};

inline bool validVersion(const char* version) {
    return strlen(version) == 64 && strspn(version, "0123456789abcdef") == 64;
}

inline bool validSmsNumber(const char* number) {
    const size_t len = strlen(number);
    return len >= 9 && len <= 16 && number[0] == '+'
        && number[1] >= '1' && number[1] <= '9'
        && strspn(number + 1, "0123456789") == len - 1;
}

inline bool readThreshold(JsonVariantConst value, float minimum, float maximum, float& target) {
    if (value.isNull()) return true;
    if (!value.is<float>()) return false;
    const float parsed = value.as<float>();
    if (!isfinite(parsed) || parsed < minimum || parsed > maximum) return false;
    target = parsed;
    return true;
}

inline bool parseRemoteSettings(JsonObjectConst doc, const char* deviceId, RemoteSettings& setting) {
    if (strcmp(doc["device_id"] | "", deviceId) != 0) return false;
    const char* version = doc["config_version"] | "";
    if (!validVersion(version) || !doc["camera_enabled"].is<bool>()) return false;
    if (!doc.containsKey("sms_recipient")) return false;
    JsonVariantConst number = doc["sms_recipient"];
    if (!number.isNull()) {
        if (!number.is<const char*>() || !validSmsNumber(number.as<const char*>())) return false;
        strcpy(setting.smsRecipient, number.as<const char*>());
    }
    JsonVariantConst interval = doc["sample_interval_sec"];
    if (!interval.isNull()) {
        if (!interval.is<unsigned int>()) return false;
        const unsigned int seconds = interval.as<unsigned int>();
        if (seconds < 2 || seconds > 60) return false;
        setting.reportIntervalSec = seconds;
    }
    if (!readThreshold(doc["ppm_threshold"], 1, 50, setting.ppmAdvisory)
        || !readThreshold(doc["temperature_threshold"], 1, 38, setting.tempAdvisory)
        || !readThreshold(doc["current_threshold"], 0.1f, 8, setting.currentAdvisory)
        || !readThreshold(doc["humidity_threshold"], 1, 100, setting.humidityAdvisory)) {
        return false;
    }
    setting.cameraEnabled = doc["camera_enabled"].as<bool>();
    strcpy(setting.version, version);
    return true;
}
