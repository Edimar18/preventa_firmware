/**
 * ============================================================================
 * PREVENTA — IoT Multi-Sensor Residential Fire Detection System
 * ESP32 Main Processing Firmware  —  STANDALONE / AUTONOMOUS VERSION
 * ============================================================================
 *
 * ARCHITECTURE OVERVIEW:
 *
 *   [MQ-7] [DHT11/22] [ACS712] ──(direct GPIO/ADC reads, every 2s)──► ESP32 Main
 *                                                                        │
 *                                                          runFullPipeline()
 *                                                     (Steps 2 → 10, Autonomous)
 *                                                                        │
 *                              ┌─────────────────────────────────────────┤
 *                              │                                         │
 *                    HTTP POST to Django backend              UART1 to ESP32-CAM
 *                    (X-Device-Id / X-Device-Key)            (Non-blocking PING/PREDICT)
 *                              │                                         │
 *                    relay / buzzer / tier LEDs                A7670C SMS (Tier ≥ 3)
 *
 * ASYNCHRONOUS NON-BLOCKING CAMERA & IMAGE UPLOAD PIPELINE:
 *   • Main sends "PREDICT" to ESP32-CAM and continues its 2.0-second sensor loop.
 *   • When CAM finishes TFLite inference, it streams the prediction AND the
 *     JPEG image frame over UART to Main.
 *   • Main immediately updates the fusion engine and uploads the image to
 *     POST /api/v1/captures/ so every capture is preserved in the dashboard.
 *
 * CLIENT PAIRING & PROVISIONING:
 *   • Wi-Fi SoftAP + Captive Portal: "PREVENTA-Setup" (IP 192.168.4.1).
 *   • Configuration stored in NVS via Preferences API.
 * ============================================================================
 */

#include <Arduino.h>
#include <ArduinoJson.h>
#include <math.h>
#include <atomic>
#include "alarm_hush.h"
#include "owner_alerts.h"
#if __has_include("secrets.h")
#include "secrets.h"
#else
#include "secrets.example.h"
#endif
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <DHT.h>
#include <time.h>
#include "driver/gpio.h"
#include "esp_system.h"
#include "remote_config.h"

// ============================================================================
// ── DEFAULT / FALLBACK CONFIGURATION (Overrides from NVS if provisioned) ────
// ============================================================================
const char* DEFAULT_WIFI_SSID     = PREVENTA_WIFI_SSID;
const char* DEFAULT_WIFI_PASSWORD = PREVENTA_WIFI_PASSWORD;
const char* DEFAULT_BACKEND_HOST  = "13.214.158.148";
const uint16_t BACKEND_PORT       = 80;
const char* DEFAULT_DEVICE_ID     = PREVENTA_DEVICE_ID;
const char* DEFAULT_DEVICE_KEY    = PREVENTA_DEVICE_KEY;

char g_wifi_ssid[64];
char g_wifi_pass[64];
char g_backend_host[64];
char g_device_id[64];
char g_device_key[128];

String g_backend_readings_url;
String g_backend_captures_url;
String g_backend_config_url;
String g_backend_setup_push_url;
String g_backend_provisioning_url;
String g_backend_activation_url;
std::atomic<bool> g_owner_active{false};
bool g_activation_ack_requested = false;
bool g_pairing_publish_pending = false;
char g_pairing_code[7] = {};
unsigned long g_pairing_started_ms = 0;

struct ConfigFetchRequest {
    bool publishPairing = false;
    bool acknowledge = false;
    char pairingCode[7] = {};
    char version[65] = {};
    char generation[37] = {};
};
RemoteSettings g_remote_settings;
portMUX_TYPE g_owner_settings_lock = portMUX_INITIALIZER_UNLOCKED;



struct ConfigDownload {
    char body[2049];
    int statusCode = 0;
    bool pairingPublished = false;
    char pairingCode[7] = {};
};
QueueHandle_t g_config_downloads = nullptr;
bool g_config_fetch_running = false;
bool g_config_connected = false;
unsigned long g_last_config_fetch_ms = 0;
unsigned long g_last_reading_upload_ms = 0;
int g_last_uploaded_tier = -1;

const unsigned long BACKEND_HTTP_TIMEOUT_MS = 2000;
unsigned long g_backend_backoff_until_ms    = 0;
int g_backend_consecutive_fails             = 0;

// ============================================================================
// ── SOFTAP & CAPTIVE PORTAL ─────────────────────────────────────────────────
// ============================================================================
const char* AP_SSID = PREVENTA_SETUP_SSID;
const byte  DNS_PORT = 53;
IPAddress   apIP(192, 168, 4, 1);
IPAddress   netMsk(255, 255, 255, 0);

WebServer webServer(apIP, 80);
DNSServer dnsServer;
Preferences preferences;

bool g_portal_active = false;
bool g_portal_dns_running = false;
unsigned long g_portal_start_ms = 0;
const unsigned long PORTAL_AUTO_EXIT_TIMEOUT_MS = 5UL * 60UL * 1000UL;

// Below Tier 3: short press toggles software standby. Tier 3+: audible hush.
preventa::AlarmHush g_alarm_hush;
bool g_buzzer_active = false;
bool g_standby_active = false;
unsigned long g_standby_start_ms = 0;
const unsigned long STANDBY_WATCHDOG_TIMEOUT_MS = 45UL * 60UL * 1000UL;

// Alert Tier Tester simulation state
int g_tier_test_override = -1;
bool g_tier_test_relay_cut = false;
unsigned long g_tier_test_start_ms = 0;

enum class SetupResult { Idle, Pending, Accepted, Failed, Offline, NoBrowser, RateLimited, NotActivated };
std::atomic<SetupResult> g_setup_sms_result{SetupResult::Idle};
std::atomic<SetupResult> g_setup_push_result{SetupResult::Idle};
std::atomic<bool> g_setup_alert_running{false};

bool g_setup_capture_pending = false;
SetupResult g_setup_capture_result = SetupResult::Idle;
uint8_t* g_setup_photo = nullptr;
size_t g_setup_photo_size = 0;
constexpr int kModelInputSize = 96;
constexpr size_t kModelInputBmpBytes = 54 + kModelInputSize * kModelInputSize * 3;
uint8_t* g_setup_model_input = nullptr;
unsigned long g_setup_photo_revision = 0;

struct PreviewInference {
    int fire_pct = -1;
    int no_fire_pct = -1;
    unsigned long response_ms = 0;
};
PreviewInference g_setup_inference;

// Cached live sensor telemetry for setup web UI
float g_ui_mq7_ppm = 0.0f;
float g_ui_temp    = 0.0f;
float g_ui_hum     = 0.0f;
float g_ui_current = 0.0f;
int   g_ui_tier    = 0;

// ============================================================================
// ── ESP32-CAM MODULE UART CONFIGURATION (UART1) ─────────────────────────────
// ============================================================================
#define CAM_SERIAL_RX_PIN 21   // ESP32-Main receives here ← CAM TX (GPIO1)
#define CAM_SERIAL_TX_PIN 22   // ESP32-Main transmits here → CAM RX (GPIO3)
HardwareSerial camSerial(1);

// Asynchronous camera state
bool          g_cam_query_pending    = false;
unsigned long g_cam_query_sent_ms    = 0;
unsigned long g_last_cam_query_ms    = 0;
int           g_cam_latest_fire_pct  = 0;
int           g_cam_latest_state     = 0;
bool          g_cam_image_streaming  = false;
String        g_deferred_reading_payload;
int           g_deferred_reading_tier = -1;

const unsigned long CAM_RESPONSE_TIMEOUT_MS = 90000UL;
const unsigned long CAM_IMAGE_STALL_TIMEOUT_MS = 10000UL;

// ============================================================================
// ── SIMCom A7670C CELLULAR MODULE CONFIGURATION (UART2) ─────────────────────
// ============================================================================
const char* SMS_RECIPIENT = PREVENTA_SMS_RECIPIENT;
#define MODEM_RX_PIN 16  // ESP32 receives here <- modem TXD
#define MODEM_TX_PIN 17  // ESP32 transmits here -> modem RXD
#define MODEM_DEFAULT_BAUD 115200
HardwareSerial modemSerial(2);

// ============================================================================
// ── SENSOR SELECTION & PIN DEFINITIONS ──────────────────────────────────────
// ============================================================================
// #define DHT_SENSOR_TYPE DHT11
#define DHT_SENSOR_TYPE DHT22

#define DHT_PIN          27
#define MQ7_ADC_PIN      34
#define ACS712_ADC_PIN   35
#define RELAY_PIN        25
#define BUZZER_PIN       26  // Keep separate from GPIO33 manual-check button
#define DIAG_BUTTON_PIN  33

// Status indicator LEDs
#define POWER_LED_PIN    13
#define SYSTEM_LED_PIN   14
#define NETWORK_LED_PIN  32

// Alert tier LEDs
#define LED_TIER1_PIN  4
#define LED_TIER2_PIN  5
#define LED_TIER3_PIN  18
#define LED_TIER4_PIN  19
#define LED_TIER5_PIN  23

// COM-NC cutoff. Use a 3.3 V-compatible input or an appropriate driver.
constexpr bool RELAY_ACTIVE_LOW = true;
const bool ENABLE_BUZZER       = true;
const bool DEBUG_SERIAL_OUTPUT = true;

// Voltage divider compensation constants (series R1=20k, shunt R2=10k -> 3.0x)
const float MQ7_DIVIDER_COMPENSATION    = 3.0f;
const float ACS712_DIVIDER_COMPENSATION = 3.0f;

// 2.0-second pipeline cadence
const float SAMPLE_INTERVAL_SEC        = 2.0f;
const unsigned long SAMPLE_INTERVAL_MS = (unsigned long)(SAMPLE_INTERVAL_SEC * 1000.0f);

// NTP configuration (Philippine Standard Time, UTC+8)
const char* NTP_SERVER1         = "ph.pool.ntp.org";
const char* NTP_SERVER2         = "pool.ntp.org";
const long  GMT_OFFSET_SEC      = 8 * 3600;
const int   DAYLIGHT_OFFSET_SEC = 0;

// Sensor & baseline constants
DHT dht(DHT_PIN, DHT_SENSOR_TYPE);
float g_last_good_temp = 30.0f;
float g_last_good_hum  = 75.0f;

const float ACS712_NOMINAL_ZERO = 2.5f;
float ACS712_VZERO             = ACS712_NOMINAL_ZERO;
constexpr float ACS712_SENSITIVITY = 0.185f;

enum class AcsZeroResult { Ok, OutOfRange, Unstable, SaveFailed };

const int   BASELINE_SAMPLES         = 30;
const float ALPHA_BASELINE           = 0.01f;
const float EPSILON                  = 0.001f;
const float DEFAULT_BASELINE_PPM     = 100.0f;
const float DEFAULT_BASELINE_TEMP    = 30.0f;
const float DEFAULT_BASELINE_HUM     = 75.0f;
const float DEFAULT_BASELINE_CURRENT = 0.0f;

const int SMS_ALERT_TIER     = 3;
const int POST_ALARM_COOLDOWN = 150;

const char* TIER_NAMES[]   = { "Normal", "Advisory", "Caution", "Warning", "Danger", "Emergency" };
const char* STATE_LABELS[] = { "Normal", "Mild", "Moderate", "Severe" };
const char* FIRE_TYPES[]   = { "unknown", "electrical", "smoldering", "open_fire" };

const char* const SETUP_TEST_SMS = "PREVENTA test message. No emergency action is needed.";

struct SmsAlert {
    char recipient[17];
    char message[161];
    char generation[37];
};

// ============================================================================
// ── DATA STRUCTURES ─────────────────────────────────────────────────────────
// ============================================================================
struct SmoothedData {
    float mq7_ppm;
    float temperature;
    float humidity;
    float acs_current;
};

struct DerivedData {
    float temp_ror;
    float current_delta_A;
    float current_delta_pct;
};

struct SensorStates {
    int mq7;
    int temperature;
    int acs712;
    int camera;
};

struct SensorStreak {
    int state;
    int count;
};

struct PersistenceData {
    int   consecutive_abnormal;
    float abnormal_duration_sec;
    SensorStreak mq7;
    SensorStreak temperature;
    SensorStreak acs712;
};

struct FusionData {
    int  score;
    int  fusion_tier;
    int  fire_type;
    bool recommend_relay_cutoff;
    int  mod_humidity_drying;
    int  mod_weak_evidence;
    int  mod_persistent_dual;
    int  mod_short_spike;
};

struct CameraDecision {
    bool should_wake;
    int  camera_state;
    int  confidence_pct;
    int  warming_ticks;
};

struct ResponseData {
    bool relay_cut;
    bool siren_active;
    bool camera_active;
    bool send_notification;
    int  notif_tier;
};

struct LogData {
    int  transition_type;
    bool in_cooldown;
    int  cooldown_remaining_sec;
    int  peak_tier;
    int  log_entry_count;
};

// ============================================================================
// ── RUNTIME GLOBAL STATE ────────────────────────────────────────────────────
// ============================================================================
float g_prev_smoothed_ppm     = -1.0f;
float g_prev_smoothed_current = -1.0f;
float g_mq7_zero_ppm          = 100.0f;
float g_mq7_zero_volts        = 2.3f;

float g_baseline_ppm          = DEFAULT_BASELINE_PPM;
float g_baseline_temp         = DEFAULT_BASELINE_TEMP;
float g_baseline_hum          = DEFAULT_BASELINE_HUM;
float g_baseline_current      = DEFAULT_BASELINE_CURRENT;
float g_dev_ppm               = 0.0f;
float g_dev_temp              = 0.0f;
float g_dev_hum               = 0.0f;
float g_dev_current           = 0.0f;
int   g_baseline_sample_count = BASELINE_SAMPLES;
bool  g_warmup_active         = false;

// The centers of the first and last three samples are 15 sample intervals apart.
constexpr int ROR_BUFFER_SAMPLES = 18;
float         g_ror_temp_buf[ROR_BUFFER_SAMPLES];
unsigned long g_ror_time_buf[ROR_BUFFER_SAMPLES];
int           g_ror_count = 0;
bool          g_previous_severe_ror = false;
bool          g_previous_severe_mq7_deviation = false;
int           g_previous_acs_delta_direction = 0;

int g_camera_state_prev       = 0;
int g_camera_warming          = 0;
PersistenceData g_persist     = { 0, 0.0f, {0,0}, {0,0}, {0,0} };

int g_last_tier               = 0;
int g_at_tier_since           = 0;
int g_improving_samples       = 0;
int g_current_tier_est        = 0;

int g_sms_peak_tier           = 0;
QueueHandle_t g_sms_alerts    = nullptr;
SemaphoreHandle_t g_modem_lock = nullptr;
int g_post_alarm_ctr          = 0;
int g_peak_tier               = 0;
int g_log_last_tier           = 0;
int g_log_entry_count         = 0;
bool g_event_active           = false;

unsigned long g_last_sample_ms       = 0;
unsigned long g_last_status_blink_ms = 0;
bool g_status_blink_state            = false;

// Hardware presence flags
bool g_system_ok      = true;
bool g_mq7_present    = true;
bool g_dht_present    = true;
bool g_acs712_present = true;
bool g_cam_present    = true;

bool g_auto_relay_cut   = false;
bool g_manual_relay_cut = false;

// ============================================================================
// ── FUNCTION PROTOTYPES ─────────────────────────────────────────────────────
// ============================================================================
void           loadPreferences();
void           savePreferences(const char* ssid, const char* pass, const char* host, const char* dev_id, const char* dev_key);
void           saveWifiCredentials(const char* ssid, const char* pass);
void           startCaptivePortal();
void           stopCaptivePortal();
void           toggleSetupPortal();
void           toggleStandbyMode();
void           handleCaptivePortal();
void           setupWebServer();
bool           requestCameraCapture(bool setupPreview);
void           finishCameraRequest();
void           clearTierTest();
const char*    setupResultName(SetupResult result);
void           sendSetupTestAlerts(void* parameter);
void           rebuildUrls();
void           loadRemoteSettings();
void           pollRemoteSettings();
void           applyCameraSetting();
const char*    smsRecipient();
const char*    testSmsRecipient();

void           setDefaultBaseline();
void           initializeAcsZero();
AcsZeroResult  calibrateAcsZero();
void           runCalibration();
void           runFullPipeline(float raw_mq7_v, float raw_temp, float raw_hum, float raw_acs_v,
                               bool temperature_valid);
bool           selfDiagnose();
bool           checkCameraLink();
void           updateStatusLEDs();
void           processCamSerial();

SmoothedData   stepSmoothing(float raw_mq7_v, float raw_acs_v, float temp, float hum);
void           stepBaselineNormalization(SmoothedData& s);
DerivedData    stepDerivedFeatures(SmoothedData& s, bool temperature_valid);
SensorStates   stepSensorClassification(SmoothedData& s, DerivedData& d);
void           stepPersistenceUpdate(SensorStates& st);
int            stepInitialTierEstimate(SensorStates& st, int& tier_floor);
FusionData     stepFusionLogic(SensorStates& st);
CameraDecision stepCameraWakeDecision(SensorStates& st, FusionData& f, int initial_tier);
int            stepFinalTierDecision(int initial_tier, int tier_floor, FusionData& f, SensorStates& st);
ResponseData   stepResponseEngine(int final_tier);
LogData        stepLoggingAudit(int final_tier, FusionData& f, SensorStates& st);

void           updateLEDs(int final_tier);
bool           relayCutRequested();
int            relayOutputLevel(bool cut);
void           updateRelay();
void           setManualRelayCut(bool cut);
void           runRelayTest(int seconds);
void           printRelayStatus();
int            effectiveAlertTier();
bool           updateBuzzer(bool active);
float          calcDeviation(float current_val, float base_val);
float          noiseSimulate(float amplitude);

void           connectWiFi();
void           maintainWiFi();
void           syncTimeNTP();
String         getTimestampString();
void           readSensorsAndRunPipeline();
bool           sendReadingToBackend(SmoothedData& s, DerivedData& d, SensorStates& st,
                                     FusionData& f, CameraDecision& cd, int initial_tier,
                                     int final_tier, ResponseData& r);
bool           postReadingPayloadToBackend(const String& payload, int final_tier);
void           flushDeferredReadingToBackend();
bool           uploadCaptureToBackend(const uint8_t* jpeg, size_t len, float confidence);
bool           probeModemBaud();
String         sendModemCommand(const String& command, const char* expected,
                                unsigned long timeout_ms);
bool           runModemDiagnostics();
void           sendModemTestSMS();
bool           trySendSMS(const char* number, const char* message);
bool           sendSmsOnModem(const char* number, const char* message);
bool           queueSmsAlert(int final_tier);
void           deliverSmsAlerts(void*);
void           handleDiagnosticButton();
void           triggerManualSelfTest();
void           printDebugSummary(SmoothedData& s, SensorStates& st, FusionData& f,
                                  int final_tier, bool backend_ok);

// ============================================================================
// ── SETUP ───────────────────────────────────────────────────────────────────
// ============================================================================
void copyOwnerSettings(RemoteSettings& output) {
    portENTER_CRITICAL(&g_owner_settings_lock);
    output = g_remote_settings;
    portEXIT_CRITICAL(&g_owner_settings_lock);
}

void storeOwnerSettings(const RemoteSettings& settings) {
    portENTER_CRITICAL(&g_owner_settings_lock);
    g_remote_settings = settings;
    portEXIT_CRITICAL(&g_owner_settings_lock);
}

void setup() {
    // Preload the idle level before enabling the GPIO output.
    gpio_set_level(static_cast<gpio_num_t>(RELAY_PIN), relayOutputLevel(false));
    pinMode(RELAY_PIN, OUTPUT);

    Serial.begin(115200);
    delay(200);
    Serial.println("\n========================================================");
    Serial.println("  PREVENTA Fire Detection — Standalone Controller");
    Serial.println("  Production-Ready Firmware v3.0");
    Serial.println("========================================================");

    // Buzzer initialization
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);

    // Status LEDs
    pinMode(POWER_LED_PIN, OUTPUT);
    pinMode(SYSTEM_LED_PIN, OUTPUT);
    pinMode(NETWORK_LED_PIN, OUTPUT);
    digitalWrite(POWER_LED_PIN, HIGH);
    digitalWrite(SYSTEM_LED_PIN, LOW);
    digitalWrite(NETWORK_LED_PIN, LOW);

    // Diagnostic Button initialization
    pinMode(DIAG_BUTTON_PIN, INPUT_PULLUP);

    // Alert Tier LEDs
    pinMode(LED_TIER1_PIN, OUTPUT);
    pinMode(LED_TIER2_PIN, OUTPUT);
    pinMode(LED_TIER3_PIN, OUTPUT);
    pinMode(LED_TIER4_PIN, OUTPUT);
    pinMode(LED_TIER5_PIN, OUTPUT);
    digitalWrite(LED_TIER1_PIN, LOW);
    digitalWrite(LED_TIER2_PIN, LOW);
    digitalWrite(LED_TIER3_PIN, LOW);
    digitalWrite(LED_TIER4_PIN, LOW);
    digitalWrite(LED_TIER5_PIN, LOW);

    // Boot self-test: Flash all 5 Tier LEDs in sequence
    int bootTestPins[] = {LED_TIER1_PIN, LED_TIER2_PIN, LED_TIER3_PIN, LED_TIER4_PIN, LED_TIER5_PIN};
    for (int i = 0; i < 5; i++) {
        digitalWrite(bootTestPins[i], HIGH);
        digitalWrite(BUZZER_PIN, HIGH);
        delay(80);
        digitalWrite(bootTestPins[i], LOW);
        digitalWrite(BUZZER_PIN, LOW);
    }

    // ADC attenuation (0 - 3.3V full-scale)
    analogSetPinAttenuation(MQ7_ADC_PIN, ADC_11db);
    analogSetPinAttenuation(ACS712_ADC_PIN, ADC_11db);

    // Sensor & Serial interfaces
    dht.begin();
    modemSerial.begin(MODEM_DEFAULT_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
    g_modem_lock = xSemaphoreCreateMutex();
    if (g_modem_lock) {
        g_sms_alerts = xQueueCreate(1, sizeof(SmsAlert));
        if (g_sms_alerts && xTaskCreate(deliverSmsAlerts, "preventa-sms", 5120, nullptr, 1, nullptr) != pdPASS) {
            vQueueDelete(g_sms_alerts);
            g_sms_alerts = nullptr;
        }
    }
    if (!g_sms_alerts) Serial.println("[MODEM] Automatic SMS worker could not start.");
    camSerial.setRxBufferSize(4096);
    camSerial.begin(115200, SERIAL_8N1, CAM_SERIAL_RX_PIN, CAM_SERIAL_TX_PIN);

    // Load persisted preferences (NVS)
    loadPreferences();
    rebuildUrls();
    loadRemoteSettings();

    // 1. Power-on stabilization & circuit warm-up delay (4 seconds)
    // Pulsing System LED indicates warm-up and settling in progress
    Serial.println("[Boot] Waiting 4s for power rails & sensor circuit settling...");
    unsigned long settle_start = millis();
    while (millis() - settle_start < 4000) {
        digitalWrite(SYSTEM_LED_PIN, ((millis() / 150) % 2) ? HIGH : LOW);
        delay(20);
    }

    // 2. Restore the ACS712 zero point after power and filter capacitor have settled
    initializeAcsZero();

    // 3. Ping camera co-processor and request test capture to warm up AEC/AGC
    if (g_remote_settings.cameraEnabled) camSerial.println("TEST_CAPTURE");

    // 4. Initialize baseline with warmup period (g_warmup_active = true)
    runCalibration();
    randomSeed(esp_random());

    // Wi-Fi Connection attempt (12s timeout)
    connectWiFi();
    syncTimeNTP();

    // Comprehensive self-diagnostic (MQ-7, DHT, ACS712, ESP32-CAM link)
    selfDiagnose();
    updateStatusLEDs();

    g_last_sample_ms = millis();
    Serial.println("=== Initialization complete — active monitoring started ===\n");
}

// ============================================================================
// ── MAIN LOOP ───────────────────────────────────────────────────────────────
// ============================================================================
void loop() {
    // 1. Short press: standby below Tier 3, timed buzzer hush at Tier 3+.
    // Long press >= 3s: Setup Portal (unchanged).
    handleDiagnosticButton();

    // 2. Standby / Sleep Mode Handler
    if (g_standby_active) {
        // Safety watchdog: automatically wake up and resume monitoring after 45 minutes
        if (millis() - g_standby_start_ms >= STANDBY_WATCHDOG_TIMEOUT_MS) {
            Serial.println("[Safety Watchdog] 45-minute Standby timer expired. Resuming active monitoring.");
            toggleStandbyMode();
        }
        delay(15);
        return;
    }

    // Refresh on every active loop so hush expiry does not wait for a sensor sample.
    updateBuzzer(effectiveAlertTier() >= 4);

    // 3. Captive Portal Mode Handler
    if (g_portal_active) {
        handleCaptivePortal();
    } else {
        maintainWiFi();
    }

    // 4. Continuously parse incoming prediction & image stream from ESP32-CAM
    processCamSerial();
    pollRemoteSettings();

    // 5. Update status LEDs according to current operating state
    updateStatusLEDs();

    // 6. Tier Tester Simulation auto-timeout (30 seconds)
    if (g_tier_test_override >= 0 && millis() - g_tier_test_start_ms > 30000) {
        Serial.println("[Tier Tester] Simulation timed out (30s). Reverting to live pipeline.");
        clearTierTest();
    }

    // 7. Periodic 2-second sensor acquisition and 10-step detection engine
    unsigned long now = millis();
    if (now - g_last_sample_ms >= SAMPLE_INTERVAL_MS) {
        g_last_sample_ms = now;
        readSensorsAndRunPipeline();
    }

    // 8. Interactive Serial Commands
    if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.equalsIgnoreCase("capture") || cmd.equalsIgnoreCase("snap")) {
            if (requestCameraCapture(g_portal_active)) {
                Serial.println("[CMD] Manual capture requested — waking ESP32-CAM...");
            } else {
                Serial.println("[CMD] Camera capture unavailable: camera disabled or busy.");
            }
        } else if (cmd.equalsIgnoreCase("modem") || cmd.equalsIgnoreCase("modem_status")) {
            runModemDiagnostics();
        } else if (cmd.equalsIgnoreCase("test_sms")) {
            sendModemTestSMS();
        } else if (cmd.equalsIgnoreCase("calibrate")) {
            runCalibration();
            Serial.println("[CMD] Clean-air calibration started (30 samples, ~60s). Keep area clear of smoke.");
        } else if (cmd.equalsIgnoreCase("baseline")) {
            setDefaultBaseline();
            Serial.println("[CMD] Default environmental baseline restored.");
        } else if (cmd.equalsIgnoreCase("status")) {
            selfDiagnose();
            runModemDiagnostics();
        } else if (cmd.equalsIgnoreCase("portal") || cmd.equalsIgnoreCase("setup")) {
            startCaptivePortal();
        } else if (cmd.equalsIgnoreCase("reset_wifi")) {
            preferences.begin("preventa", false);
            preferences.remove("ssid");
            preferences.remove("pass");
            preferences.end();
            Serial.println("[CMD] Stored Wi-Fi cleared. Device identity and owner cache retained.");
            delay(500);
            ESP.restart();
        } else if (cmd.equalsIgnoreCase("test_relay") || cmd.equalsIgnoreCase("relay_test") || cmd.equalsIgnoreCase("test relay")) {
            runRelayTest(5);
        } else if (cmd.equalsIgnoreCase("relay_off") || cmd.equalsIgnoreCase("relay_cut") || cmd.equalsIgnoreCase("cut_power") || cmd.equalsIgnoreCase("power_off") || cmd.equalsIgnoreCase("cut") || cmd.equalsIgnoreCase("relay off") || cmd.equalsIgnoreCase("turn off") || cmd.equalsIgnoreCase("turn_off") || cmd.equalsIgnoreCase("turnoff") || cmd.equalsIgnoreCase("off")) {
            setManualRelayCut(true);
        } else if (cmd.equalsIgnoreCase("relay_on") || cmd.equalsIgnoreCase("relay_restore") || cmd.equalsIgnoreCase("restore_power") || cmd.equalsIgnoreCase("power_on") || cmd.equalsIgnoreCase("restore") || cmd.equalsIgnoreCase("relay on") || cmd.equalsIgnoreCase("turn on") || cmd.equalsIgnoreCase("turn_on") || cmd.equalsIgnoreCase("turnon") || cmd.equalsIgnoreCase("on")) {
            setManualRelayCut(false);
        } else if (cmd.equalsIgnoreCase("relay_auto") || cmd.equalsIgnoreCase("auto")) {
            setManualRelayCut(false);
        } else if (cmd.equalsIgnoreCase("relay_invert") || cmd.equalsIgnoreCase("invert_relay")) {
            Serial.println("[RELAY] Verify module polarity on the bench, then set RELAY_ACTIVE_LOW in the sketch.");
        } else if (cmd.equalsIgnoreCase("relay_status") || cmd.equalsIgnoreCase("relay_info") || cmd.equalsIgnoreCase("relay")) {
            printRelayStatus();
        } else if (cmd.length() > 0) {
            Serial.println("[CMD] Commands: test_relay | turn off | turn on | relay_auto | relay_status | capture | modem | test_sms | calibrate | baseline | status | portal | reset_wifi");
        }
    }
}

// ============================================================================
// ── NON-BLOCKING ESP32-CAM SERIAL STREAM & IMAGE RECEIVER ───────────────────
// ============================================================================
void clearSetupPreview() {
    free(g_setup_photo);
    g_setup_photo = nullptr;
    g_setup_photo_size = 0;
    free(g_setup_model_input);
    g_setup_model_input = nullptr;
    g_setup_inference = {};
}

bool requestCameraCapture(bool setupPreview) {
    if (!g_remote_settings.cameraEnabled || g_cam_query_pending || g_cam_image_streaming) return false;
    if (setupPreview) {
        clearSetupPreview();
        g_setup_capture_pending = true;
        g_setup_capture_result = SetupResult::Pending;
        // Legacy camera builds ignore this request and still respond to PREDICT.
        camSerial.println("MODEL_INPUT_ONCE");
    }
    camSerial.println("PREDICT");
    g_cam_query_pending = true;
    g_cam_query_sent_ms = g_last_cam_query_ms = millis();
    return true;
}

void finishCameraRequest() {
    if (g_setup_capture_pending && g_setup_capture_result == SetupResult::Pending) {
        if (g_setup_photo || g_setup_model_input || g_setup_inference.fire_pct >= 0) {
            ++g_setup_photo_revision;
            g_setup_capture_result = SetupResult::Accepted;
        } else {
            g_setup_capture_result = SetupResult::Failed;
        }
    }
    g_setup_capture_pending = false;
    g_cam_query_pending = false;
}

int cameraStateForScore(int fire) {
    if (fire >= 85) return 3;
    if (fire >= 60) return 2;
    if (fire >= 30) return 1;
    return 0;
}

void processCamSerial() {
    static String line = "";
    static bool reading_img = false;
    static bool readingModelInput = false;
    static size_t img_len = 0;
    static size_t img_idx = 0;
    static uint8_t* img_buf = nullptr;
    static float img_conf = 0.0f;
    static unsigned long img_last_byte_ms = 0;
    static bool autonomousFrame = false;

    const auto finishFrame = []() {
        // The CAM can send an unsolicited FIRE/IMG pair while a preview request is pending.
        if (!autonomousFrame || !g_setup_capture_pending) finishCameraRequest();
        autonomousFrame = false;
    };

    if (!camSerial.available()) {
        // Detect a genuinely stalled payload by time since the last received
        // byte, not total frame duration.
        if (reading_img && millis() - img_last_byte_ms > CAM_IMAGE_STALL_TIMEOUT_MS) {
            if (img_buf) free(img_buf);
            img_buf = nullptr;
            img_len = 0;
            img_idx = 0;
            reading_img = false;
            readingModelInput = false;
            g_cam_image_streaming = false;
            finishFrame();
            Serial.println("[CAM] Image stream made no progress for 10s; receiver reset.");
            flushDeferredReadingToBackend();
        } else if (g_cam_query_pending && millis() - g_cam_query_sent_ms > CAM_RESPONSE_TIMEOUT_MS) {
            finishCameraRequest();
            autonomousFrame = false;
            g_cam_present = false;
            Serial.println("[CAM] No complete inference/capture response after 90s; check CAM power, UART wiring, and camera status.");
            flushDeferredReadingToBackend();
        }
        return;
    }

    while (camSerial.available()) {
        char c = (char)camSerial.read();

        if (reading_img) {
            // Always consume the advertised payload, even if allocation failed.
            // Otherwise JPEG bytes would be misread as text protocol lines.
            if (img_idx < img_len && img_buf != nullptr)
                img_buf[img_idx] = (uint8_t)c;
            ++img_idx;
            img_last_byte_ms = millis();

            if (img_idx >= img_len) {
                g_cam_image_streaming = false;
                if (img_buf != nullptr) {
                    if (readingModelInput) {
                        const bool isBmp = img_len == kModelInputBmpBytes && img_buf[0] == 'B'
                            && img_buf[1] == 'M' && img_buf[18] == kModelInputSize && img_buf[22] == kModelInputSize;
                        if (isBmp && g_setup_capture_pending && g_portal_active && g_setup_inference.fire_pct >= 0) {
                            free(g_setup_model_input);
                            g_setup_model_input = img_buf;
                            img_buf = nullptr;
                        }
                    } else {
                        const bool isJpeg = img_len >= 4 && img_buf[0] == 0xFF &&
                                            img_buf[1] == 0xD8 && img_buf[img_len - 2] == 0xFF &&
                                            img_buf[img_len - 1] == 0xD9;
                        if (isJpeg && g_setup_capture_pending && !autonomousFrame) {
                            if (g_portal_active) {
                                g_setup_photo = img_buf;
                                g_setup_photo_size = img_len;
                                img_buf = nullptr;
                            }
                        } else if (isJpeg) {
                            Serial.printf("[CAM] Valid JPEG received: %u bytes (Conf: %.0f%%). Uploading...\n", (unsigned int)img_len, img_conf);
                            uploadCaptureToBackend(img_buf, img_len, img_conf);
                        } else {
                            Serial.println("[CAM] Rejected frame: invalid JPEG start/end markers.");
                        }
                    }
                    free(img_buf);
                } else {
                    Serial.printf("[CAM] Drained %u-byte frame after buffer allocation failed/size limit.\n", (unsigned int)img_len);
                }
                img_buf = nullptr;
                img_len = 0;
                img_idx = 0;
                reading_img = false;
                if (readingModelInput) {
                    readingModelInput = false;
                    g_cam_query_sent_ms = millis();
                } else {
                    img_conf = 0.0f;
                    finishFrame();
                    flushDeferredReadingToBackend();
                }
            }
            continue;
        }

        if (c == '\n') {
            line.trim();

            if (line.startsWith("PREDICT:")) {
                autonomousFrame = false;
                if (g_setup_capture_pending) g_setup_inference = {};
                int fire = 0;
                int noFire = 0;
                if (g_remote_settings.cameraEnabled && sscanf(line.c_str(), "PREDICT:%d,%d", &fire, &noFire) == 2) {
                    img_conf = (float)fire; // Stored for the upcoming IMG stream!
                    if (g_setup_capture_pending) {
                        if (fire >= 0 && fire <= 100 && noFire >= 0 && noFire <= 100) {
                            g_setup_inference.fire_pct = fire;
                            g_setup_inference.no_fire_pct = noFire;
                            g_setup_inference.response_ms = millis() - g_last_cam_query_ms;
                        }
                    } else {
                        g_cam_latest_fire_pct = fire;
                        g_cam_latest_state = cameraStateForScore(fire);
                    }
                    g_cam_present = true;
                    // Give inference and the subsequent IMG header a full response window.
                    g_cam_query_sent_ms = millis();
                    Serial.printf("[CAM] %s inference: Fire=%d%%, NoFire=%d%%\n",
                                  g_setup_capture_pending ? "Setup" : "Live", fire, noFire);
                }
            } else if (line.startsWith("PREDICT_ERROR:")) {
                finishCameraRequest();
                autonomousFrame = false;
                Serial.println("[CAM] Camera inference returned an error.");
                flushDeferredReadingToBackend();
            } else if (line.startsWith("FIRE:")) {
                autonomousFrame = true;
                img_conf = atof(line.substring(5).c_str());
            } else if (line.startsWith("IMG:") || line.startsWith("MODEL_INPUT:")) {
                const bool modelInput = line.startsWith("MODEL_INPUT:");
                const String lengthText = line.substring(modelInput ? sizeof("MODEL_INPUT:") - 1 : 4);
                char* end = nullptr;
                const unsigned long parsedLength = strtoul(lengthText.c_str(), &end, 10);
                // The camera firmware caps review JPEGs at 96 KiB. Drain up to
                // 1 MiB if a corrupt/older sender advertises more, so UART text
                // framing is recovered without attempting a huge allocation.
                if (end != lengthText.c_str() && *end == '\0' && parsedLength == 0) {
                    if (!modelInput) {
                        finishFrame();
                        g_cam_image_streaming = false;
                        Serial.println("[CAM] Camera returned a prediction but could not provide a review JPEG.");
                        flushDeferredReadingToBackend();
                    }
                } else if (end != lengthText.c_str() && *end == '\0' && parsedLength > 0 && parsedLength <= 1024UL * 1024UL) {
                    img_len = (size_t)parsedLength;
                    img_buf = nullptr;
                    readingModelInput = modelInput;
                    const bool needsBuffer = modelInput
                        ? g_setup_capture_pending && g_portal_active && !autonomousFrame && img_len == kModelInputBmpBytes
                        : img_len <= 96 * 1024;
                    if (g_remote_settings.cameraEnabled && needsBuffer) img_buf = (uint8_t*)malloc(img_len);
                    img_idx = 0;
                    img_last_byte_ms = millis();
                    reading_img = true;
                    g_cam_image_streaming = true;
                } else {
                    img_len = 0;
                    finishFrame();
                    g_cam_image_streaming = false;
                    Serial.println("[CAM] Rejected invalid camera image length.");
                    flushDeferredReadingToBackend();
                }
            } else if (line == "PONG" || line.startsWith("CAM_READY:1") || line.startsWith("TEST_CAPTURE") || line.startsWith("STANDBY_OK")) {
                g_cam_present = true;
                if (!g_setup_capture_pending || line.startsWith("CAM_READY:1")) finishCameraRequest();
                if (line.startsWith("CAM_READY:1") && !g_remote_settings.cameraEnabled) applyCameraSetting();
                flushDeferredReadingToBackend();
            }

            line = "";
        } else {
            line += c;
        }
    }
}

// ============================================================================
// ── PREFERENCES & CAPTIVE PORTAL PROVISIONING ───────────────────────────────
// ============================================================================
void loadPreferences() {
    preferences.begin("preventa", false);
    const String provisioned = preferences.getString("provisioned_id", "");
    if (provisioned != DEFAULT_DEVICE_ID) {
        preferences.remove("ssid");
        preferences.remove("pass");
        preferences.remove("remote_cfg");
        preferences.remove("dev_id");
        preferences.remove("dev_key");
        preferences.putString("provisioned_id", DEFAULT_DEVICE_ID);
        Serial.println("[Config] New provisioned identity: old Wi-Fi and owner cache cleared.");
    }
    String s_ssid = preferences.getString("ssid", DEFAULT_WIFI_SSID);
    String s_pass = preferences.getString("pass", DEFAULT_WIFI_PASSWORD);
    preferences.end();
    strlcpy(g_wifi_ssid, s_ssid.c_str(), sizeof(g_wifi_ssid));
    strlcpy(g_wifi_pass, s_pass.c_str(), sizeof(g_wifi_pass));
    strlcpy(g_backend_host, DEFAULT_BACKEND_HOST, sizeof(g_backend_host));
    strlcpy(g_device_id, DEFAULT_DEVICE_ID, sizeof(g_device_id));
    strlcpy(g_device_key, DEFAULT_DEVICE_KEY, sizeof(g_device_key));
    Serial.printf("[Config] Provisioned device: %s\n", g_device_id);
}

void savePreferences(const char* ssid, const char* pass, const char* host, const char* dev_id, const char* dev_key) {
    preferences.begin("preventa", false);
    preferences.putString("ssid", ssid);
    preferences.putString("pass", pass);
    preferences.putString("host", host);

    preferences.end();

    strncpy(g_wifi_ssid, ssid, sizeof(g_wifi_ssid) - 1);
    strncpy(g_wifi_pass, pass, sizeof(g_wifi_pass) - 1);
    strncpy(g_backend_host, host, sizeof(g_backend_host) - 1);


    rebuildUrls();
    Serial.println("[Config] Settings saved to non-volatile storage (NVS).");
}

void saveWifiCredentials(const char* ssid, const char* pass) {
    preferences.begin("preventa", false);
    preferences.putString("ssid", ssid);
    preferences.putString("pass", pass);
    preferences.end();

    strncpy(g_wifi_ssid, ssid, sizeof(g_wifi_ssid) - 1);
    strncpy(g_wifi_pass, pass, sizeof(g_wifi_pass) - 1);
    Serial.printf("[Config] Wi-Fi network provisioned: %s\n", g_wifi_ssid);
}

void rebuildUrls() {
    String httpsOrigin(CLOUD_HTTPS_ORIGIN);
    while (httpsOrigin.endsWith("/")) httpsOrigin.remove(httpsOrigin.length() - 1);
    const bool use_tls = CLOUD_CA_PEM[0] && httpsOrigin.startsWith("https://");
    if (!use_tls && (CLOUD_HTTPS_ORIGIN[0] || CLOUD_CA_PEM[0])) {
        g_backend_readings_url = "";
        g_backend_captures_url = "";
        g_backend_config_url = "";
        g_backend_provisioning_url = "";
        g_backend_activation_url = "";
        g_backend_setup_push_url = "";
        Serial.println("[Config] Cloud requests disabled. Set both the HTTPS origin and CA certificate.");
        return;
    }
    const String origin = use_tls ? httpsOrigin : "http://" + String(g_backend_host) + ":" + String(BACKEND_PORT);
    g_backend_readings_url = origin + "/api/v1/readings/";
    g_backend_captures_url = origin + "/api/v1/captures/";
    g_backend_config_url = use_tls ? origin + "/api/v1/devices/config/" : "";
    g_backend_provisioning_url = use_tls ? origin + "/api/v1/devices/provisioning/" : "";
    g_backend_activation_url = use_tls ? origin + "/api/v1/devices/activation/" : "";
    g_backend_setup_push_url = use_tls ? origin + "/api/v1/devices/setup-test-push/" : "";
}

bool beginBackendRequest(HTTPClient& http, WiFiClientSecure& client, const String& url) {
    if (!url.length()) return false;
    http.setConnectTimeout(BACKEND_HTTP_TIMEOUT_MS);
    http.setTimeout(BACKEND_HTTP_TIMEOUT_MS);
    if (url.startsWith("https://")) {
        client.setCACert(CLOUD_CA_PEM);
        client.setHandshakeTimeout(4);
        return http.begin(client, url);
    }
    return http.begin(url);
}

void applyCameraSetting() {
    if (!g_remote_settings.cameraEnabled) {
        g_cam_latest_state = 0;
        g_cam_latest_fire_pct = 0;
        g_camera_state_prev = 0;
        g_camera_warming = 0;
    }
    camSerial.println(!g_standby_active && g_remote_settings.cameraEnabled ? "STANDBY:0" : "STANDBY:1");
}

void loadRemoteSettings() {
    if (!g_backend_config_url.length()) return;
    if (!preferences.begin("preventa", true)) return;
    const String cached = preferences.getString("remote_cfg", "");
    preferences.end();
    JsonDocument doc;
    RemoteSettings settings;
    if (!deserializeJson(doc, cached)
        && strcmp(doc["_origin"] | "", CLOUD_HTTPS_ORIGIN) == 0
        && parseRemoteSettings(doc.as<JsonObjectConst>(), g_device_id, settings)) {
        storeOwnerSettings(settings);
        g_owner_active.store(strcmp(settings.activationState, "active") == 0);
        g_activation_ack_requested = strcmp(settings.activationState, "pending") == 0;
        applyCameraSetting();
    }
}

void fetchRemoteSettings(void* parameter) {
    const ConfigFetchRequest request = *static_cast<ConfigFetchRequest*>(parameter);
    delete static_cast<ConfigFetchRequest*>(parameter);
    ConfigDownload result = {};
    strlcpy(result.pairingCode, request.pairingCode, sizeof(result.pairingCode));
    {
        WiFiClientSecure client;
        HTTPClient http;
        http.useHTTP10(true);
        const String url = request.acknowledge ? g_backend_activation_url : g_backend_provisioning_url;
        if (beginBackendRequest(http, client, url)) {
            http.addHeader("X-Device-Id", g_device_id);
            http.addHeader("X-Device-Key", g_device_key);
            if (request.acknowledge || request.publishPairing) {
                JsonDocument body;
                if (request.acknowledge) {
                    body["config_version"] = request.version;
                    body["activation_generation"] = request.generation;
                } else body["pairing_code"] = request.pairingCode;
                String encoded;
                serializeJson(body, encoded);
                http.addHeader("Content-Type", "application/json");
                result.statusCode = http.POST(encoded);
                result.pairingPublished = request.publishPairing && result.statusCode == 200;
            } else result.statusCode = http.GET();
            if (result.statusCode == 200 && http.getSize() > 0 && http.getSize() <= 2048) {
                const String body = http.getString();
                if (body.length() == static_cast<size_t>(http.getSize())) body.toCharArray(result.body, sizeof(result.body));
            }
            http.end();
        }
    }
    xQueueOverwrite(g_config_downloads, &result);
    vTaskDelete(nullptr);
}

void pollRemoteSettings() {
    if (!g_backend_provisioning_url.length()) return;
    if (g_config_downloads == nullptr) g_config_downloads = xQueueCreate(1, sizeof(ConfigDownload));
    if (g_config_downloads == nullptr) return;
    ConfigDownload download;
    if (xQueueReceive(g_config_downloads, &download, 0) == pdTRUE) {
        g_config_fetch_running = false;
        if (download.pairingPublished && strcmp(download.pairingCode, g_pairing_code) == 0) {
            g_pairing_publish_pending = false;
            g_pairing_started_ms = millis();
        }
        if (download.statusCode == 401 || download.statusCode == 403) {
            // Explicit server refusal differs from an offline/network failure.
            g_owner_active.store(false);
            RemoteSettings refused;
            copyOwnerSettings(refused);
            strlcpy(refused.activationState, "unclaimed", sizeof(refused.activationState));
            refused.smsRecipient[0] = 0;
            storeOwnerSettings(refused);
            g_activation_ack_requested = false;
            if (preferences.begin("preventa", false)) { preferences.remove("remote_cfg"); preferences.end(); }
        }
        if (download.statusCode == 409) { g_activation_ack_requested = false; g_pairing_publish_pending = false; }
        JsonDocument doc;
        RemoteSettings settings;
        if (download.body[0] && !deserializeJson(doc, download.body)
                && parseRemoteSettings(doc.as<JsonObjectConst>(), g_device_id, settings)) {
            doc["_origin"] = CLOUD_HTTPS_ORIGIN;
            String cached;
            serializeJson(doc, cached);
            bool saved = false;
            if (preferences.begin("preventa", false)) {
                saved = preferences.putString("remote_cfg", cached) == cached.length();
                preferences.end();
            }
            if (saved) {
                const bool changed = strcmp(g_remote_settings.version, settings.version) != 0;
                const bool previously_active = g_owner_active.load();
                const bool camera_changed = settings.cameraEnabled != g_remote_settings.cameraEnabled;
                storeOwnerSettings(settings);
                g_owner_active.store(strcmp(settings.activationState, "active") == 0);
                g_activation_ack_requested = strcmp(settings.activationState, "pending") == 0;
                if (camera_changed) applyCameraSetting();
                if (changed || previously_active != g_owner_active.load()) {
                    g_last_uploaded_tier = -1;
                    Serial.printf("[Owner] State: %s | Location: %s\n", settings.activationState, settings.location);
                }
            }
        }
    }
    const bool connected = WiFi.status() == WL_CONNECTED;
    const bool reconnected = connected && !g_config_connected;
    if (reconnected) {
        configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER1, NTP_SERVER2);
        g_last_config_fetch_ms = millis() - 60000UL;
    }
    g_config_connected = connected;
    if (!connected || g_config_fetch_running || time(nullptr) < 1704067200) return;
    const unsigned long interval = g_owner_active.load() ? 60000UL : 5000UL;
    if (millis() - g_last_config_fetch_ms < interval) return;
    auto* request = new ConfigFetchRequest;
    if (!request) return;
    request->acknowledge = g_activation_ack_requested;
    request->publishPairing = !request->acknowledge && g_pairing_publish_pending
        && preventa::pairingWindowOpen(millis(), g_pairing_started_ms);
    strlcpy(request->pairingCode, g_pairing_code, sizeof(request->pairingCode));
    strlcpy(request->version, g_remote_settings.version, sizeof(request->version));
    strlcpy(request->generation, g_remote_settings.activationGeneration, sizeof(request->generation));
    g_last_config_fetch_ms = millis();
    g_config_fetch_running = xTaskCreate(fetchRemoteSettings, "preventa-config", 14336, request, 1, nullptr) == pdPASS;
    if (!g_config_fetch_running) delete request;
}

const char* smsRecipient() {
    return g_owner_active.load() ? g_remote_settings.smsRecipient : "";
}

const char* testSmsRecipient() {
    return smsRecipient();
}

void startCaptivePortal() {
    if (g_portal_active) return;
    if (!g_owner_active.load()) {
        snprintf(g_pairing_code, sizeof(g_pairing_code), "%06lu", (unsigned long)(esp_random() % 1000000UL));
        g_pairing_started_ms = millis();
        g_pairing_publish_pending = true;
        g_last_config_fetch_ms = millis() - 60000UL;
    }
    Serial.printf("\n[Portal] Starting protected setup AP: '%s'...\n", AP_SSID);
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAPConfig(apIP, apIP, netMsk);
    WiFi.softAP(AP_SSID, PREVENTA_SETUP_PASSWORD);

    if (WiFi.status() != WL_CONNECTED && g_wifi_ssid[0]) {
        WiFi.begin(g_wifi_ssid, g_wifi_pass);
    }
    setupWebServer();

    g_portal_active = true;
    g_portal_start_ms = millis();
    Serial.printf("[Portal] Connect to '%s' -> Open http://192.168.4.1\n", AP_SSID);
}

void stopCaptivePortal() {
    if (!g_portal_active) return;
    g_portal_active = false;
    dnsServer.stop();
    g_portal_dns_running = false;
    webServer.stop();
    clearTierTest();
    clearSetupPreview();
    g_setup_capture_result = SetupResult::Idle;
    WiFi.softAPdisconnect(true);
    Serial.println("[Portal] SoftAP stopped. Reconnecting to station Wi-Fi...");
    if (WiFi.status() != WL_CONNECTED) connectWiFi();
    updateStatusLEDs();
}

void toggleSetupPortal() {
    if (g_portal_active) {
        Serial.println("[Portal] Button long-press -> Exiting Setup Mode.");
        stopCaptivePortal();
    } else {
        Serial.println("[Portal] Button long-press -> Starting Setup Mode (SoftAP).");
        startCaptivePortal();
    }
}

void handleCaptivePortal() {
    if (WiFi.status() == WL_CONNECTED) {
        if (g_portal_dns_running) dnsServer.stop();
        g_portal_dns_running = false;
    } else {
        if (!g_portal_dns_running) g_portal_dns_running = dnsServer.start(DNS_PORT, "*", apIP);
        if (g_portal_dns_running) dnsServer.processNextRequest();
    }
    webServer.handleClient();

    // 5-minute inactivity safety timeout: auto-reconnect to Wi-Fi if abandoned
    if (millis() - g_portal_start_ms >= PORTAL_AUTO_EXIT_TIMEOUT_MS) {
        Serial.println("[Portal] 5-minute setup timeout reached without save. Restoring station Wi-Fi...");
        stopCaptivePortal();
    }
}

String escapeSetupHtml(String text) {
    text.replace("&", "&amp;");
    text.replace("<", "&lt;");
    text.replace(">", "&gt;");
    text.replace("'", "&#39;");
    text.replace("\"", "&quot;");
    return text;
}

void setupWebServer() {
    static bool registered = false;
    if (registered) {
        webServer.begin();
        return;
    }
    registered = true;
    const char* headerKeys[] = {"X-Preventa-Setup"};
    webServer.collectHeaders(headerKeys, 1);
    webServer.on("/", HTTP_GET, []() {
        int n = WiFi.scanNetworks();
        String wifiOptions = "";
        for (int i = 0; i < n; ++i) {
            const String ssid = escapeSetupHtml(WiFi.SSID(i));
            wifiOptions += "<option value='" + ssid + "'>" + ssid + " (" + String(WiFi.RSSI(i)) + " dBm)</option>";
        }

        String html = "<!DOCTYPE html><html lang='en'><head><meta charset='utf-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>PREVENTA Setup & Diagnostics</title>"
"<style>"
"body{font-family:-apple-system,BlinkMacSystemFont,Segoe UI,Roboto,sans-serif;background:#121212;color:#eee;margin:0;padding:12px}"
".wrap{max-width:480px;margin:auto}"
"h2{color:#ff5252;margin:8px 0;text-align:center;font-size:22px}"
".sub{color:#888;text-align:center;font-size:12px;margin-bottom:14px}"
".card{background:#1e1e1e;border:1px solid #333;border-radius:10px;padding:14px;margin-bottom:12px;box-shadow:0 4px 12px rgba(0,0,0,0.4)}"
"h3{color:#ff8a80;margin:0 0 10px;font-size:15px;border-bottom:1px solid #2a2a2a;padding-bottom:6px}"
"label{display:block;margin:8px 0 3px;font-size:12px;color:#aaa}"
"input,select{width:100%;box-sizing:border-box;padding:9px;border:1px solid #444;background:#2a2a2a;color:#fff;border-radius:6px;font-size:14px}"
".btn{width:100%;box-sizing:border-box;background:#ff5252;color:#fff;border:none;padding:10px;border-radius:6px;font-size:13px;font-weight:bold;margin-top:8px;cursor:pointer}"
".btn:hover{background:#ff7b7b}"
".btn-sec{background:#37474f}.btn-sec:hover{background:#455a64}"
".btn-warn{background:#f57c00}.btn-warn:hover{background:#fb8c00}"
".btn-dang{background:#d32f2f}.btn-dang:hover{background:#e53935}"
".btn-succ{background:#2e7d32}.btn-succ:hover{background:#388e3c}"
".grid2{display:grid;grid-template-columns:1fr 1fr;gap:8px}"
".grid3{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px}"
".grid5{display:grid;grid-template-columns:repeat(5,1fr);gap:4px}"
".box{background:#252525;border-radius:6px;padding:8px;text-align:center}"
".box .val{font-size:17px;font-weight:bold;color:#4caf50}"
".box .lbl{font-size:10px;color:#888;margin-top:2px}"
".status-msg{padding:8px;border-radius:6px;font-size:12px;text-align:center;margin-top:8px;display:none}"
"</style></head><body>"
"<div class='wrap'>"
"<h2>🔥 PREVENTA Setup</h2>"
"<div class='sub'>IoT Fire Detection Appliance Management</div>"
"<div class='card'><h3>Owner activation</h3><p id='owner_state'>Checking device state...</p><p>Device: <strong id='owner_device'></strong></p><p>Location: <span id='owner_location'>Not set</span></p><p>Pairing code: <strong id='owner_pin'></strong> <span id='owner_pin_time'></span></p><button type='button' class='btn btn-sec' onclick='renewPairing(this)'>New pairing code</button><p>Save home Wi-Fi first. Note the code, reconnect your phone to the internet, then open owner setup and scan the activation card.</p><a href='https://preventa.duckdns.org/setup/' target='_blank' rel='noreferrer' style='color:#ffaaaa'>Open secure owner setup</a><p>Local monitoring stays active. SMS and cloud alerts require completed activation.</p></div>"

// 1. Wi-Fi Provisioning Card (Device ID, Key, Host locked / hidden)
"<div class='card'>"
"<h3>📶 Wi-Fi Provisioning</h3>"
"<form action='/save' method='POST'>"
"<label>Select Home 2.4 GHz Network</label>"
"<select name='ssid'>" + wifiOptions + "<option value='other'>-- Other / Hidden SSID --</option></select>"
"<label>Wi-Fi Password</label>"
"<input type='password' id='pwd' name='pass' placeholder='Enter Wi-Fi password' required>"
"<div style='margin-top:6px;font-size:12px;'><label style='display:inline;'><input type='checkbox' style='width:auto;' onclick='togglePwd()'> Show password</label></div>"
"<button type='submit' class='btn'>Save & Connect</button>"
"</form>"
"<div style='font-size:11px;color:#777;margin-top:8px;text-align:center;'>Device ID & Cloud Server are pre-configured to your PREVENTA account.</div>"
"</div>"

// 2. Live Sensor Telemetry Card
"<div class='card'>"
"<h3>📊 Live Sensor Telemetry</h3>"
"<div class='grid2' style='margin-bottom:8px;'>"
"<div class='box'><div class='val' id='v_ppm'>--</div><div class='lbl'>MQ-7 (ppm)</div></div>"
"<div class='box'><div class='val' id='v_curr'>--</div><div class='lbl'>ACS712 Current (A)</div></div>"
"</div>"
"<div class='grid3'>"
"<div class='box'><div class='val' id='v_temp'>--</div><div class='lbl'>Temp (°C)</div></div>"
"<div class='box'><div class='val' id='v_hum'>--</div><div class='lbl'>Humidity (%)</div></div>"
"<div class='box'><div class='val' id='v_tier'>0</div><div class='lbl'>Active Tier</div></div>"
"</div>"
"<div style='font-size:11px;color:#aaa;margin-top:8px;text-align:center;' id='v_cam'>Camera Link: Checking...</div>"
"<div id='v_hush' style='font-size:12px;margin-top:8px' aria-live='polite'></div>"
"<p>Short press: standby below Tier 3; at Tier 3 or higher, hush the buzzer for 2 minutes. A higher tier cancels the hush. Hold 3 seconds for setup.</p>"
"<p>Start the same 30-sample baseline calibration as the Serial calibrate command. Keep the area clear of smoke and let the sensors settle.</p>"
"<button type='button' class='btn btn-sec' onclick='calibrateBaseline(this)'>Calibrate Sensor Baselines</button>"
"<div id='baseline_stat' style='font-size:12px;margin-top:6px' aria-live='polite'></div>"
"<p>For zero calibration, use your normal power source and unplug every load from the monitored outlet. Keep the mains enclosure closed.</p>"
"<button type='button' class='btn btn-sec' onclick='calibrateCurrent(this)'>Calibrate ACS712 Zero</button>"
"<div id='acs_stat' class='status-msg' aria-live='polite'></div>"
"<button type='button' id='capture_button' class='btn btn-sec' onclick='captureImage(this)'>Capture Image</button>"
"<div id='capture_stat' style='font-size:12px;margin-top:6px' aria-live='polite'></div>"
"<p>Model input: the central 96 × 96 pixels used for this prediction, enlarged for inspection.</p>"
"<figure hidden style='margin:8px 0 0'><img id='setup_model_input' alt='Exact 96 by 96 model input used for this prediction' width='96' height='96' style='display:block;width:100%;height:auto;max-width:384px;image-rendering:pixelated'><figcaption>Model input (96 × 96) — same capture as the scores below</figcaption></figure>"
"<div id='capture_inference' hidden style='font-size:13px;line-height:1.6;margin-top:6px;white-space:pre-line' aria-live='polite'></div>"
"</div>"

// 3. Tier Tester Simulation Card (Tiers 1-5)
"<div class='card'>"
"<h3>🧪 Alert Tier Tester</h3>"
"<p>Tiers 3 to 5 request a camera preview and send messages marked TEST. Tier 4 starts the siren. Tier 5 starts the siren and cuts the relay. Use an unloaded outlet for relay tests.</p>"
"<label><input type='checkbox' id='tier4_cut' style='width:auto'> Test Tier 4 electrical relay cutoff</label>"
"<div class='grid5'>"
"<button type='button' class='btn btn-sec' onclick='testTier(1)'>Tier 1</button>"
"<button type='button' class='btn btn-sec' onclick='testTier(2)'>Tier 2</button>"
"<button type='button' class='btn btn-warn' onclick='testTier(3)'>Tier 3</button>"
"<button type='button' class='btn btn-dang' onclick='testTier(4)'>Tier 4</button>"
"<button type='button' class='btn btn-dang' onclick='testTier(5)'>Tier 5</button>"
"</div>"
"<button type='button' class='btn btn-succ' style='margin-top:6px;' onclick='testTier(0)'>✓ Clear Test (Reset to Normal)</button>"
"<div id='tier_msg' class='status-msg'></div>"
"<div id='tier_delivery' style='font-size:12px;margin-top:6px' aria-live='polite'></div>"
"</div>"

// 4. Actuator & Mains Relay Test Card
"<div class='card'>"
"<h3>⚡ Appliance Relay Cutoff Test</h3>"
"<div class='grid2'>"
"<button type='button' class='btn btn-warn' onclick='relayAct(\"test\")'>Timed Cut (5s)</button>"
"<button type='button' class='btn btn-dang' onclick='relayAct(\"cut\")'>Emergency Cut</button>"
"</div>"
"<button type='button' class='btn btn-succ' onclick='relayAct(\"restore\")'>Restore (Clear Manual Cut)</button>"
"<div id='relay_msg' class='status-msg'></div>"
"</div>"

// 5. Cellular Modem & SMS Test Card
"<div class='card'>"
"<h3>📱 Cellular SMS Test</h3>"
"<label>Recipient Phone Number</label>"
"<input type='tel' id='sms_num' placeholder='Leave blank to use the configured or default number'>"
"<label>Message Body</label>"
"<input type='text' id='sms_msg' maxlength='160' placeholder='PREVENTA test message. No emergency action is needed.'>"
"<button type='button' class='btn btn-sec' onclick='sendSms(this)'>Send Test SMS</button>"
"<div id='sms_stat' class='status-msg' aria-live='polite'></div>"
"</div>"

// 6. Exit Setup Mode
"<div class='card'>"
"<button type='button' class='btn btn-sec' onclick='exitSetup()'>🚪 Exit Setup Mode & Reconnect</button>"
"</div>"

"</div>" // wrap

"<script>"
"function togglePwd(){var x=document.getElementById('pwd');x.type=(x.type==='password')?'text':'password';}"
"function showMsg(id,txt,col){var e=document.getElementById(id);if(!e)return;e.style.display='block';e.style.background=col;e.innerText=txt;setTimeout(function(){e.style.display='none';},4000);}"
"const setupResult={idle:'Ready',pending:'Working...',accepted:'Accepted by provider',failed:'Failed',offline:'Needs ESP32 internet access',no_browser:'No registered browser',rate_limited:'Try again in a minute'};"
"let captureRequesting=false,captureEpoch=0,captureRevisionFloor=null,lastPhotoRevision=null,statusRequest=0,statusApplied=0;"
"function showCapturePreview(id,visible,path,revision){"
"const image=document.getElementById(id),figure=image.parentElement;"
"image.dataset.visible=visible?'1':'0';"
"if(!visible){figure.hidden=true;image.removeAttribute('src');delete image.dataset.revision;return;}"
"if(image.dataset.revision===String(revision)){figure.hidden=!image.complete||image.naturalWidth===0;return;}"
"figure.hidden=true;image.dataset.revision=String(revision);"
"image.onload=()=>{if(image.dataset.visible==='1')figure.hidden=false;};"
"image.onerror=()=>{figure.hidden=true;delete image.dataset.revision;};"
"image.src=path+'?v='+revision;"
"}"
"function updateTelemetry(){"
"const epoch=captureEpoch,request=++statusRequest;"
"fetch('/api/status',{cache:'no-store'}).then(r=>r.json()).then(d=>{"
"if(epoch!==captureEpoch||request<statusApplied)return;statusApplied=request;lastPhotoRevision=d.photo_revision;"
"document.getElementById('v_ppm').innerText=d.mq7_ppm.toFixed(0);"
"document.getElementById('v_curr').innerText=d.current_a.toFixed(2);"
"document.getElementById('v_temp').innerText=d.temp_c.toFixed(1);"
"document.getElementById('v_hum').innerText=d.humidity.toFixed(0);"
"document.getElementById('v_tier').innerText='T' + d.tier;"
"document.getElementById('owner_state').textContent=d.activation_state==='active'?'Activated. Local and remote features enabled.':d.activation_state==='pending'?'Linked to an owner. Waiting for synchronization.':'Local monitoring active. Owner setup incomplete.';"
"document.getElementById('owner_device').textContent=d.device_id||'';"
"document.getElementById('owner_location').textContent=d.location||'Not set';"
"document.getElementById('owner_pin').textContent=d.pairing_code||'Not available';"
"document.getElementById('owner_pin_time').textContent=d.pairing_remaining_sec?'('+d.pairing_remaining_sec+'s remaining)':'';"
"document.getElementById('v_hush').innerText=d.buzzer_hushed?'Buzzer hushed: '+d.hush_remaining_sec+'s remaining. Monitoring and protection active.':(d.buzzer_active?'Buzzer sounding.':'Buzzer idle.');"
"document.getElementById('v_cam').innerText='Camera Link: ' + (d.cam_link?'ONLINE':'DISCONNECTED');"
"document.getElementById('baseline_stat').innerText=d.baseline_samples < d.baseline_target?'Collecting baseline: '+d.baseline_samples+' / '+d.baseline_target:'Sensor baselines ready.';"
"const pending=captureRequesting||d.capture_result==='pending';"
"const complete=!pending&&d.capture_result==='accepted'&&(captureRevisionFloor===null||d.photo_revision!==captureRevisionFloor);"
"document.getElementById('capture_button').disabled=pending;"
"showCapturePreview('setup_model_input',complete&&d.model_input_ready,'/api/model-input.bmp',d.photo_revision);"
"var details=document.getElementById('capture_inference');details.hidden=!complete;"
"if(!details.hidden){"
"const p=d.preview_inference;"
"if(p){"
"const winner=p.fire_pct===p.no_fire_pct?'Tie':p.fire_pct>p.no_fire_pct?'Fire':'No fire';"
"details.innerText='Fire '+p.fire_pct+'%  |  No fire '+p.no_fire_pct+'%\\nTop score: '+winner+'  |  Camera evidence: '+p.camera_state+'\\nCapture request to prediction: '+p.response_ms+' ms';"
"}else details.innerText='The camera returned no valid prediction scores for this capture.';"
"}"
"const captureStatus='Capture #'+d.photo_revision+': '+(d.model_input_ready?'96 x 96 model input ready.':'model input preview unavailable. Update the ESP32-CAM firmware or retry.');"
"document.getElementById('capture_stat').innerText=pending?'Capturing a new image...':complete?captureStatus:d.capture_result==='failed'?'Camera capture failed. Check the camera link and retry.':'';"
"document.getElementById('tier_delivery').innerText=d.test_sms==='idle'&&d.test_push==='idle'?'':'Test SMS: '+setupResult[d.test_sms]+' | Test push: '+setupResult[d.test_push];"
"}).catch(e=>{});}"
"setInterval(updateTelemetry,2000);updateTelemetry();"
"function renewPairing(button){button.disabled=true;fetch('/api/pairing',{method:'POST',headers:{'X-Preventa-Setup':'1'}}).then(r=>r.json()).then(d=>{showMsg('tier_msg',d.message,d.status==='ok'?'#37474f':'#d32f2f');updateTelemetry();}).catch(()=>{}).finally(()=>{button.disabled=false;});}"
"function calibrateBaseline(button){"
"if(!confirm('Start baseline calibration in clean air with the monitored outlet unloaded?'))return;"
"button.disabled=true;fetch('/api/cmd?c=calibrate',{method:'POST'}).then(r=>r.json()).then(d=>{"
"showMsg('acs_stat',d.message,d.status==='ok'?'#2e7d32':'#d32f2f');updateTelemetry();"
"}).catch(()=>showMsg('acs_stat','Could not start baseline calibration.','#d32f2f')).finally(()=>{button.disabled=false;});}"
"function captureImage(button){"
"if(button.disabled||captureRequesting)return;captureRequesting=true;button.disabled=true;++captureEpoch;captureRevisionFloor=lastPhotoRevision;"
"showCapturePreview('setup_model_input',false,'/api/model-input.bmp',0);"
"document.getElementById('capture_inference').hidden=true;document.getElementById('capture_stat').innerText='Capturing a new image...';"
"fetch('/api/capture',{method:'POST',cache:'no-store'}).then(r=>r.json()).then(d=>{"
"captureRequesting=false;++captureEpoch;"
"if(d.status==='ok'){if(Number.isInteger(d.previous_photo_revision))captureRevisionFloor=d.previous_photo_revision;}else{button.disabled=false;alert(d.message);}updateTelemetry();"
"}).catch(()=>{captureRequesting=false;++captureEpoch;button.disabled=false;document.getElementById('capture_stat').innerText='Could not request a capture. Please retry.';});}"
"function calibrateCurrent(button){"
"if(!confirm('Are all appliances unplugged from the monitored outlet?'))return;"
"button.disabled=true;"
"fetch('/api/acs712/zero',{method:'POST',body:new URLSearchParams({no_load:'1'})}).then(r=>r.json()).then(d=>{"
"showMsg('acs_stat',d.message,d.status==='ok'?'#2e7d32':'#d32f2f');if(d.status==='ok')updateTelemetry();"
"}).catch(()=>showMsg('acs_stat','Could not confirm calibration.','#d32f2f')).finally(()=>{button.disabled=false;});}"
"function testTier(t){"
"const electrical=t===4&&document.getElementById('tier4_cut').checked;"
"if(t>=3&&!confirm('Send Tier '+t+' TEST messages?'+((electrical||t===5)?' This cuts the relay. Confirm the monitored outlet is unloaded.':'')))return;"
"fetch('/api/test_tier?tier='+t+'&electrical='+(electrical?'1':'0'),{method:'POST',headers:{'X-Preventa-Setup':'1'}}).then(r=>r.json()).then(d=>{"
"showMsg('tier_msg',d.message,d.status==='ok'?'#37474f':'#d32f2f');updateTelemetry();"
"}).catch(()=>showMsg('tier_msg','Could not apply the tier test.','#d32f2f'));}"
"function relayAct(a){"
"fetch('/api/relay?action='+a,{method:'POST'}).then(r=>{if(!r.ok)throw Error();return r.json();}).then(d=>{"
"var msg=d.relay_cut?'Relay command: CUT':'Relay command: RELEASE';"
"if(d.automatic_cut)msg+=' (automatic cutoff active)';"
"showMsg('relay_msg',msg,d.relay_cut?'#d32f2f':'#2e7d32');"
"}).catch(()=>showMsg('relay_msg','Could not confirm relay command.','#d32f2f'));}"
"function sendSms(button){"
"var number=document.getElementById('sms_num').value;var msg=document.getElementById('sms_msg').value;"
"button.disabled=true;showMsg('sms_stat','Sending test SMS...','#37474f');"
"fetch('/api/sms',{method:'POST',body:new URLSearchParams({number,msg})}).then(r=>r.json()).then(d=>{"
"showMsg('sms_stat',d.message,d.status==='sent'?'#2e7d32':'#d32f2f');"
"}).catch(()=>showMsg('sms_stat','Could not confirm SMS submission.','#d32f2f')).finally(()=>{button.disabled=false;});}"
"function exitSetup(){"
"if(confirm('Exit Setup Mode and reconnect to your Wi-Fi?')){"
"fetch('/api/cmd?c=exit',{method:'POST'}).then(()=>{alert('Setup Mode closed. Device is reconnecting to home Wi-Fi.');});"
"}}"
"</script></body></html>";

        webServer.sendHeader("Cache-Control", "no-store");
        webServer.send(200, "text/html; charset=utf-8", html);
    });

    webServer.on("/save", HTTP_POST, []() {
        String req_ssid = webServer.arg("ssid");
        String req_pass = webServer.arg("pass");
        saveWifiCredentials(req_ssid.c_str(), req_pass.c_str());

        String res = "<!DOCTYPE html><html lang='en'><head><meta charset='utf-8'></head>"
                     "<body style='background:#121212;color:#eee;font-family:sans-serif;text-align:center;padding:40px;'>"
                     "<h2 style='color:#4caf50;'>Credentials Saved!</h2>"
                     "<p>PREVENTA is connecting to Wi-Fi: <b>" + escapeSetupHtml(req_ssid) + "</b>.</p>"
                     "<p>Setup Mode will now close. Reconnect your phone to your home network.</p>"
                     "</body></html>";
        webServer.sendHeader("Cache-Control", "no-store");
        webServer.send(200, "text/html; charset=utf-8", res);
        delay(1000);
        WiFi.disconnect();
        stopCaptivePortal();
    });

    webServer.on("/api/pairing", HTTP_POST, []() {
        if (webServer.header("X-Preventa-Setup") != "1" || g_owner_active.load()
                || strcmp(g_remote_settings.activationState, "pending") == 0) {
            webServer.send(409, "application/json", "{\"status\":\"error\",\"message\":\"Pairing is only available for an unclaimed device in setup mode.\"}");
            return;
        }
        snprintf(g_pairing_code, sizeof(g_pairing_code), "%06lu", (unsigned long)(esp_random() % 1000000UL));
        g_pairing_started_ms = millis();
        g_pairing_publish_pending = true;
        g_last_config_fetch_ms = millis() - 60000UL;
        webServer.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"New pairing code generated. It will be published when home Wi-Fi connects.\"}");
    });

    webServer.on("/api/status", HTTP_GET, []() {
        auto quote = [](const char* value) { JsonDocument quoted; quoted.set(value); String result; serializeJson(quoted, result); return result; };
        String json = "{";
        json += "\"device_id\":" + quote(g_device_id) + ",";
        json += "\"activation_state\":" + quote(g_remote_settings.activationState) + ",";
        json += "\"location\":" + quote(g_remote_settings.location) + ",";
        const bool pairing_open = !g_owner_active.load() && g_pairing_code[0]
            && preventa::pairingWindowOpen(millis(), g_pairing_started_ms);
        json += "\"pairing_code\":" + quote(pairing_open ? g_pairing_code : "") + ",";
        json += "\"pairing_remaining_sec\":" + String(pairing_open ? (preventa::pairingDurationMs - (millis() - g_pairing_started_ms)) / 1000UL : 0) + ",";
        json += "\"mq7_ppm\":" + String(g_ui_mq7_ppm, 1) + ",";
        json += "\"temp_c\":" + String(g_ui_temp, 1) + ",";
        json += "\"humidity\":" + String(g_ui_hum, 1) + ",";
        json += "\"current_a\":" + String(g_ui_current, 3) + ",";
        json += "\"cam_link\":" + String(g_cam_present ? "true" : "false") + ",";
        json += "\"cam_fire_pct\":" + String(g_cam_latest_fire_pct) + ",";
        json += "\"baseline_samples\":" + String(min(g_baseline_sample_count, BASELINE_SAMPLES)) + ",";
        json += "\"baseline_target\":" + String(BASELINE_SAMPLES) + ",";
        json += "\"photo_revision\":" + String(g_setup_photo_revision) + ",";
        json += "\"capture_result\":\"" + String(setupResultName(g_setup_capture_result)) + "\",";
        json += "\"photo_ready\":" + String(g_setup_photo ? "true" : "false") + ",";
        json += "\"model_input_ready\":" + String(g_setup_model_input ? "true" : "false") + ",";
        json += "\"preview_inference\":";
        if (g_setup_capture_result == SetupResult::Accepted && g_setup_inference.fire_pct >= 0) {
            json += "{\"fire_pct\":" + String(g_setup_inference.fire_pct)
                    + ",\"no_fire_pct\":" + String(g_setup_inference.no_fire_pct)
                    + ",\"camera_state\":\"" + STATE_LABELS[cameraStateForScore(g_setup_inference.fire_pct)]
                    + "\",\"response_ms\":" + String(g_setup_inference.response_ms) + "},";
        } else {
            json += "null,";
        }
        json += "\"test_sms\":\"" + String(setupResultName(g_setup_sms_result.load())) + "\",";
        json += "\"test_push\":\"" + String(setupResultName(g_setup_push_result.load())) + "\",";
        json += "\"tier\":" + String(g_tier_test_override >= 0 ? g_tier_test_override : g_ui_tier) + ",";
        const uint32_t hush_left = g_alarm_hush.remainingMs(effectiveAlertTier(), millis());
        json += "\"buzzer_hushed\":" + String(hush_left > 0 ? "true" : "false") + ",";
        json += "\"hush_remaining_sec\":" + String((hush_left + 999UL) / 1000UL) + ",";
        json += "\"buzzer_active\":" + String(g_buzzer_active ? "true" : "false") + ",";
        json += "\"relay_cut\":" + String(relayCutRequested() ? "true" : "false") + ",";
        json += "\"wifi_connected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false");
        json += "}";
        webServer.sendHeader("Cache-Control", "no-store");
        webServer.send(200, "application/json", json);
    });

    webServer.on("/api/test_tier", HTTP_POST, []() {
        const String value = webServer.arg("tier");
        if (value.length() != 1 || value[0] < '0' || value[0] > '5') {
            webServer.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Select a tier from 0 to 5.\"}");
            return;
        }
        const int tier = value[0] - '0';
        if (tier >= SMS_ALERT_TIER && webServer.header("X-Preventa-Setup") != "1") {
            webServer.send(403, "application/json", "{\"status\":\"error\",\"message\":\"Start the alert test from the setup page.\"}");
            return;
        }
        if (tier == 0) {
            clearTierTest();
            webServer.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Tier test cleared.\"}");
            return;
        }
        if (g_ui_tier >= 3 || (tier >= SMS_ALERT_TIER && g_setup_alert_running.load())) {
            webServer.send(409, "application/json", "{\"status\":\"error\",\"message\":\"A live warning or a previous test message is still active.\"}");
            return;
        }
        if (tier >= SMS_ALERT_TIER) {
            g_setup_alert_running.store(true);
            g_setup_sms_result.store(SetupResult::Pending);
            g_setup_push_result.store(SetupResult::Pending);
            if (xTaskCreate(sendSetupTestAlerts, "preventa-setup-test", 14336,
                            reinterpret_cast<void*>(static_cast<uintptr_t>(tier)), 1, nullptr) != pdPASS) {
                g_setup_sms_result.store(SetupResult::Failed);
                g_setup_push_result.store(SetupResult::Failed);
                g_setup_alert_running.store(false);
            }
        }
        g_tier_test_override = tier;
        g_tier_test_relay_cut = tier == 5 || (tier == 4 && webServer.arg("electrical") == "1");
        g_tier_test_start_ms = millis();
        updateLEDs(tier);
        updateBuzzer(tier >= 4);
        updateRelay();
        const bool capture = tier >= 3 && requestCameraCapture(true);
        String message = "Simulating Tier " + String(tier) + " for 30 seconds.";
        if (tier >= 3 && !capture) message += " Camera is disabled or busy.";
        webServer.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"" + message + "\"}");
    });

    webServer.on("/api/capture", HTTP_POST, []() {
        if (!requestCameraCapture(true)) {
            webServer.send(409, "application/json", "{\"status\":\"error\",\"message\":\"Camera is disabled or busy.\"}");
            return;
        }
        webServer.sendHeader("Cache-Control", "no-store");
        webServer.send(202, "application/json", "{\"status\":\"ok\",\"previous_photo_revision\":"
                       + String(g_setup_photo_revision) + "}");
    });

    webServer.on("/api/capture.jpg", HTTP_GET, []() {
        if (!g_setup_photo || g_setup_capture_result != SetupResult::Accepted
            || (webServer.hasArg("v") && webServer.arg("v") != String(g_setup_photo_revision))) {
            webServer.send(404, "text/plain", "Capture unavailable.");
            return;
        }
        webServer.sendHeader("Cache-Control", "no-store");
        webServer.send_P(200, "image/jpeg", reinterpret_cast<const char*>(g_setup_photo), g_setup_photo_size);
    });

    webServer.on("/api/model-input.bmp", HTTP_GET, []() {
        if (!g_setup_model_input || g_setup_capture_result != SetupResult::Accepted
            || (webServer.hasArg("v") && webServer.arg("v") != String(g_setup_photo_revision))) {
            webServer.send(404, "text/plain", "Model input preview unavailable.");
            return;
        }
        webServer.sendHeader("Cache-Control", "no-store");
        webServer.send_P(200, "image/bmp", reinterpret_cast<const char*>(g_setup_model_input), kModelInputBmpBytes);
    });

    webServer.on("/api/relay", HTTP_POST, []() {
        String act = webServer.arg("action");
        if (act == "test") {
            runRelayTest(5);
        } else if (act == "cut") {
            setManualRelayCut(true);
        } else if (act == "restore") {
            setManualRelayCut(false);
        } else {
            webServer.send(400, "application/json", "{\"status\":\"invalid_action\"}");
            return;
        }
        String json = "{\"status\":\"ok\",\"relay_cut\":";
        json += relayCutRequested() ? "true" : "false";
        json += ",\"automatic_cut\":";
        json += g_auto_relay_cut ? "true" : "false";
        json += "}";
        webServer.send(200, "application/json", json);
    });

    webServer.on("/api/acs712/zero", HTTP_POST, []() {
        if (webServer.arg("no_load") != "1") {
            webServer.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Confirm the monitored outlet has no load.\"}");
            return;
        }
        switch (calibrateAcsZero()) {
            case AcsZeroResult::Ok:
                webServer.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Zero-current offset saved.\"}");
                break;
            case AcsZeroResult::OutOfRange:
                webServer.send(409, "application/json", "{\"status\":\"error\",\"message\":\"The sensor voltage is out of the expected range. Check the ACS712 supply and signal.\"}");
                break;
            case AcsZeroResult::Unstable:
                webServer.send(409, "application/json", "{\"status\":\"error\",\"message\":\"The sensor reading is unstable. Unplug all monitored loads and try again.\"}");
                break;
            case AcsZeroResult::SaveFailed:
                webServer.send(500, "application/json", "{\"status\":\"error\",\"message\":\"The offset could not be saved. Try again.\"}");
                break;
        }
    });

    webServer.on("/api/sms", HTTP_POST, []() {
        String num = webServer.arg("number");
        num.trim();
        if (num.isEmpty()) {
            num = testSmsRecipient();
        } else {
            for (const char* separator : {" ", "(", ")", ".", "-"}) num.replace(separator, "");
            if (num.length() == 11 && num.startsWith("09")) num = "+63" + num.substring(1);
            if (num.length() == 12 && num.startsWith("639")) num = "+" + num;
        }
        if (!validSmsNumber(num.c_str())) {
            webServer.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Enter a valid phone number or leave the field blank.\"}");
            return;
        }
        String msg = webServer.arg("msg");
        msg.trim();
        if (msg.isEmpty()) msg = SETUP_TEST_SMS;
        if (msg.length() > 160 || msg.indexOf(char(26)) >= 0 || msg.indexOf(char(27)) >= 0) {
            webServer.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Use a short plain-text test message.\"}");
            return;
        }
        if (trySendSMS(num.c_str(), msg.c_str())) {
            webServer.send(200, "application/json", "{\"status\":\"sent\",\"message\":\"SMS accepted by the modem. Check the receiving phone.\"}");
        } else {
            webServer.send(503, "application/json", "{\"status\":\"error\",\"message\":\"SMS submission failed or the modem is busy. Check the modem, SIM and signal.\"}");
        }
    });

    webServer.on("/api/cmd", HTTP_POST, []() {
        String c = webServer.arg("c");
        if (c == "exit") {
            webServer.send(200, "application/json", "{\"status\":\"exiting\"}");
            delay(500);
            stopCaptivePortal();
            return;
        } else if (c == "calibrate") {
            if (g_ui_tier >= 3 || g_tier_test_override > 0) {
                webServer.send(409, "application/json", "{\"status\":\"error\",\"message\":\"Finish the active warning or tier test before calibration.\"}");
                return;
            }
            runCalibration();
            webServer.send(200, "application/json", "{\"status\":\"ok\",\"message\":\"Baseline calibration started. Keep the area clear for 30 samples.\"}");
            return;
        }
        webServer.send(200, "application/json", "{\"status\":\"ok\"}");
    });

    webServer.onNotFound([]() {
        webServer.sendHeader("Location", "http://192.168.4.1/", true);
        webServer.send(302, "text/plain", "");
    });

    webServer.begin();
}

// ============================================================================
// ── CONNECTIVITY: WIFI & NTP ────────────────────────────────────────────────
// ============================================================================
void connectWiFi() {
    Serial.printf("[WiFi] Connecting to %s...\n", g_wifi_ssid);
    WiFi.mode(WIFI_STA);
    WiFi.begin(g_wifi_ssid, g_wifi_pass);

    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 12000) {
        delay(250);
        Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("[WiFi] Connected successfully. Device IP: ");
        Serial.println(WiFi.localIP());
        g_backend_consecutive_fails = 0;
        g_backend_backoff_until_ms = 0;
    } else {
        Serial.println("[WiFi] Connection timeout. Local autonomous detection active.");
    }
}

void maintainWiFi() {
    static unsigned long lastAttempt = 0;
    if (WiFi.status() != WL_CONNECTED) {
        unsigned long now = millis();
        if (now - lastAttempt > 15000) {
            lastAttempt = now;
            Serial.println("[WiFi] Reconnecting...");
            WiFi.disconnect();
            WiFi.begin(g_wifi_ssid, g_wifi_pass);
        }
    }
}

void syncTimeNTP() {
    if (WiFi.status() != WL_CONNECTED) return;
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER1, NTP_SERVER2);
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 4000)) {
        Serial.printf("[NTP] Time synchronized (PST UTC+8): %s\n", getTimestampString().c_str());
    }
}

String getTimestampString() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo, 50)) {
        return String("time-not-synced");
    }
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
    return String(buf);
}

// ============================================================================
// ── SELF-DIAGNOSTIC & STATUS INDICATORS ─────────────────────────────────────
// ============================================================================
bool checkCameraLink() {
    // A diagnostic ping must not flush the binary image currently being sent.
    if (g_cam_query_pending) return g_cam_present;

    while (camSerial.available()) camSerial.read();

    camSerial.println("PING");
    unsigned long start = millis();
    String resp = "";
    while (millis() - start < 400) {
        while (camSerial.available()) {
            char c = (char)camSerial.read();
            if (c == '\n') {
                resp.trim();
                if (resp == "PONG" || resp.startsWith("CAM_READY:1")) {
                    return true;
                }
                resp = "";
            } else {
                resp += c;
            }
        }
    }
    return false;
}

bool selfDiagnose() {
    bool ok = true;

    float mq7_v = analogReadMilliVolts(MQ7_ADC_PIN) / 1000.0f;
    float acs_v = analogReadMilliVolts(ACS712_ADC_PIN) / 1000.0f;

    g_mq7_present    = (mq7_v > 0.05f && mq7_v < 3.1f);
    g_acs712_present = (acs_v > 0.05f && acs_v < 3.1f);

    float t = dht.readTemperature();
    g_dht_present = !isnan(t);

    g_cam_present = checkCameraLink();

    if (!g_mq7_present)    { ok = false; Serial.println("[SELF-TEST] ⚠ MQ-7 fault: Voltage out of range."); }
    if (!g_acs712_present) { ok = false; Serial.println("[SELF-TEST] ⚠ ACS712 fault: Voltage out of range."); }
    if (!g_dht_present)    { ok = false; Serial.println("[SELF-TEST] ⚠ DHT fault: Read returned NaN."); }
    if (!g_cam_present)    { ok = false; Serial.println("[SELF-TEST] ⚠ ESP32-CAM fault: UART handshake failed."); }

    g_system_ok = ok;
    if (ok) {
        Serial.println("[SELF-TEST] ✓ All sensors & ESP32-CAM co-processor verified.");
    } else {
        Serial.println("[SELF-TEST] ⚠ System self-test flagged one or more faults.");
    }
    return ok;
}

void updateStatusLEDs() {
    // 1. Standby Mode: all LEDs extinguished
    if (g_standby_active) {
        digitalWrite(POWER_LED_PIN, LOW);
        digitalWrite(SYSTEM_LED_PIN, LOW);
        digitalWrite(NETWORK_LED_PIN, LOW);
        return;
    }

    digitalWrite(POWER_LED_PIN, HIGH);

    // 2. Setup Mode: 1-2-3 triple-pulse cadence on System LED
    if (g_portal_active) {
        unsigned long cycle = millis() % 1200;
        bool pulse = (cycle < 150) || (cycle >= 300 && cycle < 450) || (cycle >= 600 && cycle < 750);
        digitalWrite(SYSTEM_LED_PIN, pulse ? HIGH : LOW);

        unsigned long now = millis();
        if (now - g_last_status_blink_ms >= 400) {
            g_last_status_blink_ms = now;
            digitalWrite(NETWORK_LED_PIN, !digitalRead(NETWORK_LED_PIN));
        }
        return;
    }

    // 3. Boot / Warmup Mode: Fast pulsing blink on System LED
    if (g_warmup_active) {
        unsigned long cycle = millis() % 300;
        digitalWrite(SYSTEM_LED_PIN, (cycle < 150) ? HIGH : LOW);
        if (WiFi.status() == WL_CONNECTED) {
            digitalWrite(NETWORK_LED_PIN, HIGH);
        } else {
            digitalWrite(NETWORK_LED_PIN, LOW);
        }
        return;
    }

    // 4. Normal System Diagnostic LED
    if (g_system_ok) {
        digitalWrite(SYSTEM_LED_PIN, HIGH);
    } else {
        unsigned long now = millis();
        if (now - g_last_status_blink_ms >= 300) {
            g_last_status_blink_ms = now;
            g_status_blink_state = !g_status_blink_state;
            digitalWrite(SYSTEM_LED_PIN, g_status_blink_state ? HIGH : LOW);
        }
    }

    // Network Status LED
    if (WiFi.status() == WL_CONNECTED) {
        digitalWrite(NETWORK_LED_PIN, HIGH);
    } else {
        unsigned long now = millis();
        if (now - g_last_status_blink_ms >= 500) {
            digitalWrite(NETWORK_LED_PIN, !digitalRead(NETWORK_LED_PIN));
        }
    }
}

// ============================================================================
// ── SENSOR ACQUISITION & EXECUTION ──────────────────────────────────────────
// ============================================================================
void readSensorsAndRunPipeline() {
    float temp_c  = dht.readTemperature();
    float hum_pct = dht.readHumidity();

    const bool temperature_valid = isfinite(temp_c) && temp_c >= -10.0f && temp_c <= 100.0f;
    const bool humidity_valid = isfinite(hum_pct) && hum_pct >= 0.0f && hum_pct <= 100.0f;
    if (temperature_valid) {
        g_last_good_temp = temp_c;
    } else {
        temp_c = g_last_good_temp;
    }
    if (humidity_valid) {
        g_last_good_hum  = hum_pct;
    } else {
        hum_pct = g_last_good_hum;
    }

    float mq7_adc_volts = analogReadMilliVolts(MQ7_ADC_PIN) / 1000.0f;
    float raw_mq7_v     = mq7_adc_volts * MQ7_DIVIDER_COMPENSATION;

    float acs_adc_volts = analogReadMilliVolts(ACS712_ADC_PIN) / 1000.0f;
    float raw_acs_v     = acs_adc_volts * ACS712_DIVIDER_COMPENSATION;

    g_mq7_present = (mq7_adc_volts > 0.05f && mq7_adc_volts < 3.1f);
    if (acs_adc_volts <= 0.01f || acs_adc_volts >= 3.2f) g_acs712_present = false;
    g_dht_present = temperature_valid && humidity_valid;

    g_system_ok = g_mq7_present && g_dht_present && g_acs712_present && g_cam_present;

    runFullPipeline(raw_mq7_v, temp_c, hum_pct, raw_acs_v, temperature_valid);
}

// ============================================================================
// ── FULL 10-STEP FIRE DETECTION PIPELINE ────────────────────────────────────
// ============================================================================
void runFullPipeline(float raw_mq7_v, float raw_temp, float raw_hum, float raw_acs_v,
                     bool temperature_valid) {
    SmoothedData smoothed = stepSmoothing(raw_mq7_v, raw_acs_v, raw_temp, raw_hum);

    // Cache latest smoothed sensor values for setup UI telemetry
    g_ui_mq7_ppm = smoothed.mq7_ppm;
    g_ui_temp    = smoothed.temperature;
    g_ui_hum     = smoothed.humidity;
    g_ui_current = smoothed.acs_current;

    stepBaselineNormalization(smoothed);
    DerivedData derived = stepDerivedFeatures(smoothed, temperature_valid);
    SensorStates states = stepSensorClassification(smoothed, derived);
    stepPersistenceUpdate(states);

    int tier_floor   = 0;
    int initial_tier = stepInitialTierEstimate(states, tier_floor);
    if (!g_warmup_active && smoothed.humidity >= g_remote_settings.humidityAdvisory) {
        initial_tier = max(initial_tier, 1);
    }

    FusionData fusion = stepFusionLogic(states);

    CameraDecision cam_dec = stepCameraWakeDecision(states, fusion, initial_tier);
    states.camera = cam_dec.camera_state;
    g_camera_state_prev = cam_dec.camera_state;

    int final_tier = stepFinalTierDecision(initial_tier, tier_floor, fusion, states);
    g_ui_tier = final_tier;
    g_auto_relay_cut = final_tier == 5
        || (final_tier >= 4 && (fusion.recommend_relay_cutoff || fusion.fire_type == 1));
    updateRelay();

    // A live warning takes precedence over setup simulation outputs.
    if (g_tier_test_override >= 0) {
        if (final_tier < 3) return;
        clearTierTest();
    }

    ResponseData resp = stepResponseEngine(final_tier);
    LogData log_data  = stepLoggingAudit(final_tier, fusion, states);

    bool backend_ok = sendReadingToBackend(smoothed, derived, states, fusion, cam_dec,
                                            initial_tier, final_tier, resp);

    if (DEBUG_SERIAL_OUTPUT) {
        printDebugSummary(smoothed, states, fusion, final_tier, backend_ok);
    }
}

// ============================================================================
// ── STEP 2: SIGNAL SMOOTHING & CONVERSION ───────────────────────────────────
// ============================================================================
SmoothedData stepSmoothing(float raw_mq7_v, float raw_acs_v, float temp, float hum) {
    SmoothedData s;

    float raw_acs_clamped = constrain(raw_acs_v, 0.0f, 5.0f);
    float current_raw = (raw_acs_clamped - ACS712_VZERO) / ACS712_SENSITIVITY;

    if (g_prev_smoothed_current < -900.0f) {
        s.acs_current = current_raw;
    } else {
        s.acs_current = 0.9f * g_prev_smoothed_current + 0.1f * current_raw;
    }
    g_prev_smoothed_current = s.acs_current;

    float raw_mq7_clamped = constrain(raw_mq7_v, 0.01f, 5.0f);
    float ratio = (2.3f / raw_mq7_clamped) - 0.23f;
    if (ratio <= 0.0f) ratio = 0.01f;

    float ppm_raw = 99.042f * powf(ratio, -1.518f);
    ppm_raw = constrain(ppm_raw, 0.0f, 2000.0f);

    if (g_prev_smoothed_ppm < 0.0f) {
        s.mq7_ppm = ppm_raw;
    } else {
        s.mq7_ppm = 0.9f * g_prev_smoothed_ppm + 0.1f * ppm_raw;
    }
    g_prev_smoothed_ppm = s.mq7_ppm;

    s.temperature = temp;
    s.humidity    = hum;

    return s;
}

// ============================================================================
// ── STEP 2.4: BASELINE NORMALIZATION ────────────────────────────────────────
// ============================================================================
void stepBaselineNormalization(SmoothedData& s) {
    if (g_baseline_sample_count < BASELINE_SAMPLES) {
        g_warmup_active = true;

        if (g_baseline_sample_count == 0) {
            g_baseline_ppm     = s.mq7_ppm;
            g_baseline_temp    = s.temperature;
            g_baseline_hum     = s.humidity;
            g_baseline_current = fabsf(s.acs_current);
        } else {
            int n = g_baseline_sample_count + 1;
            g_baseline_ppm     += (s.mq7_ppm             - g_baseline_ppm)     / n;
            g_baseline_temp    += (s.temperature          - g_baseline_temp)    / n;
            g_baseline_hum     += (s.humidity             - g_baseline_hum)     / n;
            g_baseline_current += (fabsf(s.acs_current)   - g_baseline_current) / n;
        }

        g_baseline_sample_count++;
        g_dev_ppm = g_dev_temp = g_dev_hum = g_dev_current = 0.0f;

        if (g_baseline_sample_count >= BASELINE_SAMPLES) {
            g_mq7_zero_ppm = g_baseline_ppm;
        }
        return;
    }

    g_warmup_active = false;

    g_baseline_ppm     = ALPHA_BASELINE * s.mq7_ppm          + (1.0f - ALPHA_BASELINE) * g_baseline_ppm;
    g_baseline_temp    = ALPHA_BASELINE * s.temperature       + (1.0f - ALPHA_BASELINE) * g_baseline_temp;
    g_baseline_hum     = ALPHA_BASELINE * s.humidity          + (1.0f - ALPHA_BASELINE) * g_baseline_hum;
    g_baseline_current = ALPHA_BASELINE * fabsf(s.acs_current)+ (1.0f - ALPHA_BASELINE) * g_baseline_current;

    g_baseline_sample_count++;

    g_dev_ppm     = calcDeviation(s.mq7_ppm,           g_baseline_ppm);
    g_dev_temp    = calcDeviation(s.temperature,        g_baseline_temp);
    g_dev_hum     = calcDeviation(s.humidity,           g_baseline_hum);
    g_dev_current = calcDeviation(fabsf(s.acs_current), g_baseline_current);
}

// ============================================================================
// ── STEP 2.5: DERIVED FEATURES (Rate-of-Rise & Current Delta) ───────────────
// ============================================================================
float medianTemperature(float first, float second, float third) {
    const float lower = min(first, second);
    const float upper = max(first, second);
    return max(lower, min(upper, third));
}

DerivedData stepDerivedFeatures(SmoothedData& s, bool temperature_valid) {
    DerivedData d;
    d.temp_ror          = -999.0f;
    d.current_delta_A   = 0.0f;
    d.current_delta_pct = 0.0f;

    if (temperature_valid) {
        if (g_ror_count == ROR_BUFFER_SAMPLES) {
            for (int i = 1; i < ROR_BUFFER_SAMPLES; i++) {
                g_ror_temp_buf[i - 1] = g_ror_temp_buf[i];
                g_ror_time_buf[i - 1] = g_ror_time_buf[i];
            }
            g_ror_count--;
        }
        g_ror_temp_buf[g_ror_count] = s.temperature;
        g_ror_time_buf[g_ror_count] = millis();
        g_ror_count++;

        if (g_ror_count == ROR_BUFFER_SAMPLES) {
            const int recent = ROR_BUFFER_SAMPLES - 3;
            // Median endpoints keep a single bad sample at either end from driving the rate.
            const float older = medianTemperature(g_ror_temp_buf[0], g_ror_temp_buf[1], g_ror_temp_buf[2]);
            const float newer = medianTemperature(g_ror_temp_buf[recent], g_ror_temp_buf[recent + 1],
                                                  g_ror_temp_buf[recent + 2]);
            const unsigned long elapsed_ms = g_ror_time_buf[recent + 1] - g_ror_time_buf[1];
            if (elapsed_ms > 0) {
                d.temp_ror = constrain((newer - older) * 60000.0f / elapsed_ms, -20.0f, 20.0f);
            }
        }
    }

    float abs_current = fabsf(s.acs_current);
    d.current_delta_A   = abs_current - g_baseline_current;
    d.current_delta_pct = g_dev_current;

    return d;
}

// ============================================================================
// ── STEP 3: SENSOR STATE CLASSIFICATION ─────────────────────────────────────
// ============================================================================
SensorStates stepSensorClassification(SmoothedData& s, DerivedData& d) {
    SensorStates st;
    st.camera = g_camera_state_prev;

    if (g_warmup_active) {
        st.mq7 = 0; st.temperature = 0; st.acs712 = 0;
        return st;
    }

    // MQ-7: Rise above clean-air reference
    {
        int abs_st = 0;
        float ppm = s.mq7_ppm - g_mq7_zero_ppm;
        if (ppm < 0.0f) ppm = 0.0f;

        if      (ppm >= 350) abs_st = 3;
        else if (ppm >= 120) abs_st = 2;
        else if (ppm >= g_remote_settings.ppmAdvisory) abs_st = 1;

        const bool severe_deviation = g_dev_ppm >= 150.0f;
        int dev_st = 0;
        // Confirm a baseline-relative spike. The first sample still contributes moderate evidence.
        if      (severe_deviation && g_previous_severe_mq7_deviation) dev_st = 3;
        else if (g_dev_ppm >= 80.0f) dev_st = 2;
        else if (g_dev_ppm >= 30.0f) dev_st = 1;
        g_previous_severe_mq7_deviation = severe_deviation;

        st.mq7 = max(abs_st, dev_st);
    }

    // Temperature (Absolute floor tuned to 38°C for PH climate)
    {
        int abs_st = 0;
        float temp = s.temperature;
        if      (temp >= 55) abs_st = 3;
        else if (temp >= 45) abs_st = 2;
        else if (temp >= g_remote_settings.tempAdvisory) abs_st = 1;

        const bool severe_ror = d.temp_ror >= 5.0f;
        int ror_st = 0;
        // Keep the first fast rise available for fusion, and confirm a rate-only severe state.
        if      (severe_ror && g_previous_severe_ror) ror_st = 3;
        else if (d.temp_ror >= 2.0f) ror_st = 2;
        else if (d.temp_ror >= 0.5f) ror_st = 1;
        g_previous_severe_ror = severe_ror;

        int dev_st = 0;
        float dev = g_dev_temp;
        if (dev > 0) {
            if      (dev >= 90) dev_st = 3;
            else if (dev >= 50) dev_st = 2;
            else if (dev >= 20) dev_st = 1;
        }

        st.temperature = max(abs_st, max(ror_st, dev_st));
    }

    // ACS712 Current
    {
        float abs_I = fabsf(s.acs_current);
        float delta = fabsf(d.current_delta_A);

        int abs_st = 0;
        if      (abs_I >= 12) abs_st = 3;
        else if (abs_I >= 10) abs_st = 2;
        else if (abs_I >= g_remote_settings.currentAdvisory) abs_st = 1;

        // Confirm a mild delta in the same direction. Larger changes still classify immediately.
        int delta_direction = 0;
        if (delta >= 0.5f) delta_direction = d.current_delta_A > 0.0f ? 1 : -1;

        int delta_st = 0;
        if      (delta >= 3.0f) delta_st = 3;
        else if (delta >= 1.5f) delta_st = 2;
        else if (delta_direction != 0 && delta_direction == g_previous_acs_delta_direction) delta_st = 1;
        g_previous_acs_delta_direction = delta_direction;

        st.acs712 = max(abs_st, delta_st);
    }

    return st;
}

// ============================================================================
// ── STEP 4: PERSISTENCE TRACKING ────────────────────────────────────────────
// ============================================================================
void stepPersistenceUpdate(SensorStates& st) {
    bool any_abnormal = (st.mq7 >= 1 || st.temperature >= 1 || st.acs712 >= 1);
    if (any_abnormal) {
        g_persist.consecutive_abnormal++;
    } else {
        g_persist.consecutive_abnormal = 0;
    }
    g_persist.abnormal_duration_sec = g_persist.consecutive_abnormal * SAMPLE_INTERVAL_SEC;

    if (st.mq7 > 0) {
        if (st.mq7 >= g_persist.mq7.state) {
            g_persist.mq7.count++;
            g_persist.mq7.state = st.mq7;
        } else {
            g_persist.mq7.count = 1;
            g_persist.mq7.state = st.mq7;
        }
    } else {
        g_persist.mq7 = {0, 0};
    }

    if (st.temperature > 0) {
        if (st.temperature >= g_persist.temperature.state) {
            g_persist.temperature.count++;
            g_persist.temperature.state = st.temperature;
        } else {
            g_persist.temperature.count = 1;
            g_persist.temperature.state = st.temperature;
        }
    } else {
        g_persist.temperature = {0, 0};
    }

    if (st.acs712 > 0) {
        if (st.acs712 >= g_persist.acs712.state) {
            g_persist.acs712.count++;
            g_persist.acs712.state = st.acs712;
        } else {
            g_persist.acs712.count = 1;
            g_persist.acs712.state = st.acs712;
        }
    } else {
        g_persist.acs712 = {0, 0};
    }
}

// ============================================================================
// ── STEP 5: INITIAL TIER ESTIMATE ───────────────────────────────────────────
// ============================================================================
int stepInitialTierEstimate(SensorStates& st, int& tier_floor) {
    int mq7  = st.mq7;
    int temp = st.temperature;
    int acs  = st.acs712;
    int cam  = st.camera;

    float dur        = g_persist.abnormal_duration_sec;
    float dur_mq7    = g_persist.mq7.count         * SAMPLE_INTERVAL_SEC;
    float dur_temp   = g_persist.temperature.count  * SAMPLE_INTERVAL_SEC;
    float dur_acs    = g_persist.acs712.count       * SAMPLE_INTERVAL_SEC;

    tier_floor = 0;

    if (cam == 3) {
        tier_floor = 5;
    } else if (mq7 == 3 && temp >= 2 && dur_mq7 >= 20) {
        tier_floor = 5;
    } else if (mq7 == 3 && temp == 3 && acs >= 2) {
        tier_floor = 5;
    } else if (mq7 == 3 || temp == 3 || acs == 3) {
        tier_floor = 3;
    }

    int gte1 = (mq7 >= 1) + (temp >= 1) + (acs >= 1);
    int gte2 = (mq7 >= 2) + (temp >= 2) + (acs >= 2);
    int eq3  = (mq7 == 3) + (temp == 3) + (acs == 3);

    int tier = 0;
    if (gte1 >= 1) tier = max(tier, 1);
    if (gte2 >= 1 && (dur_mq7 >= 12 || dur_temp >= 12 || dur_acs >= 12)) tier = max(tier, 2);
    if (gte1 >= 2) tier = max(tier, 2);
    if (eq3 >= 1)  tier = max(tier, 3);
    if (gte2 >= 2) tier = max(tier, 3);
    if (cam >= 1 && gte1 >= 1) tier = max(tier, 3);
    if (cam >= 2 && gte2 >= 1) tier = max(tier, 4);
    if (eq3 >= 2 && dur >= 14) tier = max(tier, 4);
    if (cam == 3 && gte2 >= 1) tier = max(tier, 5);
    if (eq3 >= 3 && dur >= 20) tier = max(tier, 5);

    return constrain(max(tier, tier_floor), 0, 5);
}

// ============================================================================
// ── STEP 6: FUSION LOGIC ────────────────────────────────────────────────────
// ============================================================================
FusionData stepFusionLogic(SensorStates& st) {
    FusionData f;

    const int W_MQ7[]  = {0, 1, 3, 5};
    const int W_TEMP[] = {0, 1, 2, 4};
    const int W_ACS[]  = {0, 1, 2, 4};
    const int W_CAM[]  = {0, 2, 4, 7};

    int base = W_MQ7[st.mq7] + W_TEMP[st.temperature] + W_ACS[st.acs712] + W_CAM[st.camera];

    f.mod_humidity_drying = 0;
    if (g_dev_hum < -10.0f && g_dev_temp > 10.0f && st.temperature >= 1)
        f.mod_humidity_drying = 1;

    f.mod_weak_evidence = 0;
    int moderate_count = (st.mq7 >= 2) + (st.temperature >= 2) + (st.acs712 >= 2);
    if (st.camera == 0 && moderate_count == 1)
        f.mod_weak_evidence = -1;

    f.mod_persistent_dual = 0;
    int sustained_cats = 0;
    if (g_persist.mq7.count         * SAMPLE_INTERVAL_SEC >= 12 && st.mq7 >= 1)         sustained_cats++;
    if (g_persist.temperature.count * SAMPLE_INTERVAL_SEC >= 12 && st.temperature >= 1) sustained_cats++;
    if (g_persist.acs712.count      * SAMPLE_INTERVAL_SEC >= 12 && st.acs712 >= 1)      sustained_cats++;
    if (sustained_cats >= 2) f.mod_persistent_dual = 1;

    f.mod_short_spike = 0;
    if (g_persist.abnormal_duration_sec > 0 && g_persist.abnormal_duration_sec < 4)
        f.mod_short_spike = -1;

    int total_mod = f.mod_humidity_drying + f.mod_weak_evidence +
                    f.mod_persistent_dual + f.mod_short_spike;

    f.score = max(0, base + total_mod);

    if      (f.score >= 12) f.fusion_tier = 5;
    else if (f.score >= 9)  f.fusion_tier = 4;
    else if (f.score >= 6)  f.fusion_tier = 3;
    else if (f.score >= 4)  f.fusion_tier = 2;
    else if (f.score >= 2)  f.fusion_tier = 1;
    else                    f.fusion_tier = 0;

    f.fire_type = 0;
    if (st.acs712 >= 2 && st.mq7 <= 1 && st.temperature >= 1) {
        f.fire_type = 1; // Electrical
    } else if (st.mq7 >= 2 && st.acs712 == 0 && st.camera == 0) {
        f.fire_type = 2; // Smoldering
    } else if (st.mq7 >= 2 && st.temperature >= 2 && st.acs712 >= 2) {
        f.fire_type = 3; // Open fire
    } else if (st.mq7 >= 1 && st.temperature >= 1 && st.camera >= 1) {
        f.fire_type = 3; // Open fire corroborated by vision
    }

    f.recommend_relay_cutoff = (f.fire_type == 1 && st.acs712 >= 3);

    return f;
}

// ============================================================================
// ── STEP 7: CAMERA WAKE DECISION & ASYNC INFERENCE TRIGGER ──────────────────
// ============================================================================
CameraDecision stepCameraWakeDecision(SensorStates& st, FusionData& f, int initial_tier) {
    CameraDecision cd;
    cd.should_wake    = false;
    cd.camera_state   = 0;
    cd.confidence_pct = 0;
    cd.warming_ticks  = g_camera_warming;
    if (!g_remote_settings.cameraEnabled) return cd;

    float dur_mq7  = g_persist.mq7.count         * SAMPLE_INTERVAL_SEC;
    float dur_temp = g_persist.temperature.count  * SAMPLE_INTERVAL_SEC;
    float dur_acs  = g_persist.acs712.count       * SAMPLE_INTERVAL_SEC;

    if (st.mq7 >= 2 && st.temperature >= 1) cd.should_wake = true;
    if (st.acs712 >= 2 && st.temperature >= 1) cd.should_wake = true;
    if (st.mq7 >= 1 && st.acs712 >= 2) cd.should_wake = true;
    if (st.mq7 == 3 || st.temperature == 3 || st.acs712 == 3) cd.should_wake = true;

    if (g_persist.mq7.state >= 2         && dur_mq7  >= 20) cd.should_wake = true;
    if (g_persist.temperature.state >= 2 && dur_temp >= 20) cd.should_wake = true;
    if (g_persist.acs712.state >= 2      && dur_acs  >= 20) cd.should_wake = true;

    if (initial_tier >= 3 || f.fusion_tier >= 3) cd.should_wake = true;

    if (!cd.should_wake) {
        g_camera_warming = 0;
        return cd;
    }

    g_camera_warming++;
    cd.warming_ticks = g_camera_warming;

    if (g_camera_warming < 2) {
        cd.camera_state = 0;
        return cd;
    }

    // Trigger asynchronous non-blocking inference & capture query if none pending
    if (millis() - g_last_cam_query_ms >= 3000) requestCameraCapture(false);

    // Use latest confirmed camera state from background receiver
    cd.confidence_pct = g_cam_latest_fire_pct;
    cd.camera_state   = g_cam_latest_state;

    return cd;
}

// ============================================================================
// ── STEP 8: FINAL TIER DECISION ─────────────────────────────────────────────
// ============================================================================
int stepFinalTierDecision(int initial_tier, int tier_floor, FusionData& f, SensorStates& st) {
    const int DEESC_COOLDOWN[] = { 0, 60, 60, 30, 30, 15 };

    int candidate = max(initial_tier, f.fusion_tier);
    int cam = st.camera;

    if      (cam == 3 && candidate >= 2) candidate = max(candidate, 5);
    else if (cam == 2 && candidate >= 3) candidate = max(candidate, 4);
    else if (cam == 1 && candidate >= 2) candidate = max(candidate, 3);

    candidate = max(candidate, tier_floor);

    int max_jump = (tier_floor > 0) ? 5 : 2;
    if (candidate > g_last_tier + max_jump)
        candidate = g_last_tier + max_jump;

    int final_tier = candidate;
    bool any_abnormal = (st.mq7 >= 1 || st.temperature >= 1 || st.acs712 >= 1);

    if (candidate < g_last_tier) {
        if (any_abnormal) {
            final_tier = g_last_tier;
            g_improving_samples = 0;
        } else {
            g_improving_samples++;
            int needed = DEESC_COOLDOWN[g_last_tier];
            if (g_improving_samples >= needed) {
                final_tier = g_last_tier - 1;
            } else {
                final_tier = g_last_tier;
            }
        }
    } else {
        if (any_abnormal) {
            g_improving_samples = 0;
        } else {
            g_improving_samples++;
        }
    }

    if (final_tier == g_last_tier) {
        g_at_tier_since++;
    } else {
        g_at_tier_since = 0;
    }

    final_tier = constrain(final_tier, 0, 5);
    g_last_tier = final_tier;
    g_current_tier_est = final_tier;

    return final_tier;
}

// ============================================================================
// ── STEP 9: RESPONSE ENGINE ─────────────────────────────────────────────────
// ============================================================================
ResponseData stepResponseEngine(int final_tier) {
    ResponseData r;
    r.relay_cut         = relayCutRequested();
    r.siren_active      = false;
    r.camera_active     = false;
    r.send_notification = false;
    r.notif_tier        = 0;

    updateLEDs(final_tier);

    if (final_tier >= 4) r.siren_active = true;
    if (final_tier >= 3 && g_remote_settings.cameraEnabled) r.camera_active = true;

    if (final_tier < SMS_ALERT_TIER) {
        g_sms_peak_tier = 0;
    } else if (final_tier > g_sms_peak_tier && queueSmsAlert(final_tier)) {
        r.send_notification = true;
        r.notif_tier = final_tier;
        g_sms_peak_tier = final_tier;
    }

    r.siren_active = updateBuzzer(r.siren_active);

    return r;
}

// ============================================================================
// ── STEP 10: LOGGING & DE-ESCALATION AUDIT ──────────────────────────────────
// ============================================================================
LogData stepLoggingAudit(int final_tier, FusionData& f, SensorStates& st) {
    LogData l;
    l.transition_type        = 0;
    l.in_cooldown            = false;
    l.cooldown_remaining_sec = 0;
    l.peak_tier              = g_peak_tier;
    l.log_entry_count        = g_log_entry_count;

    int prev = g_log_last_tier;

    if (prev == 0 && final_tier > 0) {
        l.transition_type = 1;
        g_event_active    = true;
    } else if (final_tier > prev) {
        l.transition_type = 2;
    } else if (final_tier < prev && final_tier > 0) {
        l.transition_type = 3;
    } else if (final_tier == 0 && prev > 0) {
        l.transition_type = 4;
        g_post_alarm_ctr  = 0;
        g_event_active    = false;
    }

    if (final_tier > g_peak_tier) g_peak_tier = final_tier;
    if (final_tier == 0 && prev == 0) g_peak_tier = 0;
    l.peak_tier = g_peak_tier;

    if (final_tier == 0 && g_post_alarm_ctr < POST_ALARM_COOLDOWN) {
        g_post_alarm_ctr++;
        l.in_cooldown            = true;
        l.cooldown_remaining_sec = (int)((POST_ALARM_COOLDOWN - g_post_alarm_ctr) * SAMPLE_INTERVAL_SEC);
    }

    if (g_log_entry_count < 100) g_log_entry_count++;
    l.log_entry_count = g_log_entry_count;
    g_log_last_tier = final_tier;

    return l;
}

// ============================================================================
// ── HARDWARE OUTPUT DRIVERS ─────────────────────────────────────────────────
// ============================================================================
bool relayCutRequested() {
    return g_auto_relay_cut || g_manual_relay_cut || g_tier_test_relay_cut;
}

void clearTierTest() {
    // A simulated acknowledgement must not suppress a subsequent live alarm.
    g_alarm_hush.clear();
    g_tier_test_override = -1;
    g_tier_test_relay_cut = false;
    updateLEDs(g_ui_tier);
    updateBuzzer(g_ui_tier >= 4);
    updateRelay();
}

int relayOutputLevel(bool cut) {
    const bool output_high = RELAY_ACTIVE_LOW ? !cut : cut;
    return output_high ? HIGH : LOW;
}

void updateRelay() {
    digitalWrite(RELAY_PIN, relayOutputLevel(relayCutRequested()));
}

void setManualRelayCut(bool cut) {
    g_manual_relay_cut = cut;
    updateRelay();
    const bool effective_cut = relayCutRequested();
    Serial.printf("[RELAY] Manual cut %s. Output command: %s (GPIO%d %s).\n",
                  cut ? "requested" : "cleared", effective_cut ? "CUT" : "RELEASE",
                  RELAY_PIN, relayOutputLevel(effective_cut) == HIGH ? "HIGH" : "LOW");
}

void runRelayTest(int seconds) {
    if (seconds <= 0) seconds = 5;
    Serial.println("\n[RELAY TEST] ================================================");
    Serial.printf("[RELAY TEST] STARTING %d-SECOND TIMED POWER CUT TEST\n", seconds);
    Serial.println("[RELAY TEST] Requesting relay cutoff.");
    Serial.println("[RELAY TEST] ================================================");

    const bool previous_manual_cut = g_manual_relay_cut;
    setManualRelayCut(true);

    for (int s = seconds; s > 0; s--) {
        Serial.printf("[RELAY TEST] Cut active... %d second%s remaining\n", s, s == 1 ? "" : "s");
        delay(1000);
    }

    setManualRelayCut(previous_manual_cut);
    Serial.println("[RELAY TEST] Timed relay test completed.\n");
}

void printRelayStatus() {
    const bool effective_cut = relayCutRequested();
    Serial.println("\n================ [RELAY STATUS] ================");
    Serial.printf("GPIO Output Level:  GPIO%d %s (commanded)\n", RELAY_PIN,
                  relayOutputLevel(effective_cut) == HIGH ? "HIGH" : "LOW");
    Serial.printf("Trigger Logic:      %s\n", RELAY_ACTIVE_LOW ? "ACTIVE-LOW" : "ACTIVE-HIGH");
    Serial.printf("Automatic Cut:      %s\n", g_auto_relay_cut ? "REQUESTED" : "CLEAR");
    Serial.printf("Manual Cut:         %s\n", g_manual_relay_cut ? "REQUESTED" : "CLEAR");
    Serial.printf("Tier Test Cut:      %s\n", g_tier_test_relay_cut ? "REQUESTED" : "CLEAR");
    Serial.printf("Relay Command:      %s\n", effective_cut ? "CUT" : "RELEASE");
    Serial.printf("Pipeline Tier:      Tier %d\n", g_last_tier);
    Serial.println("================================================\n");
}

int effectiveAlertTier() {
    return max(g_ui_tier, g_tier_test_override);
}

bool updateBuzzer(bool active) {
    const bool hushed = g_alarm_hush.active(effectiveAlertTier(), millis());
    g_buzzer_active = ENABLE_BUZZER && active && !g_standby_active && !hushed;
    digitalWrite(BUZZER_PIN, g_buzzer_active ? HIGH : LOW);
    return g_buzzer_active;
}

void updateLEDs(int final_tier) {
    digitalWrite(LED_TIER1_PIN, LOW);
    digitalWrite(LED_TIER2_PIN, LOW);
    digitalWrite(LED_TIER3_PIN, LOW);
    digitalWrite(LED_TIER4_PIN, LOW);
    digitalWrite(LED_TIER5_PIN, LOW);

    switch (final_tier) {
        case 1: digitalWrite(LED_TIER1_PIN, HIGH); break;
        case 2: digitalWrite(LED_TIER2_PIN, HIGH); break;
        case 3: digitalWrite(LED_TIER3_PIN, HIGH); break;
        case 4: digitalWrite(LED_TIER4_PIN, HIGH); break;
        case 5:
            digitalWrite(LED_TIER1_PIN, HIGH);
            digitalWrite(LED_TIER2_PIN, HIGH);
            digitalWrite(LED_TIER3_PIN, HIGH);
            digitalWrite(LED_TIER4_PIN, HIGH);
            digitalWrite(LED_TIER5_PIN, HIGH);
            break;
        default: break;
    }
}

// ============================================================================
// ── BACKEND HTTP CLIENT (Non-blocking retry & backoff) ──────────────────────
// ============================================================================
bool sendReadingToBackend(SmoothedData& s, DerivedData& d, SensorStates& st,
                           FusionData& f, CameraDecision& cd, int initial_tier,
                           int final_tier, ResponseData& r) {

    if (!g_owner_active.load() || WiFi.status() != WL_CONNECTED || g_portal_active) return false;

    if (millis() < g_backend_backoff_until_ms) return false;

    if (final_tier == g_last_uploaded_tier
        && millis() - g_last_reading_upload_ms < g_remote_settings.reportIntervalSec * 1000UL
        && g_deferred_reading_payload.isEmpty()) {
        return true;
    }

    StaticJsonDocument<1536> doc;
    if (g_remote_settings.version[0]) doc["config_version"] = g_remote_settings.version;

    doc["smooth_ppm"]            = round(s.mq7_ppm * 10.0f) / 10.0f;
    doc["smooth_temperature"]    = round(s.temperature * 10.0f) / 10.0f;
    doc["smooth_humidity"]       = round(s.humidity * 10.0f) / 10.0f;
    doc["smooth_current"]        = round(s.acs_current * 1000.0f) / 1000.0f;

    doc["deviation_ppm"]         = round(g_dev_ppm * 10.0f) / 10.0f;
    doc["deviation_temperature"] = round(g_dev_temp * 10.0f) / 10.0f;
    doc["deviation_humidity"]    = round(g_dev_hum * 10.0f) / 10.0f;
    doc["deviation_current"]     = round(g_dev_current * 10.0f) / 10.0f;

    doc["temp_ror"]              = (d.temp_ror > -998.0f) ? (float)(round(d.temp_ror * 100.0f) / 100.0f) : 0.0f;
    doc["temp_ror_valid"]        = (d.temp_ror > -998.0f);
    doc["curr_delta_A"]          = round(d.current_delta_A * 1000.0f) / 1000.0f;
    doc["curr_delta_pct"]        = round(d.current_delta_pct * 10.0f) / 10.0f;

    doc["state_mq7"]             = st.mq7;
    doc["state_temp"]            = st.temperature;
    doc["state_acs"]             = st.acs712;
    doc["state_camera"]          = st.camera;
    doc["label_mq7"]             = STATE_LABELS[st.mq7];
    doc["label_temp"]            = STATE_LABELS[st.temperature];
    doc["label_acs"]             = STATE_LABELS[st.acs712];
    doc["label_camera"]          = STATE_LABELS[st.camera];

    doc["consec_abnormal"]       = g_persist.consecutive_abnormal;
    doc["abnormal_dur_sec"]      = (int)g_persist.abnormal_duration_sec;
    doc["streak_mq7_state"]      = g_persist.mq7.state;
    doc["streak_mq7_sec"]        = (int)(g_persist.mq7.count * SAMPLE_INTERVAL_SEC);
    doc["streak_temp_state"]     = g_persist.temperature.state;
    doc["streak_temp_sec"]       = (int)(g_persist.temperature.count * SAMPLE_INTERVAL_SEC);
    doc["streak_acs_state"]      = g_persist.acs712.state;
    doc["streak_acs_sec"]        = (int)(g_persist.acs712.count * SAMPLE_INTERVAL_SEC);

    doc["initial_tier"]          = initial_tier;
    doc["fusion_score"]          = f.score;
    doc["fusion_tier"]           = f.fusion_tier;
    doc["fire_type"]             = FIRE_TYPES[f.fire_type];
    doc["relay_recommend"]       = f.recommend_relay_cutoff;
    doc["mod_hum_dry"]           = (f.mod_humidity_drying != 0);
    doc["mod_weak_ev"]           = (f.mod_weak_evidence != 0);
    doc["mod_persist"]           = (f.mod_persistent_dual != 0);
    doc["mod_spike"]             = (f.mod_short_spike != 0);

    doc["cam_wake"]              = cd.should_wake;
    doc["cam_state"]             = cd.camera_state;
    doc["cam_confidence"]        = (float)cd.confidence_pct;
    doc["cam_warming"]           = cd.warming_ticks;

    doc["final_tier"]            = final_tier;
    doc["tier_name"]             = TIER_NAMES[final_tier];
    doc["at_tier_since"]         = g_at_tier_since;
    doc["improving"]             = g_improving_samples;

    doc["relay_cut"]             = r.relay_cut;
    doc["siren"]                 = r.siren_active;
    doc["cam_active"]            = r.camera_active;
    doc["send_notif"]            = r.send_notification;
    doc["notif_tier"]            = r.notif_tier;

    String payload;
    serializeJson(doc, payload);

    // A synchronous sensor POST can block the main loop while the camera is
    // returning inference text or JPEG bytes. Keep the latest reading queued
    // until that camera transaction finishes so UART RX is drained promptly.
    if (g_cam_query_pending || g_cam_image_streaming) {
        g_deferred_reading_payload = payload;
        g_deferred_reading_tier = final_tier;
        return true;
    }
    return postReadingPayloadToBackend(payload, final_tier);
}

bool postReadingPayloadToBackend(const String& payload, int final_tier) {
    if (!g_owner_active.load() || WiFi.status() != WL_CONNECTED || g_portal_active) return false;
    if (millis() < g_backend_backoff_until_ms) return false;

    WiFiClientSecure client;
    HTTPClient http;
    bool ok = false;

    if (beginBackendRequest(http, client, g_backend_readings_url)) {
        http.addHeader("Content-Type", "application/json");
        http.addHeader("X-Device-Id", g_device_id);
        http.addHeader("X-Device-Key", g_device_key);

        int code = http.POST(payload);

        if (code == 200 || code == 201) {
            ok = true;
            g_backend_consecutive_fails = 0;
            g_deferred_reading_payload = "";
            g_last_reading_upload_ms = millis();
            g_last_uploaded_tier = final_tier;
        } else {
            g_backend_consecutive_fails++;
            if (g_backend_consecutive_fails >= 3) {
                g_backend_backoff_until_ms = millis() + 10000;
            }
        }
        http.end();
    }
    return ok;
}

void flushDeferredReadingToBackend() {
    if (g_deferred_reading_payload.length() == 0) return;
    if (postReadingPayloadToBackend(g_deferred_reading_payload, g_deferred_reading_tier)) {
        Serial.println("[Backend] Deferred sensor reading uploaded after camera stream.");
    } else {
        Serial.println("[Backend] Deferred sensor reading remains queued for the next retry.");
    }
}

bool uploadCaptureToBackend(const uint8_t* jpeg, size_t len, float confidence) {
    if (!g_owner_active.load()) return false;
    if (!g_remote_settings.cameraEnabled) return false;
    if (WiFi.status() != WL_CONNECTED || g_portal_active) {
        Serial.println("[Capture] WiFi not connected or portal active — skipping capture upload.");
        return false;
    }
    if (jpeg == nullptr || len < 4 || len > 96 * 1024) return false;
    if (jpeg[0] != 0xFF || jpeg[1] != 0xD8 || jpeg[len - 2] != 0xFF || jpeg[len - 1] != 0xD9) {
        Serial.println("[Capture] Refusing upload: JPEG markers are invalid.");
        return false;
    }

    String boundary = "----preventa" + String((unsigned long)millis(), HEX);
    String header = "--" + boundary + "\r\n"
                    "Content-Disposition: form-data; name=\"image\"; filename=\"capture.jpg\"\r\n"
                    "Content-Type: image/jpeg\r\n\r\n";

    String confField = "\r\n--" + boundary + "\r\n"
                       "Content-Disposition: form-data; name=\"confidence\"\r\n\r\n" +
                       String(confidence, 1) + "\r\n";

    String classLabel = (confidence >= 60.0f) ? "fire" : ((confidence >= 30.0f) ? "smoke" : "normal");
    String classField = "--" + boundary + "\r\n"
                        "Content-Disposition: form-data; name=\"classification\"\r\n\r\n" +
                        classLabel + "\r\n";

    String footer = "--" + boundary + "--\r\n";

    const size_t total_len = header.length() + len + confField.length() + classField.length() + footer.length();

    // Present the multipart pieces as one readable stream. This avoids a
    // second full-size JPEG/body allocation while the received JPEG is still
    // resident in RAM.
    class MultipartImageStream : public Stream {
    public:
        MultipartImageStream(const String& a, const uint8_t* image, size_t imageLen,
                             const String& b, const String& c, const String& d)
            : partIndex(0), offset(0) {
            parts[0] = (const uint8_t*)a.c_str(); lengths[0] = a.length();
            parts[1] = image; lengths[1] = imageLen;
            parts[2] = (const uint8_t*)b.c_str(); lengths[2] = b.length();
            parts[3] = (const uint8_t*)c.c_str(); lengths[3] = c.length();
            parts[4] = (const uint8_t*)d.c_str(); lengths[4] = d.length();
        }

        int available() override {
            size_t left = 0;
            for (size_t i = partIndex; i < 5; ++i) {
                if (i == partIndex) left += lengths[i] - offset;
                else left += lengths[i];
            }
            return (int)left;
        }

        int read() override {
            skipEmpty();
            if (partIndex >= 5) return -1;
            return parts[partIndex][offset++];
        }

        int peek() override {
            skipEmpty();
            if (partIndex >= 5) return -1;
            return parts[partIndex][offset];
        }

        void flush() override {}
        size_t write(uint8_t) override { return 0; }

        size_t readBytes(char* buffer, size_t length) override {
            size_t copied = 0;
            while (copied < length) {
                skipEmpty();
                if (partIndex >= 5) break;
                const size_t count = min(length - copied, lengths[partIndex] - offset);
                memcpy(buffer + copied, parts[partIndex] + offset, count);
                copied += count;
                offset += count;
            }
            return copied;
        }

    private:
        const uint8_t* parts[5];
        size_t lengths[5];
        size_t partIndex;
        size_t offset;

        void skipEmpty() {
            while (partIndex < 5 && offset >= lengths[partIndex]) {
                ++partIndex;
                offset = 0;
            }
        }
    } bodyStream(header, jpeg, len, confField, classField, footer);

    WiFiClientSecure client;
    HTTPClient http;
    http.setTimeout(20000);
    bool ok = false;

    if (beginBackendRequest(http, client, g_backend_captures_url)) {
        http.addHeader("Content-Type", "multipart/form-data; boundary=" + boundary);
        http.addHeader("X-Device-Id", g_device_id);
        http.addHeader("X-Device-Key", g_device_key);

        int code = http.sendRequest("POST", &bodyStream, total_len);
        if (code == 200 || code == 201) {
            ok = true;
            Serial.printf("[Capture] Image uploaded successfully (HTTP %d). View in Dashboard -> History -> Captures.\n", code);
        } else {
            Serial.printf("[Capture] Backend upload returned HTTP %d: %s\n", code, http.getString().c_str());
        }
        http.end();
    } else {
        Serial.println("[Capture] Could not connect to captures endpoint: " + g_backend_captures_url);
    }
    return ok;
}

// ============================================================================
// ── A7670C SMS ALERT DISPATCH ───────────────────────────────────────────────
// ============================================================================
String sendModemCommand(const String& command, const char* expected,
                        unsigned long timeout_ms) {
    while (modemSerial.available()) modemSerial.read();
    const bool contains_number = command.startsWith("AT+CMGS=");
    Serial.printf("[MODEM] >> %s\n", contains_number ? "AT+CMGS=<recipient>" : command.c_str());
    modemSerial.print(command);
    modemSerial.print("\r\n");

    String response;
    const unsigned long started_ms = millis();
    unsigned long last_byte_ms = started_ms;
    bool matched = false;
    while (millis() - started_ms < timeout_ms) {
        while (modemSerial.available()) {
            response += (char)modemSerial.read();
            last_byte_ms = millis();
            if (expected && response.indexOf(expected) >= 0) matched = true;
        }
        if (response.indexOf("ERROR") >= 0) break;
        if (matched && millis() - last_byte_ms >= 120) break;
        delay(1);
    }
    if (contains_number) {
        Serial.println(matched ? "[MODEM] << SMS prompt received" : "[MODEM] << SMS prompt missing");
    } else if (response.length()) {
        Serial.print("[MODEM] << ");
        Serial.print(response);
        if (!response.endsWith("\n")) Serial.println();
    } else {
        Serial.println("[MODEM] << (no response)");
    }
    return response;
}

bool probeModemBaud() {
    static const unsigned long candidate_bauds[] = {
        115200UL, 9600UL, 57600UL, 38400UL, 19200UL, 230400UL
    };
    Serial.println("[MODEM] Probing UART baud rates (115200, 9600, 57600, 38400, 19200, 230400)...");

    for (unsigned long baud : candidate_bauds) {
        modemSerial.end();
        modemSerial.begin(baud, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
        delay(30);
        for (int attempt = 0; attempt < 2; ++attempt) {
            while (modemSerial.available()) modemSerial.read();
            modemSerial.print("AT\r\n");
            String response;
            const unsigned long started_ms = millis();
            while (millis() - started_ms < 350) {
                while (modemSerial.available()) response += (char)modemSerial.read();
                if (response.indexOf("OK") >= 0) {
                    Serial.printf("[MODEM] A modem answered AT at %lu baud.\n", baud);
                    return true;
                }
                delay(1);
            }
        }
    }
    modemSerial.end();
    modemSerial.begin(MODEM_DEFAULT_BAUD, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);
    Serial.println("[MODEM] No AT response. Check carrier power/startup, identified UART pins, crossed TX/RX, common GND, compatible UART levels, and baud.");
    Serial.println("[MODEM] USB power by itself does not connect the carrier UART to the ESP32.");
    return false;
}

bool runModemDiagnostics() {
    if (!g_modem_lock || xSemaphoreTake(g_modem_lock, 0) != pdTRUE) {
        Serial.println("[MODEM] Busy or unavailable. Retry diagnostics shortly.");
        return false;
    }
    if (!probeModemBaud()) {
        xSemaphoreGive(g_modem_lock);
        return false;
    }

    sendModemCommand("AT+CMEE=2", "OK", 2000);
    sendModemCommand("ATI", "OK", 2000);
    String sim = sendModemCommand("AT+CPIN?", "OK", 2000);
    if (sim.indexOf("+CPIN: READY") < 0) {
        Serial.println("[MODEM] SIM is not reported READY; check SIM seating, PIN lock, and SIM service.");
    }
    sendModemCommand("AT+CSQ", "OK", 2000);
    String registration = sendModemCommand("AT+CEREG?", "OK", 2000);
    if (registration.indexOf(",1") >= 0 || registration.indexOf(",5") >= 0) {
        Serial.println("[MODEM] LTE registration reports home or roaming service.");
    } else {
        Serial.println("[MODEM] LTE registration is not confirmed; inspect the +CEREG response and antenna/SIM/network coverage.");
    }
    sendModemCommand("AT+CMGF?", "OK", 2000);
    xSemaphoreGive(g_modem_lock);
    Serial.println("[MODEM] Diagnostics complete. Type test_sms to send one SMS to the configured recipient.");
    return true;
}

void sendModemTestSMS() {
    if (!g_owner_active.load()) { Serial.println("[Owner] Activate the device before sending SMS tests."); return; }
    Serial.println("[MODEM] Running status checks before the test SMS...");
    if (!runModemDiagnostics()) {
        Serial.println("[MODEM] Test SMS skipped because the modem check did not complete.");
        return;
    }
    const String body = String("PREVENTA A7670C test: UART/SMS check at ") + getTimestampString();
    if (trySendSMS(testSmsRecipient(), body.c_str())) {
        Serial.println("[MODEM] Test SMS accepted by the carrier. Confirm receipt on the phone.");
    } else {
        Serial.println("[MODEM] Test SMS failed. Use the AT responses above to distinguish SIM, registration, SMS configuration, or signal problems.");
    }
}

bool trySendSMS(const char* number, const char* message) {
    if (!g_modem_lock || xSemaphoreTake(g_modem_lock, 0) != pdTRUE) return false;
    const bool sent = sendSmsOnModem(number, message);
    xSemaphoreGive(g_modem_lock);
    return sent;
}

bool sendSmsOnModem(const char* number, const char* message) {
    if (!g_owner_active.load() || !validSmsNumber(number)) return false;
    if (!probeModemBaud()) return false;
    if (sendModemCommand("AT", "OK", 1500).indexOf("OK") < 0) {
        Serial.println("[MODEM] AT handshake failed before SMS.");
        return false;
    }
    if (sendModemCommand("AT+CMGF=1", "OK", 2000).indexOf("OK") < 0) {
        Serial.println("[MODEM] Could not set SMS text mode.");
        return false;
    }

    String cmgs = String("AT+CMGS=\"") + number + "\"";
    if (sendModemCommand(cmgs, ">", 10000).indexOf(">") < 0) {
        Serial.println("[MODEM] No SMS prompt from AT+CMGS; check the SIM, registration and SMS service.");
        return false;
    }

    modemSerial.print(message);
    modemSerial.write(26);
    Serial.println("[MODEM] >> <SMS text> + Ctrl-Z");
    String result;
    const unsigned long started_ms = millis();
    unsigned long last_byte_ms = started_ms;
    while (millis() - started_ms < 60000UL) {
        while (modemSerial.available()) {
            result += (char)modemSerial.read();
            last_byte_ms = millis();
        }
        if (result.indexOf("+CMGS:") >= 0 && result.indexOf("OK") >= 0 && millis() - last_byte_ms >= 150) break;
        if (result.indexOf("+CMS ERROR") >= 0 || result.indexOf("ERROR") >= 0) break;
        delay(1);
    }
    if (result.length()) {
        Serial.print("[MODEM] << ");
        Serial.print(result);
        if (!result.endsWith("\n")) Serial.println();
    } else {
        Serial.println("[MODEM] << (no SMS submission response within 60 seconds)");
    }
    if (result.indexOf("+CMGS:") < 0 || result.indexOf("OK") < 0) return false;

    Serial.println("[MODEM] SMS submission accepted by the network.");
    return true;
}

bool queueSmsAlert(int final_tier) {
    const char* recipient = smsRecipient();
    if (!g_sms_alerts || !validSmsNumber(recipient)) return false;
    SmsAlert alert = {};
    memcpy(alert.recipient, recipient, strlen(recipient) + 1);
    if (!g_owner_active.load()) return false;
    strlcpy(alert.generation, g_remote_settings.activationGeneration, sizeof(alert.generation));
    preventa::formatOwnerSms(alert.message, sizeof(alert.message), final_tier,
                            g_remote_settings.location, g_device_id, false);
    // Keep the newest alert waiting if the modem is already sending a message.
    return xQueueOverwrite(g_sms_alerts, &alert) == pdPASS;
}

void deliverSmsAlerts(void*) {
    for (;;) {
        SmsAlert alert;
        if (xQueueReceive(g_sms_alerts, &alert, portMAX_DELAY) != pdTRUE) continue;
        if (xSemaphoreTake(g_modem_lock, portMAX_DELAY) == pdTRUE) {
            RemoteSettings current;
            copyOwnerSettings(current);
            const bool same_owner = g_owner_active.load()
                && strcmp(alert.generation, current.activationGeneration) == 0;
            const bool sent = same_owner && sendSmsOnModem(alert.recipient, alert.message);
            xSemaphoreGive(g_modem_lock);
            if (!sent) Serial.println("[MODEM] Automatic SMS submission failed.");
        }
    }
}

const char* setupResultName(SetupResult result) {
    switch (result) {
        case SetupResult::Pending: return "pending";
        case SetupResult::Accepted: return "accepted";
        case SetupResult::Failed: return "failed";
        case SetupResult::Offline: return "offline";
        case SetupResult::NoBrowser: return "no_browser";
        case SetupResult::RateLimited: return "rate_limited";
        case SetupResult::Idle: return "idle";
        case SetupResult::NotActivated: return "not_activated";
    }
    return "failed";
}

SetupResult sendSetupTestPush(int tier) {
    if (!g_owner_active.load()) return SetupResult::NotActivated;
    if (WiFi.status() != WL_CONNECTED || g_backend_setup_push_url.isEmpty()) return SetupResult::Offline;
    WiFiClientSecure client;
    HTTPClient http;
    if (!beginBackendRequest(http, client, g_backend_setup_push_url)) return SetupResult::Failed;
    http.setTimeout(20000);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("X-Device-Id", g_device_id);
    http.addHeader("X-Device-Key", g_device_key);
    const int code = http.POST("{\"tier\":" + String(tier) + "}");
    http.end();
    switch (code) {
        case 200: return SetupResult::Accepted;
        case 409: return SetupResult::NoBrowser;
        case 429: return SetupResult::RateLimited;
        default: return SetupResult::Failed;
    }
}

void sendSetupTestAlerts(void* parameter) {
    if (!g_owner_active.load()) {
        g_setup_push_result.store(SetupResult::NotActivated);
        g_setup_sms_result.store(SetupResult::NotActivated);
        g_setup_alert_running.store(false);
        vTaskDelete(nullptr);
        return;
    }
    const int tier = static_cast<int>(reinterpret_cast<uintptr_t>(parameter));
    RemoteSettings owner;
    copyOwnerSettings(owner);
    char recipient[17];
    strlcpy(recipient, owner.smsRecipient, sizeof(recipient));
    g_setup_push_result.store(sendSetupTestPush(tier));

    char message[161];
    preventa::formatOwnerSms(message, sizeof(message), tier, owner.location, g_device_id, true);
    const bool accepted = validSmsNumber(recipient) && trySendSMS(recipient, message);
    g_setup_sms_result.store(accepted ? SetupResult::Accepted : SetupResult::Failed);
    g_setup_alert_running.store(false);
    vTaskDelete(nullptr);
}

// ============================================================================
// ── CALIBRATION & BASELINE ROUTINES ─────────────────────────────────────────
// ============================================================================
void setDefaultBaseline() {
    g_baseline_ppm          = DEFAULT_BASELINE_PPM;
    g_baseline_temp         = DEFAULT_BASELINE_TEMP;
    g_baseline_hum          = DEFAULT_BASELINE_HUM;
    g_baseline_current      = DEFAULT_BASELINE_CURRENT;
    g_mq7_zero_ppm          = DEFAULT_BASELINE_PPM;

    g_dev_ppm     = 0.0f;
    g_dev_temp    = 0.0f;
    g_dev_hum     = 0.0f;
    g_dev_current = 0.0f;

    g_baseline_sample_count = BASELINE_SAMPLES;
    g_warmup_active         = false;
    g_previous_severe_ror = false;
    g_previous_severe_mq7_deviation = false;
    g_previous_acs_delta_direction = 0;
}

bool validAcsZero(float zero) {
    return isfinite(zero) && zero >= 1.5f && zero <= 3.5f;
}

AcsZeroResult sampleAcsZero(float& zero) {
    constexpr int sample_count = 160;
    constexpr float max_idle_spread = 0.5f * ACS712_SENSITIVITY;
    float sum = 0.0f;
    float minimum = 5.0f;
    float maximum = 0.0f;
    for (int i = 0; i < sample_count; i++) {
        const float voltage = analogReadMilliVolts(ACS712_ADC_PIN) * (ACS712_DIVIDER_COMPENSATION / 1000.0f);
        sum += voltage;
        minimum = min(minimum, voltage);
        maximum = max(maximum, voltage);
        delay(3);
    }
    zero = sum / sample_count;
    if (!validAcsZero(zero)) return AcsZeroResult::OutOfRange;
    if (maximum - minimum > max_idle_spread) return AcsZeroResult::Unstable;
    return AcsZeroResult::Ok;
}

void initializeAcsZero() {
    if (preferences.begin("preventa-acs", true)) {
        const float saved = preferences.getFloat("zero_v", NAN);
        preferences.end();
        if (validAcsZero(saved)) {
            ACS712_VZERO = saved;
            Serial.printf("[ACS712] Saved zero point: %.4f V\n", ACS712_VZERO);
            return;
        }
    }

    float measured;
    if (sampleAcsZero(measured) == AcsZeroResult::Ok) ACS712_VZERO = measured;
    Serial.printf("[ACS712] Provisional zero point: %.4f V. Use the setup portal to save calibration.\n", ACS712_VZERO);
}

AcsZeroResult calibrateAcsZero() {
    float measured;
    const AcsZeroResult result = sampleAcsZero(measured);
    if (result != AcsZeroResult::Ok) return result;
    if (!preferences.begin("preventa-acs", false)) return AcsZeroResult::SaveFailed;
    const size_t written = preferences.putFloat("zero_v", measured);
    preferences.end();
    if (written != sizeof(float)) return AcsZeroResult::SaveFailed;

    ACS712_VZERO = measured;
    g_prev_smoothed_current = -999.0f;
    g_baseline_current = DEFAULT_BASELINE_CURRENT;
    g_dev_current = 0.0f;
    g_persist.acs712 = {0, 0};
    g_previous_acs_delta_direction = 0;
    Serial.printf("[ACS712] Saved zero point: %.4f V\n", ACS712_VZERO);
    return AcsZeroResult::Ok;
}

void runCalibration() {
    g_baseline_sample_count = 0;
    g_warmup_active         = true;

    g_prev_smoothed_ppm     = -1.0f;
    g_prev_smoothed_current = -999.0f;
    g_ror_count             = 0;
    g_previous_severe_ror   = false;
    g_previous_severe_mq7_deviation = false;
    g_previous_acs_delta_direction = 0;

    g_camera_warming        = 0;
    g_camera_state_prev     = 0;
    g_last_tier             = 0;
    g_at_tier_since         = 0;
    g_improving_samples     = 0;
    g_current_tier_est      = 0;

    g_sms_peak_tier    = 0;
    g_post_alarm_ctr   = 0;
    g_peak_tier        = 0;
    g_log_last_tier    = 0;
    g_log_entry_count  = 0;
    g_event_active     = false;

    g_persist = { 0, 0.0f, {0,0}, {0,0}, {0,0} };

    g_mq7_zero_ppm   = 100.0f;
    g_mq7_zero_volts = 2.3f;
}

// ============================================================================
// ── DEBUG OUTPUT & HELPERS ──────────────────────────────────────────────────
// ============================================================================
void printDebugSummary(SmoothedData& s, SensorStates& st, FusionData& f,
                        int final_tier, bool backend_ok) {
    Serial.printf(
        "[%s] T:%d(%s)  CO:%.0fppm(%s)  Temp:%.1fC(%s)  RH:%.0f%%  I:%.2fA(%s)  "
        "Cam:%s(%d%%)  Score:%d  Fire:%s  Cloud:%s\n",
        getTimestampString().c_str(),
        final_tier, TIER_NAMES[final_tier],
        s.mq7_ppm, STATE_LABELS[st.mq7],
        s.temperature, STATE_LABELS[st.temperature],
        s.humidity,
        s.acs_current, STATE_LABELS[st.acs712],
        STATE_LABELS[st.camera], g_cam_latest_fire_pct,
        f.score, FIRE_TYPES[f.fire_type],
        backend_ok ? "OK" : (g_portal_active ? "PORTAL" : (WiFi.status() == WL_CONNECTED ? "BACKOFF" : "OFFLINE"))
    );
}

float calcDeviation(float current_val, float base_val) {
    float safe_base = (fabsf(base_val) < EPSILON) ? EPSILON : base_val;
    return ((current_val - safe_base) / fabsf(safe_base)) * 100.0f;
}

float noiseSimulate(float amplitude) {
    return ((float)random(0, 10001) / 10000.0f - 0.5f) * amplitude;
}

// ============================================================================
// ── MANUAL DIAGNOSTIC TEST (BUTTON GPIO33) ──────────────────────────────────
// ============================================================================
void toggleStandbyMode() {
    g_alarm_hush.clear();
    g_standby_active = !g_standby_active;
    if (g_standby_active) {
        g_standby_start_ms = millis();
        Serial.println("\n========================================================");
        Serial.println("[POWER] Device entered Standby / Sleep (Single click to turn ON).");
        Serial.println("========================================================\n");
        // Extinguish status and alert LEDs
        digitalWrite(POWER_LED_PIN, LOW);
        digitalWrite(SYSTEM_LED_PIN, LOW);
        digitalWrite(NETWORK_LED_PIN, LOW);
        updateLEDs(0);
        updateBuzzer(false);
        updateRelay();
        camSerial.println("STANDBY:1");
    } else {
        Serial.println("\n========================================================");
        Serial.println("[POWER] Device woke up — active fire monitoring resumed.");
        Serial.println("========================================================\n");
        applyCameraSetting();
        digitalWrite(POWER_LED_PIN, HIGH);
        digitalWrite(SYSTEM_LED_PIN, HIGH);
        if (WiFi.status() == WL_CONNECTED) digitalWrite(NETWORK_LED_PIN, HIGH);
        g_last_sample_ms = millis();
    }
}

void handleDiagnosticButton() {
    static bool was_pressed = false;
    static unsigned long press_start_ms = 0;
    static bool hold_3s_alerted = false;

    bool is_pressed = (digitalRead(DIAG_BUTTON_PIN) == LOW);

    if (is_pressed && !was_pressed) {
        // Button was just pressed down
        was_pressed = true;
        press_start_ms = millis();
        hold_3s_alerted = false;
    } else if (is_pressed && was_pressed) {
        // Button is being held
        unsigned long duration = millis() - press_start_ms;
        if (duration >= 3000 && !hold_3s_alerted) {
            hold_3s_alerted = true;
            // Quick visual feedback: double flash System LED to indicate Setup Mode threshold reached
            digitalWrite(SYSTEM_LED_PIN, LOW);
            delay(50);
            digitalWrite(SYSTEM_LED_PIN, HIGH);
            delay(50);
            digitalWrite(SYSTEM_LED_PIN, LOW);
            Serial.println("[Button] Held >= 3.0s (Release now to enter/exit Setup Mode)");
        }
    } else if (!is_pressed && was_pressed) {
        // Button was released
        was_pressed = false;
        unsigned long duration = millis() - press_start_ms;

        if (duration >= 80 && duration < 2000) {
            const int tier = effectiveAlertTier();
            if (preventa::shortPressAction(g_standby_active, tier)
                    == preventa::ShortPressAction::HushBuzzer) {
                g_alarm_hush.acknowledge(tier, millis());
                updateBuzzer(tier >= 4);
                Serial.printf("[Button] Tier %d acknowledged: buzzer hushed for 120s. "
                              "Monitoring, messages and relay protection remain active. "
                              "A higher tier cancels the hush.\n", tier);
            } else {
                toggleStandbyMode();
            }
        } else if (duration >= 3000) {
            // Long Press (>= 3.0s): Toggle Setup Portal (SoftAP)
            toggleSetupPortal();
        }
    }
}

void triggerManualSelfTest() {
    Serial.println("\n========================================================");
    Serial.println("[DIAG] MANUAL DIAGNOSTIC TEST TRIGGERED (BUTTON GPIO33)");
    Serial.println("========================================================");

    // 1. Audio / Visual Sweep Verification
    int testPins[] = {LED_TIER1_PIN, LED_TIER2_PIN, LED_TIER3_PIN, LED_TIER4_PIN, LED_TIER5_PIN};
    for (int i = 0; i < 5; i++) {
        digitalWrite(testPins[i], HIGH);
        digitalWrite(BUZZER_PIN, HIGH);
        delay(120);
        digitalWrite(testPins[i], LOW);
        digitalWrite(BUZZER_PIN, LOW);
        delay(40);
    }

    // Status LEDs check
    digitalWrite(POWER_LED_PIN, HIGH);
    digitalWrite(SYSTEM_LED_PIN, HIGH);
    digitalWrite(NETWORK_LED_PIN, HIGH);
    delay(250);
    digitalWrite(NETWORK_LED_PIN, (WiFi.status() == WL_CONNECTED) ? HIGH : LOW);

    // 2. Sensor Scan
    bool sensors_ok = selfDiagnose();

    // 3. 4G LTE Test SMS Dispatch
    Serial.println("[DIAG] Testing 4G LTE Cellular Modem (A7670C)...");
    char testMsg[160];
    snprintf(testMsg, sizeof(testMsg), "PREVENTA SELF-TEST: Sensors: %s, Time: %s",
             sensors_ok ? "PASSED" : "FLAGGED", getTimestampString().c_str());
    if (trySendSMS(testSmsRecipient(), testMsg)) {
        Serial.println("[DIAG] ✓ Test SMS successfully transmitted over 4G LTE.");
    } else {
        Serial.println("[DIAG] ⚠ 4G LTE test SMS dispatch failed (check SIM/signal/power).");
    }
    Serial.println("[DIAG] === Manual diagnostic test complete ===\n");
}
