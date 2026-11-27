/**
 * ============================================================================
 * PREVENTA — ESP32-CAM Fire Confirmation Module
 * AI-Thinker board + OV2640 sensor, with a legacy RGB565 fallback
 * ============================================================================
 *
 * WIRED UART CO-PROCESSOR (no WiFi, no WebServer).
 *
 * This module is a dedicated camera/AI inference co-processor. It communicates
 * with the main PREVENTA ESP32 controller over UART at 115200 baud.
 *
 * Wiring:
 *   Main TX (GPIO22)  →  CAM RX (GPIO3 / U0RXD)
 *   Main RX (GPIO21)  ←  CAM TX (GPIO1 / U0TXD)
 *   GND               ↔  GND
 *   5V Power          ←  5V Rail
 *
 * Operating Modes:
 *   1. On-demand inference & image stream (PREDICT / CAPTURE / SNAP):
 *      Main sends "PREDICT" -> CAM captures, runs the embedded TFLite model, and replies:
 *          PREDICT:<fire_pct>,<no_fire_pct>
 *          MODEL_INPUT:<byte_count> and BMP bytes when requested by MODEL_INPUT_ONCE
 *          IMG:<byte_count>
 *          <raw binary JPEG bytes>
 *
 *   2. Link Handshake:
 *      Main sends "PING" -> CAM immediately replies "PONG"
 *
 *   3. 10-minute autonomous watchdog:
 *      Captures and transmits image + confidence to Main for backend storage:
 *          FIRE:<confidence_pct>
 *          IMG:<byte_count>
 *          <raw binary JPEG bytes>
 * ============================================================================
 */

#include "esp_camera.h"
#include "esp_timer.h"
#include "esp_jpg_decode.h"
#include "img_converters.h"

// TensorFlow Lite for Microcontrollers Includes
#include <TensorFlowLite_ESP32.h>
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"

// Generated model array
#include "model.h"
#include "camera_transport.h"

#define CAMERA_MODEL_AI_THINKER

// AI Thinker Pin Definitions
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// AI-Thinker built-in white flash LED on GPIO4
#define FLASH_GPIO_NUM     4
const bool ENABLE_FLASH_ASSIST = true;

// 10-minute independent watchdog
const unsigned long AUTO_CAPTURE_INTERVAL_MS = 10UL * 60UL * 1000UL;

// TFLite Globals
const tflite::Model* tflite_model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;
TfLiteTensor* output = nullptr;
tflite::ErrorReporter* error_reporter = nullptr;

// Tensor Arena: 500 KB in PSRAM
constexpr int kTensorArenaSize = 500 * 1024;
uint8_t* tensor_arena = nullptr;

constexpr int kFrameWidth = 320;
constexpr int kFrameHeight = 240;
constexpr int kInputSize = 96;
constexpr int kCropLeft = (kFrameWidth - kInputSize) / 2;
constexpr int kCropTop = (kFrameHeight - kInputSize) / 2;
constexpr size_t kMaxPreviewBytes = 96 * 1024; // ESP32 Main receiver limit.
constexpr size_t kModelInputRowBytes = kInputSize * 3;
constexpr size_t kModelInputBmpHeaderBytes = 54;
constexpr size_t kModelInputBmpBytes = kModelInputBmpHeaderBytes + kModelInputRowBytes * kInputSize;
static_assert(kModelInputRowBytes % 4 == 0, "BMP rows must align to 4 bytes.");

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,
    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QVGA,
    .jpeg_quality = 12,
    .fb_count = 1,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

unsigned long g_last_auto_capture_ms = 0;
bool g_cam_initialized = false;
bool g_camera_sensor_ready = false;
bool g_inference_running = false;
bool g_standby_mode = false;

// Non-blocking UART accumulator
char g_uart_rx_buf[64];
int  g_uart_rx_idx = 0;

// ============================================================================
// Flash Assist
// ============================================================================
void flashPulseOn() {
    if (!ENABLE_FLASH_ASSIST) return;
    pinMode(FLASH_GPIO_NUM, OUTPUT);
    digitalWrite(FLASH_GPIO_NUM, HIGH);
    delay(40);
}

void flashPulseOff() {
    if (!ENABLE_FLASH_ASSIST) return;
    digitalWrite(FLASH_GPIO_NUM, LOW);
}

camera_fb_t* captureFreshFrame() {
    // WHEN_EMPTY may retain a frame from the previous request, even with one
    // buffer. The driver timestamps frame start using esp_timer_get_time().
    // Only infer/display a frame whose acquisition started after this request.
    const int64_t requestedAtUs = esp_timer_get_time();
    constexpr int kMaxFreshFrameAttempts = 4;
    for (int attempt = 0; attempt < kMaxFreshFrameAttempts; ++attempt) {
        camera_fb_t* frame = esp_camera_fb_get();
        if (!frame) return nullptr;
        const int64_t startedAtUs = int64_t(frame->timestamp.tv_sec) * 1000000LL
                                  + frame->timestamp.tv_usec;
        if (startedAtUs >= requestedAtUs) return frame;
        esp_camera_fb_return(frame);
        delay(1); // Let the driver begin another frame using the released buffer.
    }
    return nullptr; // Fail rather than label an old frame as a new capture.
}

// ============================================================================
// TFLite Inference
// ============================================================================
void writeInputPixel(size_t offset, uint8_t red, uint8_t green, uint8_t blue) {
    if (input->type == kTfLiteInt8) {
        input->data.int8[offset] = static_cast<int8_t>(red - 128);
        input->data.int8[offset + 1] = static_cast<int8_t>(green - 128);
        input->data.int8[offset + 2] = static_cast<int8_t>(blue - 128);
    } else {
        input->data.f[offset] = red;
        input->data.f[offset + 1] = green;
        input->data.f[offset + 2] = blue;
    }
}

struct JpegCrop {
    const camera_fb_t* frame;
    bool dimensionsOk;
    size_t pixelsWritten;
};

size_t readJpeg(void* context, size_t index, uint8_t* outputBytes, size_t length) {
    const camera_fb_t* frame = static_cast<JpegCrop*>(context)->frame;
    if (index > frame->len || length > frame->len - index) return 0;
    if (outputBytes) memcpy(outputBytes, frame->buf + index, length);
    return length;
}

bool writeJpegCrop(void* context, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint8_t* rgb) {
    auto& crop = *static_cast<JpegCrop*>(context);
    if (!rgb) {
        if (x == 0 && y == 0) crop.dimensionsOk = width == kFrameWidth && height == kFrameHeight;
        return crop.dimensionsOk;
    }
    if (!crop.dimensionsOk || x + width > kFrameWidth || y + height > kFrameHeight) return false;

    // The SDK decoder callback passes RGB888 blocks. Only intersecting pixels
    // are copied, so there is no full-frame RGB buffer or BMP channel swap.
    const int x0 = max(int(x), kCropLeft);
    const int x1 = min(int(x) + width, kCropLeft + kInputSize);
    const int y0 = max(int(y), kCropTop);
    const int y1 = min(int(y) + height, kCropTop + kInputSize);
    for (int row = y0; row < y1; ++row) {
        for (int col = x0; col < x1; ++col) {
            const size_t source = ((row - y) * width + (col - x)) * 3;
            const size_t target = ((row - kCropTop) * kInputSize + (col - kCropLeft)) * 3;
            writeInputPixel(target, rgb[source], rgb[source + 1], rgb[source + 2]);
            ++crop.pixelsWritten;
        }
    }
    return true;
}

bool prepareInput(camera_fb_t* frame) {
    if (!frame->buf || frame->width != kFrameWidth || frame->height != kFrameHeight) return false;
    if (frame->format == PIXFORMAT_JPEG) {
        if (frame->len < 4) return false;
        JpegCrop crop = {frame, false, 0};
        return esp_jpg_decode(frame->len, JPG_SCALE_NONE, readJpeg, writeJpegCrop, &crop) == ESP_OK
            && crop.dimensionsOk && crop.pixelsWritten == kInputSize * kInputSize;
    }
    if (frame->format != PIXFORMAT_RGB565 || frame->len < kFrameWidth * kFrameHeight * 2) return false;

    for (int row = 0; row < kInputSize; ++row) {
        for (int col = 0; col < kInputSize; ++col) {
            const size_t offset = ((row + kCropTop) * kFrameWidth + (col + kCropLeft)) * 2;
            const uint16_t packed = (uint16_t(frame->buf[offset]) << 8) | frame->buf[offset + 1];
            writeInputPixel((row * kInputSize + col) * 3,
                            (packed & 0xF800) >> 8, (packed & 0x07E0) >> 3, (packed & 0x001F) << 3);
        }
    }
    return true;
}

void writeBmpNumber(uint8_t* target, uint32_t value) {
    for (int i = 0; i < 4; ++i) target[i] = static_cast<uint8_t>(value >> (i * 8));
}

void snapshotModelInput(uint8_t* bmp) {
    memset(bmp, 0, kModelInputBmpHeaderBytes);
    bmp[0] = 'B';
    bmp[1] = 'M';
    writeBmpNumber(bmp + 2, kModelInputBmpBytes);
    writeBmpNumber(bmp + 10, kModelInputBmpHeaderBytes);
    writeBmpNumber(bmp + 14, 40);
    writeBmpNumber(bmp + 18, kInputSize);
    writeBmpNumber(bmp + 22, kInputSize);
    bmp[26] = 1; // Color planes.
    bmp[28] = 24; // Bits per pixel.
    writeBmpNumber(bmp + 34, kModelInputBmpBytes - kModelInputBmpHeaderBytes);

    const bool quantized = input->type == kTfLiteInt8;
    uint8_t* pixels = bmp + kModelInputBmpHeaderBytes;
    for (int row = kInputSize - 1; row >= 0; --row) {
        for (int col = 0; col < kInputSize; ++col) {
            const size_t offset = (row * kInputSize + col) * 3;
            for (int channel = 2; channel >= 0; --channel) {
                const size_t index = offset + channel;
                // The tensor stores RGB. BMP writes BGR and rows bottom-first.
                *pixels++ = quantized ? static_cast<uint8_t>(input->data.int8[index] + 128)
                                      : static_cast<uint8_t>(input->data.f[index]);
            }
        }
    }
}

bool runInference(camera_fb_t* frame, float& fire, float& noFire, uint8_t* modelInputPreview) {
    if (!prepareInput(frame)) return false;
    if (modelInputPreview) snapshotModelInput(modelInputPreview);
    if (interpreter->Invoke() != kTfLiteOk) return false;
    if (output->type == kTfLiteInt8) {
        fire = (output->data.int8[0] - output->params.zero_point) * output->params.scale;
        noFire = (output->data.int8[1] - output->params.zero_point) * output->params.scale;
    } else {
        fire = output->data.f[0];
        noFire = output->data.f[1];
    }
    if (!isfinite(fire) || !isfinite(noFire)) return false;
    fire = constrain(fire, 0.0f, 1.0f);
    noFire = constrain(noFire, 0.0f, 1.0f);
    return true;
}

void streamInference(camera_fb_t* frame, float fire, float noFire, bool autonomous,
                     const uint8_t* modelInputPreview) {
    uint8_t* encoded = nullptr;
    size_t length = frame->len;
    const uint8_t* jpeg = frame->buf;
    if (frame->format != PIXFORMAT_JPEG) {
        length = 0;
        jpeg = frame2jpg(frame, 85, &encoded, &length) ? encoded : nullptr;
    }
    const bool usable = jpeg && length >= 4 && length <= kMaxPreviewBytes
        && jpeg[0] == 0xFF && jpeg[1] == 0xD8 && jpeg[length - 2] == 0xFF && jpeg[length - 1] == 0xD9;
    if (!autonomous) {
        Serial.printf("PREDICT:%d,%d\n", int(fire * 100.0f), int(noFire * 100.0f));
        if (modelInputPreview) {
            Serial.printf("MODEL_INPUT:%u\n", static_cast<unsigned>(kModelInputBmpBytes));
            Serial.write(modelInputPreview, kModelInputBmpBytes);
        }
    } else if (usable) {
        Serial.printf("FIRE:%.1f\n", fire * 100.0f);
    }
    if (usable) {
        Serial.printf("IMG:%u\n", static_cast<unsigned>(length));
        Serial.write(jpeg, length);
    } else if (!autonomous) {
        Serial.println("IMG:0");
    }
    Serial.flush();
    free(encoded);
}

void captureAndSend(bool autonomous, bool includeModelInput) {
    if (!g_cam_initialized || g_inference_running || (autonomous && g_standby_mode)) {
        if (!autonomous) {
            Serial.println("PREDICT_ERROR:camera_unavailable");
            Serial.flush();
        }
        return;
    }
    g_inference_running = true;
    flashPulseOn();
    camera_fb_t* frame = captureFreshFrame();
    flashPulseOff();

    uint8_t* preview = frame && includeModelInput
        ? static_cast<uint8_t*>(ps_malloc(kModelInputBmpBytes)) : nullptr;
    float fire, noFire;
    const bool predicted = frame && runInference(frame, fire, noFire, preview);
    if (predicted) {
        streamInference(frame, fire, noFire, autonomous, preview);
    } else if (!autonomous) {
        Serial.println(frame ? "PREDICT_ERROR:inference" : "PREDICT_ERROR:capture");
        Serial.flush();
    }
    free(preview);
    if (frame) esp_camera_fb_return(frame);
    g_inference_running = false;
}

bool initializeCameraSensor() {
    if (g_camera_sensor_ready) esp_camera_deinit();
    g_camera_sensor_ready = false;
    camera_config.xclk_freq_hz = 20000000;
    camera_config.pixel_format = PIXFORMAT_JPEG;
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        camera_config.xclk_freq_hz = 10000000;
        camera_config.pixel_format = PIXFORMAT_RGB565;
        err = esp_camera_init(&camera_config);
    }
    if (err != ESP_OK) return false;
    g_camera_sensor_ready = true;
    sensor_t* s = esp_camera_sensor_get();
    if (!s) { esp_camera_deinit(); g_camera_sensor_ready = false; return false; }
    // Preserve the established orientation, crop, color tuning and auto controls.
    s->set_vflip(s, 0);
    s->set_hmirror(s, 1);
    s->set_brightness(s, 0);
    s->set_contrast(s, 1);
    s->set_saturation(s, 0);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_wb_mode(s, 0);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_gainceiling(s, (gainceiling_t)2);
    s->set_bpc(s, 1);
    s->set_wpc(s, 1);
    s->set_lenc(s, 1);
    return true;
}

bool settleCameraSensor() {
    const unsigned long started = millis();
    int frames = 0;
    while (frames < 6 && millis() - started < 8000UL) {
        camera_fb_t* frame = esp_camera_fb_get();
        if (!frame) return false;
        esp_camera_fb_return(frame);
        ++frames;
        delay(60);
    }
    return frames == 6;
}

void captureSetupPreview(uint32_t requestId) {
    Serial.printf("\nCAPTURE_BEGIN:%lu\n", static_cast<unsigned long>(requestId));
    if (!g_cam_initialized || g_inference_running) {
        Serial.printf("CAPTURE_ERROR:%lu,camera_unavailable\n", static_cast<unsigned long>(requestId));
        return;
    }
    g_inference_running = true;
    flashPulseOn();
    camera_fb_t* frame = captureFreshFrame();
    flashPulseOff();
    uint8_t* preview = frame ? static_cast<uint8_t*>(ps_malloc(kModelInputBmpBytes)) : nullptr;
    float fire = 0, noFire = 0;
    const bool predicted = frame && preview && runInference(frame, fire, noFire, preview);
    if (predicted) {
        Serial.printf("PREDICT:%d,%d\nMODEL_INPUT:%u\n", int(fire * 100.0f), int(noFire * 100.0f), static_cast<unsigned>(kModelInputBmpBytes));
        Serial.write(preview, kModelInputBmpBytes);
        Serial.printf("\nCAPTURE_END:%lu,%lu\n", static_cast<unsigned long>(requestId),
                      static_cast<unsigned long>(preventa::cameraPreviewChecksum(preview, kModelInputBmpBytes)));
    } else {
        const char* reason = !frame ? "capture" : !preview ? "preview_memory" : "inference";
        Serial.printf("CAPTURE_ERROR:%lu,%s\n", static_cast<unsigned long>(requestId), reason);
    }
    Serial.flush();
    free(preview);
    if (frame) esp_camera_fb_return(frame);
    g_inference_running = false;
    g_last_auto_capture_ms = millis();
}

void recalibrateCameraSensor(uint32_t requestId) {
    Serial.printf("\nCAM_RECALIBRATE_BEGIN:%lu\n", static_cast<unsigned long>(requestId));
    if (!interpreter || !input || !output || g_inference_running) {
        Serial.printf("CAM_RECALIBRATED:%lu,ERROR:model_unavailable_or_busy\n", static_cast<unsigned long>(requestId));
        Serial.flush();
        return;
    }
    g_inference_running = true;
    g_cam_initialized = false;
    flashPulseOff();
    const bool ready = initializeCameraSensor() && settleCameraSensor();
    g_cam_initialized = ready;
    g_inference_running = false;
    g_last_auto_capture_ms = millis();
    Serial.printf("CAM_RECALIBRATED:%lu,%s\n", static_cast<unsigned long>(requestId), ready ? "OK" : "ERROR:sensor_restart_or_frames");
    Serial.flush();
}

bool cameraCommandId(const char* text, uint32_t& value) {
    if (text[0] < '0' || text[0] > '9') return false;
    char* end = nullptr;
    const unsigned long id = strtoul(text, &end, 10);
    if (!end || *end || !id || id > UINT32_MAX) return false;
    value = static_cast<uint32_t>(id);
    return true;
}

// ============================================================================
// Command Dispatcher
// ============================================================================
void handleCommand(const char* cmd) {
    static bool modelInputRequested = false;
    static unsigned long modelInputRequestedAt = 0;
    uint32_t requestId = 0;
    if (strncmp(cmd, "PREVIEW:", 8) == 0 && cameraCommandId(cmd + 8, requestId)) {
        captureSetupPreview(requestId);
    } else if (strncmp(cmd, "CAM_RECALIBRATE:", 16) == 0 && cameraCommandId(cmd + 16, requestId)) {
        recalibrateCameraSensor(requestId);
    } else if (strcmp(cmd, "PING") == 0) {
        Serial.println("PONG");
        Serial.printf("STATUS:%s,PSRAM:%u,STBY:%d\n", g_cam_initialized ? "OK" : "ERR", ESP.getFreePsram(), g_standby_mode ? 1 : 0);
        Serial.flush();
    } else if (strcmp(cmd, "MODEL_INPUT_ONCE") == 0) {
        modelInputRequested = true;
        modelInputRequestedAt = millis();
    } else if (strcmp(cmd, "PREDICT") == 0 || strcmp(cmd, "CAPTURE") == 0 || strcmp(cmd, "SNAP") == 0) {
        const bool preview = modelInputRequested && millis() - modelInputRequestedAt < 15000UL;
        modelInputRequested = false;
        captureAndSend(false, preview);
    } else if (strcmp(cmd, "TEST_CAPTURE") == 0) {
        // Fast test capture to warm up AEC/AGC without running inference
        flashPulseOn();
        camera_fb_t* fb = captureFreshFrame();
        flashPulseOff();
        if (fb) {
            esp_camera_fb_return(fb);
            Serial.println("TEST_CAPTURE:OK");
        } else {
            Serial.println("TEST_CAPTURE:FAIL");
        }
        Serial.flush();
    } else if (strcmp(cmd, "STANDBY:1") == 0) {
        g_standby_mode = true;
        Serial.println("STANDBY_OK");
        Serial.flush();
    } else if (strcmp(cmd, "STANDBY:0") == 0) {
        g_standby_mode = false;
        g_last_auto_capture_ms = millis();
        Serial.println("STANDBY_OK");
        Serial.flush();
    } else if (strcmp(cmd, "STATUS") == 0) {
        Serial.printf("STATUS:%s,PSRAM:%u,STBY:%d\n", g_cam_initialized ? "OK" : "ERR", ESP.getFreePsram(), g_standby_mode ? 1 : 0);
        Serial.flush();
    }
}

// ============================================================================
// Setup
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(200);

    if (ENABLE_FLASH_ASSIST) {
        pinMode(FLASH_GPIO_NUM, OUTPUT);
        digitalWrite(FLASH_GPIO_NUM, LOW);
    }

    if (!initializeCameraSensor() || !settleCameraSensor()) {
        Serial.println("CAM_READY:0");
        return;
    }
    sensor_t* sensor = esp_camera_sensor_get();
    Serial.printf("CAM_SENSOR:0x%04X,FORMAT:%s\n", static_cast<unsigned>(sensor->id.PID),
                  camera_config.pixel_format == PIXFORMAT_JPEG ? "JPEG" : "RGB565");

    // 2. Allocate PSRAM Tensor Arena
    if (psramFound()) {
        tensor_arena = (uint8_t*)ps_malloc(kTensorArenaSize);
    } else {
        Serial.println("CAM_READY:0");
        return;
    }

    if (tensor_arena == nullptr) {
        Serial.println("CAM_READY:0");
        return;
    }

    // 3. Init TFLite
    static tflite::MicroErrorReporter micro_error_reporter;
    error_reporter = &micro_error_reporter;
    tflite_model = tflite::GetModel(g_model);
    if (tflite_model->version() != TFLITE_SCHEMA_VERSION) {
        Serial.println("CAM_READY:0");
        return;
    }

    static tflite::AllOpsResolver resolver;
    interpreter = new tflite::MicroInterpreter(tflite_model, resolver, tensor_arena, kTensorArenaSize, error_reporter);

    if (interpreter->AllocateTensors() != kTfLiteOk) {
        Serial.println("CAM_READY:0");
        return;
    }

    input = interpreter->input(0);
    output = interpreter->output(0);
    if (!input || !output || !input->dims || !output->dims
        || (input->type != kTfLiteInt8 && input->type != kTfLiteFloat32)
        || (output->type != kTfLiteInt8 && output->type != kTfLiteFloat32)
        || input->dims->size != 4 || input->dims->data[0] != 1
        || input->dims->data[1] != kInputSize || input->dims->data[2] != kInputSize || input->dims->data[3] != 3
        || output->dims->size != 2 || output->dims->data[0] != 1 || output->dims->data[1] != 2) {
        Serial.println("CAM_READY:0");
        return;
    }

    g_cam_initialized = true;
    Serial.println("CAM_READY:1");
    Serial.flush();
    g_last_auto_capture_ms = millis();
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
    // 1. Non-blocking UART Command Ingestion
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (g_uart_rx_idx > 0) {
                g_uart_rx_buf[g_uart_rx_idx] = '\0';
                handleCommand(g_uart_rx_buf);
                g_uart_rx_idx = 0;
            }
        } else if (g_uart_rx_idx < (int)sizeof(g_uart_rx_buf) - 1) {
            g_uart_rx_buf[g_uart_rx_idx++] = c;
        }
    }

    // 2. Autonomous Watchdog (suppressed during Standby mode)
    unsigned long now = millis();
    if (!g_standby_mode && (now - g_last_auto_capture_ms >= AUTO_CAPTURE_INTERVAL_MS)) {
        g_last_auto_capture_ms = now;
        captureAndSend(true, false);
    }
}
