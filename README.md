# PREVENTA firmware

Firmware for the ESP32 main controller and ESP32-CAM. This repository contains the current source used in the PREVENTA project, including fresh camera capture, the exact 96 × 96 model-input preview, AP diagnostics and timed alarm acknowledgement.

## Sketches

| Sketch | Board / build configuration | Purpose |
| --- | --- | --- |
| `ESP32-MAIN-FINAL/PREVENTA_ESP32_Main` | ESP32 Dev Module; ESP32 Arduino core 2.0.17; Huge APP partition | Sensors, tier decisions, relay, buzzer, SMS, backend communication and setup portal |
| `ESP32-CAM-FINAL/PREVENTA_ESP32_CAM` | AI Thinker ESP32-CAM; ESP32 Arduino core 2.0.17; PSRAM enabled | Camera acquisition, on-device model inference and UART image/prediction transfer |

Open the `.ino` with the same name as its enclosing sketch folder. Both sketches must be flashed to their respective boards to use the complete camera preview flow. No binary or private device configuration is included.

## Private configuration

1. Copy `ESP32-MAIN-FINAL/PREVENTA_ESP32_Main/secrets.example.h` to `secrets.h` in the same folder.
2. Set the Wi-Fi credentials, registered device ID, matching device API key, and fallback SMS recipient in E.164 format.
3. Leave `secrets.h` untracked; it is ignored by Git. The example file also permits compilation without credentials. Such a build must be configured for your device before normal cloud use.
4. Review `DEFAULT_BACKEND_HOST` and `BACKEND_PORT` in the main sketch for readings/capture uploads. Remote configuration and setup-test push use the certificate-validated origin in `remote_config.h`.
5. Existing Wi-Fi/device configuration saved in NVS takes precedence over fallback values. The AP is `PREVENTA-Setup` at `http://192.168.4.1`; hold the GPIO33 button for at least 3 seconds and release to open or close it.

The published source contains no provisioned Wi-Fi password, device API key, personal SMS fallback number, Firebase service account or VPS key. Keep the production backend environment and database separately.

## Physical button (GPIO33)

| Action | Behavior |
| --- | --- |
| Short press and release, 80 ms to under 2 seconds, below Tier 3 | Toggle existing software standby / resume |
| Short press and release at Tier 3, 4 or 5 | Acknowledge and hush the local buzzer for 120 seconds; monitoring remains active |
| Short press while in standby | Resume monitoring |
| Hold at least 3 seconds, then release | Toggle the setup AP |

A rise above the acknowledged tier cancels the hush immediately when processed. Expiry restores the buzzer if the current tier requires it. Recovery below Tier 3 clears the acknowledgement; a new warning starts unacknowledged. Another short press restarts the 120-second timer. Clearing or ending an AP tier test also clears its hush.

Normal alert thresholds remain: SMS eligibility starts at Tier 3; the buzzer starts at Tier 4. Hushing Tier 3 therefore acknowledges the warning, and escalation to Tier 4 sounds the buzzer. Hush does not disable sensing, tier decisions, LEDs, SMS eligibility, backend push processing or relay protection. AP telemetry displays hush status and remaining seconds. The normal readings `siren` value reflects the audible output at the reading time.

Standby is the existing software mode: monitoring pauses, the main MCU remains powered, and the existing 45-minute watchdog resumes monitoring. It is not deep sleep or a physical power switch.

## Libraries and builds

Validated locally with Arduino CLI 1.2.0, ESP32 Arduino core 2.0.17, ArduinoJson 7.4.3, DHT sensor library 1.4.7, Adafruit Unified Sensor 1.1.15 and TensorFlowLite_ESP32 1.0.0. Camera and networking headers come from the ESP32 core.

```sh
arduino-cli compile --fqbn esp32:esp32:esp32:PartitionScheme=huge_app ESP32-MAIN-FINAL/PREVENTA_ESP32_Main
arduino-cli compile --fqbn esp32:esp32:esp32cam ESP32-CAM-FINAL/PREVENTA_ESP32_CAM
```

The committed `model.h` is the existing model artifact; this update does not retrain it or change model weights. The camera sends the center 96 × 96 input for the prediction. AP preview only shows that actual crop, never a substitute full-frame image.

## Regression checks

Python 3, g++ and Node.js are required for the host checks:

```sh
python3 -m unittest discover -s firmware_tests -p 'test_*.py' -v
node firmware_tests/ap_capture_preview_test.cjs
```

These exercise actual firmware helpers and the embedded AP script: hush timing/escalation/rollover, button policy, output state, fresh camera frame selection, delayed responses, duplicate capture requests, crop/score pairing and missing-preview handling. Compilation and host tests do not replace checking the physical buzzer/button on the device.
