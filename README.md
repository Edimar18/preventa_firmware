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
2. Set the pre-provisioned device ID and matching device API key, plus a unique setup AP SSID/password. Leave fallback Wi-Fi and SMS fields blank for first-owner onboarding. The owner supplies the recipient through secure setup.
3. Leave `secrets.h` untracked; it is ignored by Git. The example file also permits compilation without credentials. Such a build must be configured for your device before normal cloud use.
4. Review `DEFAULT_BACKEND_HOST` and `BACKEND_PORT` in the main sketch for readings/capture uploads. Remote configuration and setup-test push use the certificate-validated origin in `remote_config.h`.
5. Device identity comes from `secrets.h`. Changing that provisioned identity clears stale Wi-Fi and owner cache once. For the same identity, saved Wi-Fi and activated owner configuration survive upgrades/reboots. Use the AP SSID/password printed on the private setup card, then open `http://192.168.4.1`; hold the GPIO33 button for at least 3 seconds and release to open or close it.

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

## First-owner activation (2026-11-27)

The private per-unit `secrets.h` includes a unique device ID/API key and setup AP SSID/password. The separate activation QR contains a one-time claim code, never the permanent device API key. Keep the private provisioning card/JSON out of Git. Blank fallback Wi-Fi and recipient fields are intentional.

Before activation the normal local sensor pipeline, LEDs, buzzer, camera previews and relay rules remain available. Every SMS path and cloud reading/capture upload is gated until activation. Open the protected AP, note its six-digit pairing code BEFORE saving home Wi-Fi (saving closes the AP), then configure home Wi-Fi. Return the phone to an internet connection, open the activation QR, create/sign in to the owner account and supply room/location and household contact. The device fetches, saves and acknowledges activation; the AP/HTTPS page shows the resulting state.

After activation, the saved configuration permits SMS/local monitoring through a Wi-Fi outage. Explicit server authentication refusal disables cloud/SMS authorization; an ordinary network timeout does not erase an already verified activation. A Wi-Fi reset preserves owner configuration. Flashing a different provisioned device identity clears stale Wi-Fi and owner cache once; firmware upgrades with the same identity keep them. Device identity comes from the provisioned firmware rather than old NVS credential overrides.

SMS includes the alert tier, installed room and hardware ID. AP test messages remain clearly marked TEST. Reopening setup on an unclaimed device generates a fresh pairing code. Room changes are fetched at the normal settings interval after activation. The current release does not implement owner transfer; clearing Wi-Fi is not an ownership-release operation.

## AP camera preview and sensor reset (2026-11-27)

Flash both updated boards together. AP preview now uses `PREVIEW:<request_id>` and returns a matching 96x96 BMP, prediction scores and a checked end marker. It does not wait for an extra full-frame JPEG. Normal live/autonomous JPEG capture uploads remain supported. The model, preprocessing and tier thresholds are unchanged.

The main UART RX buffer is 32 KiB. A request has a fixed 45-second deadline and a binary transfer fails after 10 seconds without progress. Text framing is bounded, and interrupted or stale replies cannot complete a different preview. Browser requests time out rather than keeping the capture button locked. Sensor faults/null values do not break preview telemetry.

The AP button **Recalibrate ESP32-CAM** restarts the sensor, reapplies its existing settings and settles auto exposure/gain/white balance. It does not retrain or replace the model. Clear a Tier 3+ test or resolve a real warning before recalibration. These local controls work before owner activation and without home Wi-Fi.

Idle link checks repeat every 10 seconds. The AP distinguishes no valid UART reply from camera sensor/model failure and shows received-byte counts. A flash proves that the camera received a command; it does not prove that a reply reached the main board. If received bytes stay at zero, remove power before checking CAM GPIO1/TX to main GPIO21/RX, main GPIO22/TX to CAM GPIO3/RX, and common ground.

Validation: eight host tests plus AP JavaScript regression checks pass; both board builds compile with ESP32 core 2.0.17. Physical capture still requires checking on the actual boards.
