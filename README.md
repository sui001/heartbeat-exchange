# Heartbeat Exchange

Two paired devices. Each one reads its wearer's pulse, blinks its own LED in
time with that pulse, and sends the rate to the other device over ESP-NOW. What
arrives from the other device drives a haptic motor, so each person feels the
other's heartbeat while the other feels theirs.

Symmetric: both units run the same sketch, distinguished only by `DEVICE_ID`.

## Hardware

Per node:

| Part | Role |
|---|---|
| ESP32-S3 SuperMini | MCU, onboard WS2812B RGB LED |
| MAX30102 | pulse sensor (SparkFun MAX3010x breakout) |
| DRV2605L | haptic driver |
| LRA or ERM motor | the thing you actually feel, on DRV2605L OUT+/OUT- |

### Wiring

Identical on both units. The MAX30102 and DRV2605L share one I2C bus.

| Signal | Pin |
|---|---|
| MAX30102 SDA | GPIO8 |
| MAX30102 SCL | GPIO9 |
| DRV2605L SDA | GPIO8 (shared) |
| DRV2605L SCL | GPIO9 (shared) |
| Both VIN | 3.3V |
| Both GND | GND |
| RGB LED | GPIO47, onboard, no wiring |

## Libraries

- SparkFun MAX3010x Pulse and Proximity Sensor Library (provides `MAX30105.h`
  and `heartRate.h`)
- Adafruit DRV2605 Library
- FastLED
- `WiFi.h`, `esp_wifi.h`, `esp_now.h`, `Wire.h` from the ESP32 Arduino core

## Pairing: read this before flashing

The two nodes are paired by **hardcoded MAC address**, not by discovery. Each
sketch holds the MAC of the *other* node in the `peerAddress` block near the top
of the file, selected by `#if DEVICE_ID == 1`.

This is deliberate. Broadcast-based discovery was tried and gave persistent
trouble that was never fully diagnosed. Pinning the peer MAC is what made the
link work reliably, so treat it as the intended design and not as a shortcut
waiting to be replaced. Moving back to broadcast pairing re-introduces the
original problem.

The tradeoff is that a fresh pair of boards will not talk to each other until
you replace those two MACs. There is no automatic pairing and no broadcast
fallback. If the LEDs behave correctly but no haptic ever fires, a wrong MAC is
the first thing to check.

To find a board's MAC, flash any sketch that prints `WiFi.macAddress()` with
`WiFi.mode(WIFI_STA)` set, and read it over serial.

Both nodes are pinned to WiFi channel 1. Both must be on the same channel.

## Flashing

1. Set `DEVICE_ID` to `1`, confirm the `peerAddress` for node 1 is node 2's MAC,
   flash the first unit.
2. Set `DEVICE_ID` to `2`, confirm its `peerAddress` is node 1's MAC, flash the
   second unit.
3. Power both. Place a finger on each MAX30102.

Arduino IDE settings for the ESP32-S3 SuperMini: board "ESP32S3 Dev Module",
USB CDC On Boot **enabled** (otherwise the sketch runs but serial output never
appears), flash size 4MB, and partition scheme **"Minimal SPIFFS (1.9MB APP
with OTA/128KB SPIFFS)"**. The COM port number changes after each flash, so
reselect it before opening the serial monitor.

The partition scheme is not optional. On the stock "Default 4MB with spiffs"
scheme this sketch fills 97% of the 1.2MB app partition, leaving roughly 36KB
of headroom, so almost any addition fails to link. Minimal SPIFFS gives 1.9MB
and keeps OTA available. Huge APP gives 3MB if OTA is never wanted here.

### Reproducible build

`heartbeat_node/sketch.yaml` pins the board, partition scheme, core version and
all four library versions, so the build can be reproduced without relying on
whatever happens to be installed:

```
arduino-cli compile --profile esp32s3
arduino-cli upload  --profile esp32s3 -p COM<n>
```

## What the LED means

| Colour | State |
|---|---|
| Blue | powered and running, no finger on the sensor |
| Green | finger detected, waiting for a beat lock |
| Red flash | own beat detected |
| Fast red blink, forever | fatal error at startup, a sensor was not found |

The LED always shows *your own* pulse. The other person's pulse is the haptic,
never the light.

## Tuning

Constants at the top of the sketch:

- `LED_BRIGHTNESS` (80): kept low deliberately for LiPo life.
- `BLINK_DURATION_MS` (80): how long the LED holds red per beat.
- `BPM_MIN` / `BPM_MAX` (40/200): readings outside this are discarded as noise
  or no-finger.
- `HAPTIC_EFFECT` (1): DRV2605 waveform library effect 1, "Strong Click 100%".
  See the Adafruit library or the datasheet for the full list.
- Finger detection threshold is `irValue > 50000`, inline in `readOwnPulse()`.
  Tune it there if the sensor triggers on ambient light or misses a light touch.

## How the beat gets across

Sending one packet per detected beat would flood the link at high BPM, so
instead each node sends at an interval derived from its own current BPM: one
packet per beat period, recomputed each time. The receiver treats every arriving
packet as one beat and fires the haptic.

The consequence worth knowing: the felt rate is right, but the felt *phase* is
not locked to the sender's actual beat. You feel the other person's tempo, not
their exact instant of beating.

## Known gaps

- Not verified as compiling. `FastLED`, `Adafruit DRV2605`, and the SparkFun
  MAX3010x library are not installed in the sketchbook this repo was assembled
  from, so the sketch has not been built here. Install those three before the
  first flash.
- The felt phase is not locked to the sender's beat, only the rate. See "How
  the beat gets across" above.
- `flashError()` writes the LED directly rather than through `showColor()`.
  That is intentional, since it needs to alternate on a fixed blink, and it
  never returns anyway.
