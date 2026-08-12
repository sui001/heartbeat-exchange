/*
 * PROJECT: Heartbeat Exchange
 * DEVICE:  Symmetric pulse node (set DEVICE_ID per unit)
 *
 * Each device reads its own heartbeat via MAX30102, blinks its onboard
 * RGB LED in time with its own pulse, and broadcasts BPM over ESP-NOW.
 * Received BPM from the paired device drives the haptic motor (DRV2605L),
 * so you feel the other person's heartbeat while they feel yours.
 *
 * CONFIGURATION:
 *   DEVICE_ID 1 = first unit
 *   DEVICE_ID 2 = second unit
 *   Nodes are paired by HARDCODED peer MAC (see peerAddress below), not by
 *   broadcast discovery. Broadcast pairing was tried and was unreliable;
 *   pinning the MAC is what made the link work, so do not revert to it.
 *   A fresh pair of boards needs both peerAddress values replaced.
 *
 * WIRING (both devices identical):
 *   MAX30102  SDA  -> GPIO8
 *   MAX30102  SCL  -> GPIO9
 *   MAX30102  VIN  -> 3.3V
 *   MAX30102  GND  -> GND
 *   DRV2605L  SDA  -> GPIO8  (shared I2C bus)
 *   DRV2605L  SCL  -> GPIO9
 *   DRV2605L  VIN  -> 3.3V
 *   DRV2605L  GND  -> GND
 *   Haptic motor   -> DRV2605L OUT+ / OUT-
 *   RGB LED        -> GPIO47 (onboard WS2812B, no wiring needed)
 *
 * LIBRARIES REQUIRED:
 *   - SparkFun MAX3010x Pulse and Proximity Sensor Library
 *   - Adafruit DRV2605 Library
 *   - FastLED
 *   - esp_now.h / WiFi.h (built into ESP32 Arduino core)
 *
 * FLASH INSTRUCTIONS:
 *   1. Set DEVICE_ID to 1, check its peerAddress is node 2's MAC, flash unit 1
 *   2. Set DEVICE_ID to 2, check its peerAddress is node 1's MAC, flash unit 2
 *   3. Power both on. If LEDs look right but no haptic ever fires, a wrong
 *      peerAddress is the first thing to check.
 */

// ─── CHANGE THIS PER DEVICE ───────────────────────────────────────────────────
#define DEVICE_ID  1   // 1 or 2
// ──────────────────────────────────────────────────────────────────────────────

#include <WiFi.h>
#include "esp_wifi.h"
#include <esp_now.h>
#include <Wire.h>
#include <FastLED.h>
#include "MAX30105.h"          // SparkFun MAX3010x library
#include "heartRate.h"         // SparkFun beat detection helper
#include <Adafruit_DRV2605.h>

// ─── CONFIG ───────────────────────────────────────────────────────────────────

// RGB LED
#define LED_PIN        47
#define NUM_LEDS       1
#define LED_BRIGHTNESS 80      // 0-255, keep lowish for lipo longevity

// Heartbeat blink
#define BLINK_DURATION_MS  80  // how long the LED stays lit per beat

// BPM validity window -- ignore readings outside this (noise / no finger)
#define BPM_MIN  40
#define BPM_MAX  200

// How often to broadcast BPM (ms) -- keeps ESP-NOW traffic light
#define BROADCAST_INTERVAL_MS  500

// DRV2605L haptic effect for each received beat
// Effect 1 = "Strong Click 100%" -- short, punchy, good for pulse feel
// See Adafruit DRV2605 library or datasheet for full waveform library list
#define HAPTIC_EFFECT  1

// LED colours. These are the values handleLED() actually uses -- previously the
// named constants disagreed with the literals in the function and had no effect
// on anything you could see, so they were set to match rather than the reverse
// (changing them to the old declared values would have altered the appearance).
#define PULSE_COLOR    CRGB::Red        // own beat detected
#define WAITING_COLOR  CRGB(0, 60, 0)   // finger on sensor, no beat lock yet
#define IDLE_COLOR     CRGB(0, 0, 60)   // online, no finger

// ─── GLOBALS ──────────────────────────────────────────────────────────────────

CRGB leds[NUM_LEDS];
MAX30105 pulseSensor;
Adafruit_DRV2605 haptic;

// Peer MAC addresses -- each device sends directly to the other
#if DEVICE_ID == 1
  uint8_t peerAddress[] = {0xAC, 0x27, 0x6E, 0xAF, 0x0E, 0x30};  // Node 2
#else
  uint8_t peerAddress[] = {0xAC, 0x27, 0x6E, 0xAE, 0x09, 0x40};  // Node 1
#endif

// Payload structure -- both devices use the same struct
typedef struct {
  uint8_t  senderID;
  uint8_t  bpm;
  bool     fingerDetected;
} HeartPayload;

// Local pulse state
volatile bool  ownBeatFlag    = false;   // set in beat detection, consumed in loop
float          ownBPM         = 0;
bool           fingerOnSensor = false;
unsigned long  lastBroadcast  = 0;
unsigned long  lastBeatTime   = 0;       // for LED blink timing

// Received state
volatile bool  remoteBeatPending = false;  // haptic trigger from received packet
float          remoteBPM         = 0;
bool           remoteFingerOn    = false;

// Beat detection (SparkFun pattern)
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;

// ─── ESP-NOW CALLBACKS ────────────────────────────────────────────────────────

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  // silent -- could Serial.print for debug
}

void onDataReceived(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len != sizeof(HeartPayload)) return;

  HeartPayload incoming;
  memcpy(&incoming, incomingData, sizeof(incoming));

  // Drop our own broadcasts
  if (incoming.senderID == DEVICE_ID) return;

  remoteBPM      = incoming.bpm;
  remoteFingerOn = incoming.fingerDetected;

  // Each received packet = one beat from the other person
  // (broadcast fires at BPM-derived interval -- see broadcastBeat())
  if (incoming.fingerDetected && incoming.bpm >= BPM_MIN && incoming.bpm <= BPM_MAX) {
    remoteBeatPending = true;
  }
}

// ─── SETUP ────────────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.printf("\n[Heartbeat Node %d] Starting...\n", DEVICE_ID);

  // RGB LED
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(LED_BRIGHTNESS);
  showColor(IDLE_COLOR);

  // I2C
  Wire.begin(8, 9);

  // MAX30102
  if (!pulseSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("[ERROR] MAX30102 not found -- check wiring");
    flashError();
  }
  pulseSensor.setup();
  pulseSensor.setPulseAmplitudeRed(0x0A);   // low red LED for finger detection
  pulseSensor.setPulseAmplitudeGreen(0);
  Serial.println("[OK] MAX30102 ready");

  // DRV2605L
  if (!haptic.begin()) {
    Serial.println("[ERROR] DRV2605L not found -- check wiring");
    flashError();
  }
  haptic.selectLibrary(1);
  haptic.setMode(DRV2605_MODE_INTTRIG);   // trigger via software
  Serial.println("[OK] DRV2605L ready");

  // WiFi (required for ESP-NOW even in non-WiFi mode)
  WiFi.mode(WIFI_STA);
  WiFi.begin();
  delay(500);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  WiFi.disconnect();

  // ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW init failed");
    flashError();
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataReceived);

  // Register broadcast peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (!esp_now_is_peer_exist(peerAddress)) {
    esp_now_add_peer(&peerInfo);
  }

  Serial.println("[OK] ESP-NOW ready");
  Serial.printf("[Heartbeat Node %d] Running. Place finger on MAX30102.\n", DEVICE_ID);
}

// ─── LOOP ─────────────────────────────────────────────────────────────────────

void loop() {
  readOwnPulse();
  handleLED();
  handleHaptic();
  broadcastBeat();
  debugPrint();
}

// ─── PULSE READING ────────────────────────────────────────────────────────────

void readOwnPulse() {
  long irValue = pulseSensor.getIR();

  // Finger detection threshold -- tune if needed (50000 is typical)
  fingerOnSensor = (irValue > 50000);

  if (!fingerOnSensor) {
    ownBPM = 0;
    return;
  }

  // SparkFun beat detection
  if (checkForBeat(irValue)) {
    long delta = millis() - lastBeat;
    lastBeat   = millis();

    float instantBPM = 60000.0 / delta;

    if (instantBPM >= BPM_MIN && instantBPM <= BPM_MAX) {
      // Rolling average over RATE_SIZE readings
      rates[rateSpot++] = (byte)instantBPM;
      rateSpot %= RATE_SIZE;

      float avg = 0;
      for (byte i = 0; i < RATE_SIZE; i++) avg += rates[i];
      ownBPM = avg / RATE_SIZE;

      ownBeatFlag  = true;
      lastBeatTime = millis();
    }
  }
}

// ─── OWN LED (own heartbeat) ──────────────────────────────────────────────────
// Blue  = online, no finger
// Green = finger detected, waiting for beat lock
// Red flash = beat detected

// Push to the LED only when the colour actually changes. handleLED() runs every
// pass of the main loop, and FastLED.show() on a WS2812 disables interrupts for
// the length of the transfer, so calling it unconditionally was tying up the
// loop continuously for no visible benefit.
void showColor(const CRGB &c) {
  static CRGB shown   = CRGB::Black;
  static bool primed  = false;

  if (primed && c == shown) return;

  shown    = c;
  primed   = true;
  leds[0]  = c;
  FastLED.show();
}

void handleLED() {
  if (!fingerOnSensor) {
    showColor(IDLE_COLOR);
    return;
  }

  if (ownBeatFlag) {
    ownBeatFlag = false;
    showColor(PULSE_COLOR);
    return;
  }

  if (millis() - lastBeatTime < BLINK_DURATION_MS) {
    showColor(PULSE_COLOR);
  } else {
    showColor(WAITING_COLOR);
  }
}

// ─── HAPTIC (other person's heartbeat) ───────────────────────────────────────

void handleHaptic() {
  if (remoteBeatPending) {
    remoteBeatPending = false;
    haptic.setWaveform(0, HAPTIC_EFFECT);
    haptic.setWaveform(1, 0);   // end of sequence
    haptic.go();
  }
}

// ─── BROADCAST ────────────────────────────────────────────────────────────────
//
// Rather than broadcasting on every beat detection (which could flood at high BPM),
// we broadcast at a fixed interval carrying current BPM. The receiver treats
// each arriving packet as one heartbeat trigger -- so we scale the interval
// to approximate the actual beat period.

void broadcastBeat() {
  unsigned long now = millis();

  // Derive broadcast interval from own BPM (one packet per beat period)
  // Falls back to BROADCAST_INTERVAL_MS if no valid reading
  unsigned long beatInterval = BROADCAST_INTERVAL_MS;
  if (fingerOnSensor && ownBPM >= BPM_MIN && ownBPM <= BPM_MAX) {
    beatInterval = (unsigned long)(60000.0 / ownBPM);
  }

  if (now - lastBroadcast >= beatInterval) {
    lastBroadcast = now;

    HeartPayload payload;
    payload.senderID       = DEVICE_ID;
    payload.bpm            = (uint8_t)constrain(ownBPM, 0, 255);
    payload.fingerDetected = fingerOnSensor;

    esp_now_send(peerAddress, (uint8_t *)&payload, sizeof(payload));
  }
}

// ─── DEBUG ────────────────────────────────────────────────────────────────────

unsigned long lastDebug = 0;
void debugPrint() {
  if (millis() - lastDebug < 2000) return;
  lastDebug = millis();

  Serial.printf("[Node %d] Own BPM: %.0f | Finger: %s | Remote BPM: %.0f | Remote finger: %s\n",
    DEVICE_ID,
    ownBPM,
    fingerOnSensor  ? "YES" : "no",
    remoteBPM,
    remoteFingerOn  ? "YES" : "no"
  );
}

// ─── UTILITY ──────────────────────────────────────────────────────────────────

void flashError() {
  // Red flash loop on fatal error -- never returns
  while (true) {
    leds[0] = CRGB::Red;
    FastLED.show();
    delay(200);
    leds[0] = CRGB::Black;
    FastLED.show();
    delay(200);
  }
}
