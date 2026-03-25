// ============================================================
// TennisSwingPath v2 — Accurate Swing Path Analysis
// Arduino Nano 33 BLE | BMI270 IMU
//
// Key improvements over v1 (sketch_swing_path_v1):
//
//   1. BUFFERED ANALYSIS: All IMU samples stored during swing.
//      Post-swing analysis on the complete buffer gives exact:
//        - Impact frame (argmax over full buffer, not real-time)
//        - Forward / follow-through arc split at true impact
//        - Whip computed with measured DT, not assumed 20ms
//
//   2. MEASURED DT: Integration uses actual millis() delta per
//      sample, not a fixed constant. Removes 5–15% timing error
//      caused by BLE write + Serial overhead in the loop.
//
//   3. GYRO BIAS CORRECTION: Quiet-period samples build a running
//      average of gyro offset. Each integrated angular step is
//      bias-subtracted before accumulation, reducing drift over
//      the swing window.
//
//   4. WRIST ROLL AT IMPACT: Instantaneous gx at the exact peak-
//      acceleration sample (not integrated) — cleaner measure of
//      racket face angle at contact.
//
//   5. SWING TEMPO: Time between consecutive swings (rally cadence).
//
//   6. LED PATH FEEDBACK: 1 blink=flat, 2=topspin, 3=slice.
//      Gives instant court-side feedback without looking at a phone.
//
//   7. NON-BLOCKING MAIN LOOP: BLE.poll() + sensor logic runs
//      independently. Sketch works over USB Serial even without
//      a BLE central connected.
//
//   8. BUFFER OVERFLOW PROTECTION: Swing auto-closes and is
//      analyzed if buffer fills before accel drops (very long swing
//      or missed end-trigger).
//
// IMU Axis Convention (BMI270 on Nano 33 BLE, sensor at grip):
//   X = along handle (toward tip)       gx = wrist pronation/supination
//   Y = perpendicular to string face    gy > 0 = upward path (topspin)
//   Z = across string face (lateral)    gz = main forward-swing rotation
//
// Analytics per swing (BLE analyticsChar + Serial):
//   SWING_ANALYTICS,<n>,<dur_ms>,<peak_g>,<peak_dps>,<head_mph>,
//     <attack_deg>,<h_arc_deg>,<v_arc_deg>,<spin_roll_dps>,
//     <ft_ratio>,<max_whip>,<tempo_ms>,<path_label>
//
// CSV row stream (BLE dataChar + Serial):
//   <ts_ms>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<magnitude>,<whip>
//
// BLE Characteristics (Service 180C):
//   2A56 — raw CSV row (stream, every sample)
//   2A57 — status events (CONNECTED / SWING_START / SWING_END / etc.)
//   2A58 — commands: write "START" or "STOP"
//   2A59 — per-swing analytics summary
// ============================================================

#include <Arduino_BMI270_BMM150.h>
#include <ArduinoBLE.h>
#include <math.h>

// ─── BLE ─────────────────────────────────────────────────────
BLEService              tennisService ("180C");
BLEStringCharacteristic dataChar      ("2A56", BLERead | BLENotify, 100);
BLEStringCharacteristic statusChar    ("2A57", BLERead | BLENotify,  64);
BLEStringCharacteristic commandChar   ("2A58", BLEWrite,             20);
BLEStringCharacteristic analyticsChar ("2A59", BLERead | BLENotify, 120);

// ─── Swing Detection Thresholds (tune per player) ────────────
// Increasing SWING_START_ACCEL reduces false positives but may
// miss short volleys. Increase SWING_START_GYRO on heavy rackets.
const float         SWING_START_ACCEL = 1.5f;  // g   — accel opens window
const float         SWING_END_ACCEL   = 1.15f; // g   — accel closes window
const float         SWING_START_GYRO  = 80.0f; // °/s — gyro confirms real swing
const int           MIN_SWING_SAMPLES = 8;     // ~160ms minimum valid swing
const unsigned long SWING_COOLDOWN    = 1500;  // ms dead zone between swings

// ─── Physical Constants ───────────────────────────────────────
// RACKET_ARM_M: distance from grip sensor to approximate racket
// head centre. Adjust if sensor is mounted higher on the handle.
const float RACKET_ARM_M = 0.65f; // metres

// ─── Swing Sample Buffer ──────────────────────────────────────
// 200 samples × (6 floats + 1 unsigned long) = ~5600 bytes.
// Nano 33 BLE has 256 KB SRAM — this is safe.
// At 50 Hz this covers up to 4 seconds per swing.
#define MAX_SWING_SAMPLES 200

struct ImuSample {
  float ax, ay, az;  // acceleration (g)
  float gx, gy, gz;  // angular velocity (°/s), bias-corrected at capture
  unsigned long ts;  // millis() at the moment of capture
};

ImuSample swingBuf[MAX_SWING_SAMPLES];
int       bufLen  = 0;   // valid samples in buffer (0 … MAX_SWING_SAMPLES)
bool      bufFull = false;

// ─── Gyro Bias Estimator ──────────────────────────────────────
// While the sensor is quiet (not swinging), we continuously
// average the raw gyro to estimate DC offset (bias).
// This is subtracted from each sample during integration.
// Window of 20 samples at 50 Hz = 400ms smoothing.
#define BIAS_WINDOW 20

float biasGxBuf[BIAS_WINDOW] = {0};
float biasGyBuf[BIAS_WINDOW] = {0};
float biasGzBuf[BIAS_WINDOW] = {0};
int   biasWriteIdx = 0;
int   biasCount    = 0;

float gxBias = 0, gyBias = 0, gzBias = 0;

void updateGyroBias(float gx, float gy, float gz) {
  int i = biasWriteIdx % BIAS_WINDOW;
  biasGxBuf[i] = gx;
  biasGyBuf[i] = gy;
  biasGzBuf[i] = gz;
  biasWriteIdx++;
  biasCount = min(biasWriteIdx, BIAS_WINDOW);

  float sx = 0, sy = 0, sz = 0;
  for (int j = 0; j < biasCount; j++) {
    sx += biasGxBuf[j];
    sy += biasGyBuf[j];
    sz += biasGzBuf[j];
  }
  gxBias = sx / biasCount;
  gyBias = sy / biasCount;
  gzBias = sz / biasCount;
}

// ─── Session State ────────────────────────────────────────────
bool          recording      = false;
bool          swingActive    = false;
int           swingCount     = 0;
unsigned long sessionStart   = 0;
unsigned long lastSwingEnd   = 0;
unsigned long lastSwingStart = 0;
unsigned long prevSwingMs    = 0; // tempo: interval between swing starts

// ─── Streaming Whip (per-sample, for CSV) ────────────────────
// The full-accuracy whip is recalculated from the buffer during
// post-swing analysis. This streaming value is logged per row.
float prevMagnitude = 0;
float whipWindow[5] = {0};
int   whipIdx       = 0;

// ─── Helpers ─────────────────────────────────────────────────

void blinkLED(int times, int onMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(onMs);
    digitalWrite(LED_BUILTIN, LOW);  delay(onMs);
  }
}

// Streaming whip: smoothed dMagnitude/dt for CSV column only.
float streamWhip(float mag) {
  float rate = (mag - prevMagnitude) / 0.020f;
  whipWindow[whipIdx % 5] = rate;
  whipIdx++;
  float s = 0;
  for (int i = 0; i < 5; i++) s += whipWindow[i];
  return s / 5.0f;
}

// Attack angle → path label.
String pathLabel(float deg) {
  if (deg >  25.0f) return "LOW_TO_HIGH";
  if (deg >   8.0f) return "TOPSPIN";
  if (deg >  -8.0f) return "FLAT";
  if (deg > -25.0f) return "SLICE";
  return "HIGH_TO_LOW";
}

// ─── Post-Swing Analysis ──────────────────────────────────────
//
// Called once, on the complete buffer after the swing window closes.
// All angular integration here uses the measured DT between adjacent
// samples (constrained to 5–50ms for safety), and bias-corrected gyro.
//
void analyzeAndPublish() {
  if (bufLen < MIN_SWING_SAMPLES) return;

  // ── 1. Find impact: sample with highest acceleration magnitude ──
  // This is the argmax over the entire swing, not a real-time estimate.
  // Splitting the forward / follow-through arcs at this index gives the
  // most physically meaningful separation.
  int   impactIdx = 0;
  float peakMag   = 0.0f;
  for (int i = 0; i < bufLen; i++) {
    float m = sqrtf(swingBuf[i].ax * swingBuf[i].ax
                  + swingBuf[i].ay * swingBuf[i].ay
                  + swingBuf[i].az * swingBuf[i].az);
    if (m > peakMag) { peakMag = m; impactIdx = i; }
  }

  // ── 2. Peak angular velocity (for head speed estimate) ──────────
  float peakOmegaDps = 0.0f;
  for (int i = 0; i < bufLen; i++) {
    float om = sqrtf(swingBuf[i].gx * swingBuf[i].gx
                   + swingBuf[i].gy * swingBuf[i].gy
                   + swingBuf[i].gz * swingBuf[i].gz);
    if (om > peakOmegaDps) peakOmegaDps = om;
  }

  // ── 3. Integrate gyro arcs (true DT, bias-corrected) ────────────
  // Forward arc  = samples 0  … impactIdx  (approach to impact)
  // Follow arc   = samples impactIdx … bufLen-1 (follow-through)
  //
  // gx: wrist pronation/supination around the handle long axis
  // gy: topspin/slice plane rotation (+ = upward = topspin)
  // gz: lateral / main forward swing rotation
  float fwd_gx = 0, fwd_gy = 0, fwd_gz = 0;
  float fth_gx = 0, fth_gy = 0, fth_gz = 0;

  for (int i = 1; i < bufLen; i++) {
    // Measured DT between adjacent samples (ms → s).
    // Clamp to 5–50ms: protects against glitches (BLE stall, etc.).
    float dt = (float)(swingBuf[i].ts - swingBuf[i - 1].ts) / 1000.0f;
    dt = constrain(dt, 0.005f, 0.050f);

    // Gyro values stored in buffer are already bias-corrected (see capture below).
    float gxc = swingBuf[i].gx;
    float gyc = swingBuf[i].gy;
    float gzc = swingBuf[i].gz;

    if (i <= impactIdx) {
      fwd_gx += gxc * dt;
      fwd_gy += gyc * dt;
      fwd_gz += gzc * dt;
    } else {
      fth_gx += gxc * dt;
      fth_gy += gyc * dt;
      fth_gz += gzc * dt;
    }
  }

  // ── 4. Attack angle (vertical swing plane) ──────────────────────
  // gy_integrated: how much the racket head went up (+) or down (−)
  //                in the vertical plane during the forward swing.
  // gz_integrated: horizontal arc — how far the racket swung forward.
  //
  // atan2(vy, hx) gives the angle of the path in the vertical plane.
  // Minimum horizontal arc of 2° required to suppress noise on very
  // short taps where gz_int would be near zero.
  float hArc      = fabsf(fwd_gz); // horizontal arc, always positive
  float vArc      = fwd_gy;        // vertical arc, signed (+ = upward)
  float attackDeg = (hArc > 2.0f)
                    ? atan2f(vArc, hArc) * 180.0f / PI
                    : 0.0f;

  // ── 5. Wrist roll at impact ──────────────────────────────────────
  // Instantaneous (not integrated) gx at the exact impact sample.
  // This is more informative for contact-point wrist position than
  // the integrated gx over the forward arc.
  //   > 0: wrist rolling over the ball  → topspin-promoting roll
  //   < 0: wrist cutting under the ball → slice-promoting roll
  float spinRollDps = swingBuf[impactIdx].gx; // already bias-corrected

  // ── 6. Integrated wrist roll (forward arc) ──────────────────────
  // Total wrist pronation through the forward swing.
  // Complements the instantaneous value above.
  float spinAxisDeg = (hArc > 2.0f)
                      ? atan2f(fwd_gx, fabsf(fwd_gz)) * 180.0f / PI
                      : 0.0f;

  // ── 7. Follow-through ratio ──────────────────────────────────────
  // fthArc / fwdArc:
  //   > 1.2  = full accelerating follow-through (good form / power stroke)
  //   0.5–1  = adequate follow-through
  //   < 0.5  = blocked / punched / volley-style
  float fwdArcMag = sqrtf(fwd_gx*fwd_gx + fwd_gy*fwd_gy + fwd_gz*fwd_gz);
  float fthArcMag = sqrtf(fth_gx*fth_gx + fth_gy*fth_gy + fth_gz*fth_gz);
  float ftRatio   = (fwdArcMag > 1.0f) ? fthArcMag / fwdArcMag : 0.0f;

  // ── 8. Racket head speed (mph) ───────────────────────────────────
  // v = omega × r  (linear velocity at the head centre)
  // Convert °/s → rad/s, multiply by arm length, then m/s → mph.
  // This is a relative measure; accuracy depends on RACKET_ARM_M.
  float headSpeedMph = peakOmegaDps * (PI / 180.0f) * RACKET_ARM_M * 2.237f;

  // ── 9. Peak whip from buffer (accurate DT) ──────────────────────
  // Recomputed from the buffer using actual inter-sample dt.
  // Unlike the streaming version, this is not averaged; it finds
  // the single largest positive dMag/dt in the swing.
  float maxWhip = 0.0f;
  for (int i = 1; i < bufLen; i++) {
    float dt = (float)(swingBuf[i].ts - swingBuf[i - 1].ts) / 1000.0f;
    dt = constrain(dt, 0.005f, 0.050f);

    float mCur = sqrtf(swingBuf[i].ax * swingBuf[i].ax
                     + swingBuf[i].ay * swingBuf[i].ay
                     + swingBuf[i].az * swingBuf[i].az);
    float mPrv = sqrtf(swingBuf[i-1].ax * swingBuf[i-1].ax
                     + swingBuf[i-1].ay * swingBuf[i-1].ay
                     + swingBuf[i-1].az * swingBuf[i-1].az);
    float w = (mCur - mPrv) / dt;
    if (w > maxWhip) maxWhip = w;
  }

  // ── 10. Summary scalars ─────────────────────────────────────────
  float durMs     = (float)(swingBuf[bufLen - 1].ts - swingBuf[0].ts);
  float totalHArc = fabsf(fwd_gz) + fabsf(fth_gz); // total horizontal arc
  float totalVArc = fabsf(fwd_gy) + fabsf(fth_gy); // total vertical arc

  // ── 11. Build and send analytics string ─────────────────────────
  // Format:
  //   SWING_ANALYTICS,<n>,<dur_ms>,<peak_g>,<peak_dps>,<head_mph>,
  //     <attack_deg>,<h_arc_deg>,<v_arc_deg>,<spin_roll_dps>,
  //     <ft_ratio>,<max_whip>,<tempo_ms>,<path_label>
  String analytics =
    "SWING_ANALYTICS,"     +
    String(swingCount)     + "," +
    String(durMs, 0)       + "," +
    String(peakMag, 2)     + "," +
    String(peakOmegaDps,0) + "," +
    String(headSpeedMph,1) + "," +
    String(attackDeg, 1)   + "," +
    String(totalHArc, 0)   + "," +
    String(totalVArc, 0)   + "," +
    String(spinRollDps, 0) + "," +
    String(ftRatio, 2)     + "," +
    String(maxWhip, 1)     + "," +
    String(prevSwingMs)    + "," +
    pathLabel(attackDeg);

  analyticsChar.writeValue(analytics);
  statusChar.writeValue("SWING_END:" + String(swingCount));
  Serial.println(analytics);
  Serial.println("SWING_END:" + String(swingCount));

  // ── 12. LED feedback: path-dependent blink ──────────────────────
  //   1 blink = flat drive
  //   2 blinks = topspin
  //   3 blinks = slice
  if      (attackDeg >  8.0f) blinkLED(2, 70);
  else if (attackDeg < -8.0f) blinkLED(3, 70);
  else                         blinkLED(1, 120);
  digitalWrite(LED_BUILTIN, HIGH);
}

// ─── Setup ───────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  // Non-blocking Serial wait: allow up to 3s for USB host to attach.
  // After that the sketch continues anyway → works standalone / BLE-only.
  unsigned long t = millis();
  while (!Serial && (millis() - t < 3000));

  pinMode(LED_BUILTIN, OUTPUT);

  // BMI270 init
  if (!IMU.begin()) {
    // Rapid blink = IMU failure — check library / board selection
    while (1) blinkLED(1, 200);
  }

  // BLE init
  if (!BLE.begin()) {
    // Slower blink = BLE stack failure
    while (1) blinkLED(1, 500);
  }

  BLE.setLocalName("TennisSensor");
  BLE.setAdvertisedService(tennisService);
  tennisService.addCharacteristic(dataChar);
  tennisService.addCharacteristic(statusChar);
  tennisService.addCharacteristic(commandChar);
  tennisService.addCharacteristic(analyticsChar);
  BLE.addService(tennisService);
  BLE.advertise();

  blinkLED(3, 300); // 3 slow blinks = ready
  Serial.println("READY");
}

// ─── Main Loop ───────────────────────────────────────────────
//
// Non-blocking design: BLE.poll() keeps the BLE stack alive each
// iteration. Sensor logic runs every iteration regardless of
// BLE connection state — the sketch works over Serial alone.
//
void loop() {
  unsigned long loopStart = millis();

  // ── BLE housekeeping ────────────────────────────────────────
  BLE.poll();
  BLEDevice central = BLE.central();

  // Connection / disconnection events
  static bool wasConnected = false;
  bool isConnected = central && central.connected();

  if (isConnected && !wasConnected) {
    statusChar.writeValue("CONNECTED");
    Serial.println("Connected: " + central.address());
    digitalWrite(LED_BUILTIN, HIGH);
  }
  if (!isConnected && wasConnected) {
    recording   = false;
    swingActive = false;
    bufLen      = 0;
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Disconnected");
    blinkLED(5, 100);
    BLE.advertise();
  }
  wasConnected = isConnected;

  // ── Handle START / STOP commands ────────────────────────────
  if (isConnected && commandChar.written()) {
    String cmd = commandChar.value();
    cmd.trim();

    if (cmd == "START") {
      recording      = true;
      swingCount     = 0;
      swingActive    = false;
      bufLen         = 0;
      sessionStart   = millis();
      lastSwingEnd   = 0;
      lastSwingStart = 0;
      prevSwingMs    = 0;
      prevMagnitude  = 0;
      biasWriteIdx   = 0;
      biasCount      = 0;
      gxBias = gyBias = gzBias = 0;
      for (int i = 0; i < 5; i++) whipWindow[i] = 0;
      whipIdx = 0;

      statusChar.writeValue("SESSION_START");
      Serial.println("SESSION_START");
      blinkLED(2, 100);
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else if (cmd == "STOP") {
      // Finalize any swing that was still open when STOP arrived
      if (swingActive && bufLen >= MIN_SWING_SAMPLES) {
        analyzeAndPublish();
      }
      recording   = false;
      swingActive = false;
      bufLen      = 0;
      String summary = "SESSION_END,SWINGS:" + String(swingCount);
      statusChar.writeValue(summary);
      Serial.println(summary);
      blinkLED(3, 100);
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }

  // ── Sensor logic ─────────────────────────────────────────────
  if (!recording) {
    unsigned long elapsed = millis() - loopStart;
    if (elapsed < 20) delay(20 - elapsed);
    return;
  }

  // Skip if IMU data not ready yet
  if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) {
    unsigned long elapsed = millis() - loopStart;
    if (elapsed < 20) delay(20 - elapsed);
    return;
  }

  float ax, ay, az, gx, gy, gz;
  IMU.readAcceleration(ax, ay, az);
  IMU.readGyroscope(gx, gy, gz);

  unsigned long now      = millis();
  float magnitude        = sqrtf(ax*ax + ay*ay + az*az);
  float omegaDps         = sqrtf(gx*gx + gy*gy + gz*gz);

  // Bias-corrected gyro (used for buffer storage and bias update)
  float gxc = gx - gxBias;
  float gyc = gy - gyBias;
  float gzc = gz - gzBias;

  // ── Update gyro bias during quiet periods ──────────────────
  // Only update when sensor is stationary: low accel variation
  // and low angular velocity.  This prevents swing motion from
  // contaminating the bias estimate.
  if (!swingActive && omegaDps < 20.0f && magnitude < 1.3f) {
    updateGyroBias(gx, gy, gz);
  }

  // ── Streaming whip (for CSV column) ───────────────────────
  float whip = streamWhip(magnitude);
  prevMagnitude = magnitude;

  // ── Swing START trigger ────────────────────────────────────
  // Require BOTH accel AND gyro above threshold to suppress:
  //   - Surface impacts (high accel, low gyro)
  //   - Gentle waving (high gyro, low accel)
  if (!swingActive &&
      magnitude > SWING_START_ACCEL &&
      omegaDps  > SWING_START_GYRO  &&
      (now - lastSwingEnd) > SWING_COOLDOWN) {

    swingActive = true;
    swingCount++;
    bufLen   = 0;
    bufFull  = false;

    // Swing tempo: interval from last swing start to this one
    if (lastSwingStart > 0) {
      prevSwingMs = now - lastSwingStart;
    }
    lastSwingStart = now;

    // Store trigger sample as first buffer entry
    swingBuf[bufLen++] = {ax, ay, az, gxc, gyc, gzc, now};

    String msg = "SWING_START:" + String(swingCount);
    statusChar.writeValue(msg);
    Serial.println(msg);

    // Brief LED flash at swing start
    digitalWrite(LED_BUILTIN, LOW);
    delay(15);
    digitalWrite(LED_BUILTIN, HIGH);
  }

  // ── Per-sample: push to swing buffer ──────────────────────
  else if (swingActive) {
    if (bufLen < MAX_SWING_SAMPLES) {
      swingBuf[bufLen++] = {ax, ay, az, gxc, gyc, gzc, now};
    } else {
      // Buffer full: auto-close the swing
      bufFull = true;
    }
  }

  // ── Swing END trigger ─────────────────────────────────────
  // Close when accel quiets AND minimum duration reached,
  // OR if the buffer filled (very long swing / missed end).
  if (swingActive &&
      ((magnitude < SWING_END_ACCEL && bufLen >= MIN_SWING_SAMPLES)
       || bufFull)) {

    swingActive  = false;
    lastSwingEnd = now;

    analyzeAndPublish();
  }

  // ── Stream CSV row ─────────────────────────────────────────
  // Schema (matches CLAUDE.md):
  //   timestamp_ms, ax, ay, az, gx, gy, gz, magnitude, whip
  unsigned long ts = now - sessionStart;
  String row =
    String(ts)           + "," +
    String(ax, 3)        + "," +
    String(ay, 3)        + "," +
    String(az, 3)        + "," +
    String(gx, 3)        + "," +
    String(gy, 3)        + "," +
    String(gz, 3)        + "," +
    String(magnitude, 3) + "," +
    String(whip, 2);

  dataChar.writeValue(row);
  Serial.println(row);

  // ── Hold 50 Hz loop rate ───────────────────────────────────
  // Using measured elapsed time ensures we compensate for BLE
  // write and Serial overhead — not just delaying a fixed 20ms.
  unsigned long elapsed = millis() - loopStart;
  if (elapsed < 20) delay(20 - elapsed);
}
