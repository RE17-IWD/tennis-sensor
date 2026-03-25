// ============================================================
// TennisSwingPath v1 — Accurate Swing Path Analysis
// Arduino Nano 33 BLE Sense | BMI270 IMU
//
// Key improvements over whip_effectb / mar18_timestamped:
//   1. Gyro INTEGRATION → real angular displacement per axis
//      (not just averaging, which loses cumulative rotation)
//   2. Impact detection via acceleration peak, not threshold
//   3. Attack angle from forward-swing gy_int vs gz_int arc
//   4. Spin axis angle from gx_int (wrist pronation/supination)
//   5. Follow-through ratio = follow arc / forward arc
//   6. Racket head speed: peak_omega_rad × arm_length → mph
//   7. Dual trigger: accel AND gyro thresholds (reduces false swings)
//   8. Non-blocking Serial wait (works without USB connection)
//   9. CSV: timestamp_ms,ax,ay,az,gx,gy,gz,magnitude,whip
//
// Analytics per swing (BLE + Serial):
//   SWING_ANALYTICS,<num>,<dur_ms>,<peak_g>,<peak_dps>,
//     <head_mph>,<attack_deg>,<h_arc_deg>,<v_arc_deg>,
//     <spin_axis_deg>,<ft_ratio>,<path_label>
// ============================================================

#include <Arduino_BMI270_BMM150.h>
#include <ArduinoBLE.h>

// ─── BLE ─────────────────────────────────────────────────────
BLEService              tennisService ("180C");
BLEStringCharacteristic dataChar      ("2A56", BLERead | BLENotify, 100);
BLEStringCharacteristic statusChar    ("2A57", BLERead | BLENotify, 60);
BLEStringCharacteristic commandChar   ("2A58", BLEWrite,            20);
BLEStringCharacteristic analyticsChar ("2A59", BLERead | BLENotify, 120);

// ─── Swing Detection Thresholds ──────────────────────────────
// IMPORTANT: Do not change without re-validating on labeled data.
const float SWING_START_ACCEL  = 1.5;   // g   — accel to open a swing window
const float SWING_END_ACCEL    = 1.15;  // g   — accel to close a swing window
const float SWING_START_GYRO   = 80.0;  // °/s — confirms real swing, not a bump
const int   MIN_SWING_SAMPLES  = 8;     // ~160ms minimum valid swing
const unsigned long SWING_COOLDOWN = 1500; // ms dead zone between swings

// ─── Physical Constants ───────────────────────────────────────
// Racket arm = distance from grip sensor to approximate head centre.
// Adjust if sensor is mounted further up the handle.
const float RACKET_ARM_M = 0.65; // metres
const float DT           = 0.02; // seconds per sample (50 Hz loop)

// ─── Session State ────────────────────────────────────────────
bool          recording    = false;
bool          swingActive  = false;
int           swingCount   = 0;
unsigned long sessionStart = 0;
unsigned long lastSwingEnd = 0;

// ─── Whip Effect ─────────────────────────────────────────────
// Whip = smoothed d(magnitude)/dt  (g/s)
// Captures the explosive acceleration snap at ball contact.
float prevMagnitude = 0;
float maxWhipRate   = 0;
float whipWindow[5] = {0};
int   whipIdx       = 0;

// ─── Per-swing Accumulators ───────────────────────────────────
// Angular displacement via Euler integration (deg = sum of gx*DT each sample).
// Split into forward-swing phase (before impact peak) and follow-through.
float gx_int   = 0, gy_int   = 0, gz_int   = 0; // total arc
float fwd_gx   = 0, fwd_gy   = 0, fwd_gz   = 0; // forward-swing arc
float fthr_gx  = 0, fthr_gy  = 0, fthr_gz  = 0; // follow-through arc

float peakMagnitude = 0; // peak accel magnitude during swing (impact point)
float peakOmegaDps  = 0; // peak angular velocity magnitude (°/s)
float impactGy      = 0; // gy at the impact sample  (for instantaneous angle)
float impactGz      = 0; // gz at the impact sample

int   swingSamples  = 0; // total samples in this swing
int   fwdSamples    = 0; // samples before impact peak
int   followSamples = 0; // samples after impact peak
bool  impactPassed  = false;

// ─── LED Helper ───────────────────────────────────────────────
void blinkLED(int times, int ms) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(ms);
    digitalWrite(LED_BUILTIN, LOW);  delay(ms);
  }
}

// ─── Whip Calculation ─────────────────────────────────────────
float calcWhip(float mag) {
  float rate = (mag - prevMagnitude) / DT;
  whipWindow[whipIdx % 5] = rate;
  whipIdx++;
  float sum = 0;
  for (int i = 0; i < 5; i++) sum += whipWindow[i];
  return sum / 5.0;
}

// ─── Reset Per-swing State ────────────────────────────────────
void resetSwingAccumulators() {
  gx_int  = gy_int  = gz_int  = 0;
  fwd_gx  = fwd_gy  = fwd_gz  = 0;
  fthr_gx = fthr_gy = fthr_gz = 0;
  peakMagnitude = 0;
  peakOmegaDps  = 0;
  impactGy      = 0;
  impactGz      = 0;
  swingSamples  = 0;
  fwdSamples    = 0;
  followSamples = 0;
  impactPassed  = false;
  maxWhipRate   = 0;
}

// ─── Path Label (attack angle) ────────────────────────────────
// Angle > 0 = racket path going upward  (low-to-high / topspin)
// Angle = 0 = flat drive
// Angle < 0 = racket path going downward (slice)
String pathLabel(float attackDeg) {
  if (attackDeg >  25) return "LOW_TO_HIGH";
  if (attackDeg >   8) return "TOPSPIN";
  if (attackDeg >  -8) return "FLAT";
  if (attackDeg > -25) return "SLICE";
  return "HIGH_TO_LOW";
}

// ─── Spin Label (spin-axis angle from wrist pronation/gx) ─────
// Positive gx_int = wrist rolling over the ball (topspin)
// Negative gx_int = wrist cutting under the ball (slice)
String spinLabel(float spinDeg) {
  if (spinDeg >  20) return "TOPSPIN_ROLL";
  if (spinDeg >   5) return "LIGHT_TOPSPIN";
  if (spinDeg >  -5) return "FLAT_HIT";
  if (spinDeg > -20) return "LIGHT_SLICE";
  return "HEAVY_SLICE";
}

// ─── Compute and Send Swing Analytics ─────────────────────────
void publishSwingAnalytics() {
  float durationMs = swingSamples * (DT * 1000.0);

  // ── Attack angle ────────────────────────────────────────────
  // How much the forward-swing path tilts vertically.
  // Uses integrated gy (vertical plane rotation) vs gz (horizontal).
  // gy_int > 0 → path sweeping upward (low-to-high)
  // gy_int < 0 → path sweeping downward (slice)
  float hArc       = abs(fwd_gz);
  float vArc       = fwd_gy; // signed: positive = upward
  float attackAngle = (hArc > 2.0) ? atan2(vArc, hArc) * 180.0 / PI : 0.0;

  // ── Spin axis angle ──────────────────────────────────────────
  // gx = rotation around the racket's long axis (handle).
  // Wrist rolling over → positive gx → topspin.
  // Wrist undercutting → negative gx → slice.
  float totalFwdHz  = sqrt(fwd_gz * fwd_gz + fwd_gx * fwd_gx);
  float spinAxisDeg = (totalFwdHz > 2.0) ?
                      atan2(fwd_gx, abs(fwd_gz)) * 180.0 / PI : 0.0;

  // ── Follow-through ratio ─────────────────────────────────────
  // > 1.0 means a full, accelerating follow-through (good form)
  // < 0.5 means the player stopped or blocked the swing
  float fwdArcMag  = sqrt(fwd_gx*fwd_gx   + fwd_gy*fwd_gy   + fwd_gz*fwd_gz);
  float fthrArcMag = sqrt(fthr_gx*fthr_gx + fthr_gy*fthr_gy + fthr_gz*fthr_gz);
  float ftRatio    = (fwdArcMag > 1.0) ? fthrArcMag / fwdArcMag : 0.0;

  // ── Racket head speed estimate ───────────────────────────────
  // v = omega_rad × r  (r = RACKET_ARM_M from sensor to head centre)
  // Converted to mph.  Relative metric — accuracy depends on sensor position.
  float peakOmegaRad = peakOmegaDps * (PI / 180.0);
  float headSpeedMph = peakOmegaRad * RACKET_ARM_M * 2.237;

  // ── Total arcs (forward + follow) ───────────────────────────
  float totalHArc = abs(fwd_gz) + abs(fthr_gz);
  float totalVArc = abs(fwd_gy) + abs(fthr_gy);

  // ── Build analytics string ───────────────────────────────────
  // Format: SWING_ANALYTICS,num,dur_ms,peak_g,peak_dps,
  //           head_mph,attack_deg,h_arc,v_arc,spin_deg,ft_ratio,path
  String analytics =
    "SWING_ANALYTICS," +
    String(swingCount)       + "," +
    String(durationMs, 0)    + "," +
    String(peakMagnitude, 2) + "," +
    String(peakOmegaDps, 0)  + "," +
    String(headSpeedMph, 1)  + "," +
    String(attackAngle, 1)   + "," +
    String(totalHArc, 0)     + "," +
    String(totalVArc, 0)     + "," +
    String(spinAxisDeg, 1)   + "," +
    String(ftRatio, 2)       + "," +
    pathLabel(attackAngle);

  analyticsChar.writeValue(analytics);
  statusChar.writeValue("SWING_END:" + String(swingCount));
  Serial.println(analytics);
  Serial.println("SWING_END:" + String(swingCount));
}

// ─── Setup ───────────────────────────────────────────────────
void setup() {
  Serial.begin(9600);

  // Non-blocking Serial wait: give USB 3s to connect, then continue.
  // This allows the sketch to run as a standalone wearable.
  unsigned long t = millis();
  while (!Serial && (millis() - t < 3000));

  pinMode(LED_BUILTIN, OUTPUT);

  if (!IMU.begin()) {
    while (1) blinkLED(1, 200); // short rapid blink = IMU failure
  }

  if (!BLE.begin()) {
    while (1) blinkLED(1, 500); // slower blink = BLE failure
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
void loop() {
  BLEDevice central = BLE.central();

  if (!central) return;

  Serial.println("Connected: " + central.address());
  statusChar.writeValue("CONNECTED");
  digitalWrite(LED_BUILTIN, HIGH);

  while (central.connected()) {

    // ── Handle START / STOP commands from phone ──────────────
    if (commandChar.written()) {
      String cmd = commandChar.value();
      cmd.trim();

      if (cmd == "START") {
        recording    = true;
        swingCount   = 0;
        swingActive  = false;
        sessionStart = millis();
        lastSwingEnd = 0;
        prevMagnitude = 0;
        for (int i = 0; i < 5; i++) whipWindow[i] = 0;
        whipIdx = 0;
        resetSwingAccumulators();
        statusChar.writeValue("SESSION_START");
        Serial.println("SESSION_START");
        blinkLED(2, 100);
        digitalWrite(LED_BUILTIN, HIGH);
      }
      else if (cmd == "STOP") {
        recording   = false;
        swingActive = false;
        String summary = "SESSION_END,SWINGS:" + String(swingCount);
        statusChar.writeValue(summary);
        Serial.println(summary);
        blinkLED(3, 100);
        digitalWrite(LED_BUILTIN, HIGH);
      }
    }

    if (!recording) {
      delay(20);
      continue;
    }

    // ── Read IMU ──────────────────────────────────────────────
    float ax, ay, az, gx, gy, gz;
    if (!IMU.accelerationAvailable() || !IMU.gyroscopeAvailable()) {
      delay(20);
      continue;
    }

    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    unsigned long now       = millis();
    float magnitude = sqrt(ax*ax + ay*ay + az*az);
    float omegaDps  = sqrt(gx*gx + gy*gy + gz*gz); // total angular speed °/s

    // ── Whip (smoothed d(mag)/dt) ─────────────────────────────
    float whip = calcWhip(magnitude);
    if (whip > maxWhipRate) maxWhipRate = whip;
    prevMagnitude = magnitude;

    // ── Per-sample accumulators inside a swing ────────────────
    if (swingActive) {
      swingSamples++;

      // Integrate gyro → angular displacement (degrees)
      gx_int += gx * DT;
      gy_int += gy * DT;
      gz_int += gz * DT;

      // Track peak angular velocity (→ head speed)
      if (omegaDps > peakOmegaDps) peakOmegaDps = omegaDps;

      // Detect impact: peak acceleration sample.
      // We update the impact gyro snapshot while magnitude keeps rising.
      // Once it drops ≥20% from peak AND we have enough samples, mark impact passed.
      if (magnitude > peakMagnitude) {
        peakMagnitude = magnitude;
        impactGy      = gy;
        impactGz      = gz;
        impactPassed  = false; // keep updating until it actually falls
      } else if (!impactPassed &&
                 magnitude < peakMagnitude * 0.80 &&
                 swingSamples >= (MIN_SWING_SAMPLES / 2)) {
        impactPassed = true;
      }

      // Accumulate forward-swing vs follow-through arcs separately
      if (!impactPassed) {
        fwdSamples++;
        fwd_gx += gx * DT;
        fwd_gy += gy * DT;
        fwd_gz += gz * DT;
      } else {
        followSamples++;
        fthr_gx += gx * DT;
        fthr_gy += gy * DT;
        fthr_gz += gz * DT;
      }
    }

    // ── Swing START ───────────────────────────────────────────
    // Require BOTH accel and gyro threshold — prevents false triggers
    // from hitting the racket against a surface or setting it down hard.
    if (!swingActive &&
        magnitude  > SWING_START_ACCEL &&
        omegaDps   > SWING_START_GYRO  &&
        (now - lastSwingEnd > SWING_COOLDOWN)) {

      swingActive = true;
      swingCount++;
      resetSwingAccumulators();

      String msg = "SWING_START:" + String(swingCount);
      statusChar.writeValue(msg);
      Serial.println(msg);

      // Brief LED flash at swing start
      digitalWrite(LED_BUILTIN, LOW);
      delay(30);
      digitalWrite(LED_BUILTIN, HIGH);
    }

    // ── Swing END ─────────────────────────────────────────────
    // Close window when magnitude quiets and minimum duration met.
    if (swingActive &&
        magnitude < SWING_END_ACCEL &&
        swingSamples >= MIN_SWING_SAMPLES) {

      swingActive  = false;
      lastSwingEnd = now;

      // If impact was never clearly detected (very short swing),
      // treat all samples as forward swing for the angle calculation.
      if (!impactPassed) {
        fwd_gx += fthr_gx; fwd_gy += fthr_gy; fwd_gz += fthr_gz;
        fwdSamples += followSamples;
        fthr_gx = fthr_gy = fthr_gz = 0;
        followSamples = 0;
      }

      publishSwingAnalytics();
      blinkLED(1, 60);
      digitalWrite(LED_BUILTIN, HIGH);
    }

    // ── Stream CSV row ────────────────────────────────────────
    // Columns: timestamp_ms,ax,ay,az,gx,gy,gz,magnitude,whip
    // Matches CLAUDE.md required schema (timestamp_ms first).
    unsigned long ts = millis() - sessionStart;
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

    delay(20); // 50 Hz
  }

  // ── Disconnected ───────────────────────────────────────────
  recording   = false;
  swingActive = false;
  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Disconnected");
  blinkLED(5, 100);
  BLE.advertise();
}
