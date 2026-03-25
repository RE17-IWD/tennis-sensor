# Tennis Swing Analyzer

A wrist/racket-mounted IMU sensor system for real-time tennis swing analysis. An Arduino Nano 33 BLE captures motion data from a BMI270 IMU and streams it over Bluetooth. Python scripts post-process the data into PDF reports and ML-ready feature tables.

**Endgame:** An AI/ML-powered coaching tool capable of classifying shot types, measuring technique metrics, and tracking player improvement over time.

---

## Demo

> Sensor mounted at the grip → swing the racket → instant path feedback via LED blinks → full analytics over BLE.

```
SWING_START:1
SWING_ANALYTICS,1,620,3.84,412,47.3,18.2,94,41,−38,1.14,62.4,0,TOPSPIN
SWING_END:1
```

---

## Hardware

| Component | Details |
|-----------|---------|
| Board | Arduino Nano 33 BLE |
| IMU | BMI270 (accel + gyro) + BMM150 (mag) via `Arduino_BMI270_BMM150` |
| BLE | Built-in nRF52840, 64 MHz |
| Sampling Rate | ~50 Hz (20 ms loop) |
| Placement | Racket handle / grip |

### IMU Axis Convention

```
X = along handle (toward tip)       gx = wrist pronation / supination
Y = perpendicular to string face    gy > 0 = upward path (topspin)
Z = across string face (lateral)    gz = main forward-swing rotation
```

---

## Features

### Firmware (`sketch_swing_path_v2`)
- **Buffered swing analysis** — all IMU samples stored per swing; full post-swing analysis finds the exact impact frame
- **Measured DT integration** — uses actual `millis()` delta per sample, not an assumed 20 ms constant
- **Gyro bias correction** — rolling quiet-period average subtracted from every angular integration step, reducing drift
- **Dual trigger** — requires both accel AND gyro threshold to open a swing window (eliminates false triggers from drops/bumps)
- **Wrist roll at impact** — instantaneous `gx` at the exact peak-acceleration sample
- **Follow-through ratio** — follow-arc / forward-arc; detects blocked shots vs full swings
- **Swing tempo** — interval between consecutive swing starts (rally cadence)
- **Racket head speed** — peak angular velocity × arm length → mph
- **LED path feedback** — 1 blink = flat, 2 = topspin, 3 = slice; instant court-side feedback
- **Standalone mode** — 3-second USB timeout; works as a wearable without a phone or computer

### Python Analysis (`analysis/main2.py`)
- Per-swing feature extraction: duration, peak/mean accel, whip metrics, path angle, power, gyro axes, sharpness, fatigue proxy
- Shot classification: forehand / backhand / serve / volley / unknown
- Spin classification: topspin / slice / flat / mixed + estimated RPM
- PDF report with trajectory plots, whip deep-dive, consistency analysis, ML feature space, multi-session trends
- Exports `tennis_features.csv` for downstream ML training

---

## Swing Metrics

| Metric | Description |
|--------|-------------|
| `attack_deg` | Vertical swing plane angle — positive = upward (topspin), negative = downward (slice) |
| `h_arc_deg` | Total horizontal arc (gz integral, forward + follow) |
| `v_arc_deg` | Total vertical arc (gy integral) |
| `spin_roll_dps` | Wrist roll rate at contact — positive = rolling over (topspin), negative = cutting under (slice) |
| `ft_ratio` | Follow-through / forward arc ratio — >1.2 is a full power stroke |
| `head_mph` | Racket head speed estimate from peak angular velocity |
| `max_whip` | Peak explosive snap rate (g/s) — proxy for contact quality |
| `tempo_ms` | Time between this swing start and the previous one |
| `path` | `LOW_TO_HIGH` / `TOPSPIN` / `FLAT` / `SLICE` / `HIGH_TO_LOW` |

### Path Angle Thresholds

```
> +25°   →  LOW_TO_HIGH
+8–25°   →  TOPSPIN
-8–+8°   →  FLAT
-8–-25°  →  SLICE
< -25°   →  HIGH_TO_LOW
```

---

## BLE Protocol

**Service UUID:** `180C`

| Characteristic | UUID | Direction | Description |
|----------------|------|-----------|-------------|
| `dataChar` | `2A56` | Read + Notify | Raw IMU CSV row, every sample |
| `statusChar` | `2A57` | Read + Notify | State events |
| `commandChar` | `2A58` | Write | `START` or `STOP` |
| `analyticsChar` | `2A59` | Read + Notify | Per-swing summary |

### Status Events

```
CONNECTED
SESSION_START
SWING_START:<n>
SWING_END:<n>
SWING_ANALYTICS,<n>,<dur_ms>,<peak_g>,<peak_dps>,<head_mph>,
  <attack_deg>,<h_arc_deg>,<v_arc_deg>,<spin_roll_dps>,
  <ft_ratio>,<max_whip>,<tempo_ms>,<path_label>
SESSION_END,SWINGS:<n>
```

---

## CSV Schema

Output file: `tennis_session_YYYY-MM-DD_HH-MM.csv`

| Column | Unit | Notes |
|--------|------|-------|
| `timestamp` | ms | Device uptime from boot — normalize to 0 in Python |
| `ax` | g | Acceleration X |
| `ay` | g | Acceleration Y |
| `az` | g | Acceleration Z |
| `gx` | °/s | Gyroscope X (wrist pronation axis) |
| `gy` | °/s | Gyroscope Y (topspin/slice axis) |
| `gz` | °/s | Gyroscope Z (lateral / forward swing) |
| `magnitude` | g | `sqrt(ax²+ay²+az²)` |
| `whip` | g/s | Smoothed rate-of-change of magnitude |

> **Timestamp normalization:** `df['time_s'] = (df['timestamp'] - df['timestamp'].iloc[0]) / 1000.0`

---

## File Structure

```
Arduino/
├── README.md
├── sketch_swing_path_v2/
│   └── sketch_swing_path_v2.ino     ← current firmware (USE THIS)
├── sketch_swing_path_v1/
│   └── sketch_swing_path_v1.ino     ← previous version (gyro integration, no buffer)
├── sketch_mar17whip_effectb/
│   ├── sketch_mar18_timestamped.ino ← older stable (averaging, not integration)
│   └── sketch_mar17whip_effectb.ino ← oldest
└── analysis/
    ├── main2.py                     ← analysis engine v2 (USE THIS)
    └── main3.py                     ← analysis engine v1
```

---

## Getting Started

### 1. Install Arduino Libraries

In the Arduino IDE, install via **Sketch → Include Library → Manage Libraries**:

```
Arduino_BMI270_BMM150
ArduinoBLE
```

### 2. Flash Firmware

Open `sketch_swing_path_v2/sketch_swing_path_v2.ino` in the Arduino IDE, select **Arduino Nano 33 BLE**, and upload.

Or via `arduino-cli`:

```bash
arduino-cli compile --fqbn arduino:mbed_nano:nano33ble sketch_swing_path_v2/
arduino-cli upload  --fqbn arduino:mbed_nano:nano33ble -p /dev/ttyACM0 sketch_swing_path_v2/
```

The onboard LED will blink 3 times slowly when ready.

### 3. Install Python Dependencies

```bash
pip install pandas numpy matplotlib scipy
```

### 4. Run a Session

1. Connect to **TennisSensor** (BLE service `180C`) from your phone or a BLE-capable laptop
2. Write `START` to characteristic `2A58` to begin recording
3. Swing the racket — the LED will blink after each swing (1 = flat, 2 = topspin, 3 = slice)
4. Write `STOP` to `2A58` to end the session
5. Save the streamed `dataChar` rows to a CSV file

### 5. Analyze

```bash
python analysis/main2.py data/tennis_session_2026-03-24.csv
# Outputs: tennis_analysis_v2_report.pdf + tennis_features.csv
```

Multi-session analysis:

```bash
python analysis/main2.py data/tennis_session_2026-03-*.csv
```

---

## Tunable Parameters

These live at the top of the firmware sketch. Adjust per player and racket weight.

```cpp
const float         SWING_START_ACCEL = 1.5f;  // g   — raise to reduce false positives
const float         SWING_END_ACCEL   = 1.15f; // g   — raise to shorten swing windows
const float         SWING_START_GYRO  = 80.0f; // °/s — raise on heavier rackets
const int           MIN_SWING_SAMPLES = 8;     // minimum samples (~160ms) to count as a swing
const unsigned long SWING_COOLDOWN    = 1500;  // ms between detected swings
const float         RACKET_ARM_M      = 0.65f; // m — grip-to-head distance for head speed calc
```

---

## Firmware Version History

| Sketch | Key Changes |
|--------|-------------|
| `sketch_swing_path_v2` | Buffered post-swing analysis, measured DT, gyro bias correction, wrist roll at impact, swing tempo, LED path feedback |
| `sketch_swing_path_v1` | Gyro integration (not averaging), dual accel+gyro trigger, follow-through ratio, head speed, non-blocking Serial |
| `sketch_mar18_timestamped` | Added timestamp column, fixed whip window reset on START |
| `sketch_mar17whip_effectb` | First whip effect implementation |

---

## Roadmap

### Firmware
- [ ] Magnetometer fusion (BMM150 — anchor orientation to world frame, fix drift)
- [ ] Quaternion / Euler angle output (full 3D orientation per sample)
- [ ] Impact detection event (sharp multi-axis spike at contact moment)
- [ ] Forehand vs backhand detection from initial orientation at swing start
- [ ] Binary BLE packet (packed floats — lower latency, higher throughput)

### Python / Analysis
- [ ] `ble_collector.py` — real-time BLE → CSV logger (`bleak`)
- [ ] `ml_classifier.py` — supervised shot-type classifier on `tennis_features.csv`
- [ ] `live_dashboard.py` — real-time BLE → live matplotlib / web visualization
- [ ] `fatigue_model.py` — track metric degradation across a session
- [ ] `coaching_feedback.py` — LLM-generated coaching cues from feature vectors
- [ ] `tempo_analysis.py` — rally cadence and stroke-rate stats
- [ ] `session_aggregator.py` — per-set / per-game aggregation

---

## Known Limitations

- Swing thresholds need tuning per player — a junior with a light racket will need lower values than an adult hitting hard
- Shot classifier in `main2.py` is rule-based — will misclassify unusual or ambiguous swings
- `spin_rpm` is a rough estimate, not calibrated against ball spin measurement
- No magnetometer fusion yet — gyro orientation drifts during long sessions
- Racket head speed assumes sensor is at the grip; adjust `RACKET_ARM_M` if remounted higher

---

## Dependencies

| Dependency | Version | Purpose |
|------------|---------|---------|
| `Arduino_BMI270_BMM150` | latest | IMU driver |
| `ArduinoBLE` | latest | BLE peripheral stack |
| `pandas` | any | CSV loading and feature extraction |
| `numpy` | ≥2.0 recommended | Numerical processing |
| `matplotlib` | any | PDF report plots |
| `scipy` | any | Signal filtering (`savgol_filter`) |
| `bleak` | any | *(future)* BLE collector script |
| `scikit-learn` | any | *(future)* ML classifier |
