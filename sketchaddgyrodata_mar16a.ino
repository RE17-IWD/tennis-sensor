#include <Arduino_BMI270_BMM150.h>
#include <ArduinoBLE.h>

// BLE Service and Characteristic
BLEService motionService("180C");
BLEStringCharacteristic motionChar("2A56", BLERead | BLENotify, 20);

void classifyMotion(float x, float y, float z) {
  float mag = sqrt(x*x + y*y + z*z);

  if (mag < 0.2)        Serial.println("STATIONARY");
  else if (z > 0.8)     Serial.println("FACE_UP");
  else if (z < -0.8)    Serial.println("FACE_DOWN");
  else if (x > 0.8)     Serial.println("TILT_RIGHT");
  else if (x < -0.8)    Serial.println("TILT_LEFT");
  else                  Serial.println("MOVING");
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // IMU Init
  if (!IMU.begin()) {
    Serial.println("IMU failed");
    while (1);
  }

  // BLE Init
  if (!BLE.begin()) {
    Serial.println("BLE failed");
    while (1);
  }

  BLE.setLocalName("MotionSensor");
  motionService.addCharacteristic(motionChar);
  BLE.addService(motionService);
  BLE.advertise();

  Serial.println("READY");
}

void loop() {
  BLE.poll();

  float x, y, z;
  float gx, gy, gz;

  // Read Accelerometer
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);

    // Print raw accel data
    Serial.print("ACCEL: ");
    Serial.print(x); Serial.print(",");
    Serial.print(y); Serial.print(",");
    Serial.println(z);

    // Detect motion magnitude
    float magnitude = sqrt(x*x + y*y + z*z);
    if (magnitude > 1.5) {
      Serial.println("MOVEMENT_DETECTED");
    } else {
      Serial.println("STILL");
    }

    // Classify and send over BLE
    String label = "";
    if (magnitude < 0.2)        label = "STATIONARY";
    else if (z > 0.8)           label = "FACE_UP";
    else if (z < -0.8)          label = "FACE_DOWN";
    else if (x > 0.8)           label = "TILT_RIGHT";
    else if (x < -0.8)          label = "TILT_LEFT";
    else                        label = "MOVING";

    Serial.println(label);
    motionChar.writeValue(label);
  }

  // Read Gyroscope
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    Serial.print("GYRO: ");
    Serial.print(gx); Serial.print(",");
    Serial.print(gy); Serial.print(",");
    Serial.println(gz);
  }

  delay(20);
}