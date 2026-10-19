// Sesame hardware test for my build: ESP32-S3 SuperMini + PCA9685 + BNO055 + SSD1306.
//
// A serial command line (115200 baud) to check every part on its own before
// the joints go on. Servos stay limp until you command them.
//
// Works with PlatformIO (`pio run -t upload`) and Arduino IDE (open this .ino).
// Servo numbering, rest/stand angles and the degree-to-pulse mapping follow
// the upstream firmware (dorianborian/sesame-robot), so its calibration guide
// applies unchanged.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BNO055.h>

// ======================================================================
// --- CONFIGURATION ---
// ======================================================================

// I2C pins on the ESP32-S3 SuperMini (Arduino core defaults for the S3).
#define I2C_SDA 8
#define I2C_SCL 9

#define ADDR_PCA9685 0x40
#define ADDR_BNO055  0x28  // 0x29 if the ADR pin is pulled high
#define ADDR_OLED    0x3C  // some boards use 0x3D

// Servo index -> PCA9685 channel. Index order is upstream's:
//   0=R1  1=R2  2=L1  3=L2  (hips)   4=R4  5=R3  6=L3  7=L4  (legs)
const uint8_t servoChannel[8] = {0, 1, 2, 3, 4, 5, 6, 7};
const char* servoNames[8] = {"R1", "R2", "L1", "L2", "R4", "R3", "L3", "L4"};

// Same mapping as upstream: 0..180 deg -> 732..2929 us.
const int MIN_PULSE_US = 732;
const int MAX_PULSE_US = 2929;

// Nominal PCA9685 oscillator. Cheap boards are often off by a few percent;
// this shifts every pulse by the same factor, which calibration absorbs.
const uint32_t PCA9685_OSC_HZ = 25000000;

// Pause between servo commands, so 8 servos never start at the same instant
// and pull the supply down (upstream calls this motorCurrentDelay).
const int MOTOR_DELAY_MS = 20;

// ======================================================================

Adafruit_PWMServoDriver pwm(ADDR_PCA9685, Wire);
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_BNO055 bno(55, ADDR_BNO055, &Wire);

bool hasPca = false;
bool hasOled = false;
bool hasImu = false;

int lastAngle[8] = {-1, -1, -1, -1, -1, -1, -1, -1};  // -1 = limp

// ======================================================================
// --- HELPERS ---
// ======================================================================

bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void oledShow(const String& line1, const String& line2 = "", const String& line3 = "") {
  if (!hasOled) return;
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(2);
  display.setCursor(0, 0);
  display.println(line1);
  display.setTextSize(1);
  display.setCursor(0, 28);
  display.println(line2);
  display.setCursor(0, 44);
  display.println(line3);
  display.display();
}

int angleToMicros(int angle) {
  return map(angle, 0, 180, MIN_PULSE_US, MAX_PULSE_US);
}

void setServoAngle(int id, int angle) {
  angle = constrain(angle, 0, 180);
  int us = angleToMicros(angle);
  pwm.writeMicroseconds(servoChannel[id], us);
  lastAngle[id] = angle;
  Serial.printf("OK: servo %d (%s) -> %d deg (%d us)\n", id, servoNames[id], angle, us);
  delay(MOTOR_DELAY_MS);
}

void servoOff(int id) {
  pwm.setPWM(servoChannel[id], 0, 4096);  // bit 12 = full off: no pulses, servo goes limp
  lastAngle[id] = -1;
}

void allOff() {
  for (int i = 0; i < 8; i++) servoOff(i);
  Serial.println("All servos OFF (limp).");
}

bool requirePca() {
  if (!hasPca) Serial.println("Error: PCA9685 not found. Run 'scan' and check the wiring.");
  return hasPca;
}

// True if a key was pressed; also drains the input so the next command starts clean.
bool keyPressed() {
  if (!Serial.available()) return false;
  while (Serial.available()) Serial.read();
  return true;
}

// ======================================================================
// --- TESTS ---
// ======================================================================

void cmdScan() {
  Serial.println("I2C scan:");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    if (!i2cPresent(addr)) continue;
    found++;
    const char* name = "unknown";
    if (addr == ADDR_PCA9685) name = "PCA9685 servo driver";
    else if (addr == 0x70) name = "PCA9685 all-call address (normal)";
    else if (addr == 0x28 || addr == 0x29) name = "BNO055 IMU";
    else if (addr == 0x3C || addr == 0x3D) name = "SSD1306 OLED";
    Serial.printf("  0x%02X  %s\n", addr, name);
  }
  if (found == 0) Serial.println("  nothing found -> check SDA/SCL, 3V3 and GND");
}

void cmdOled() {
  if (!hasOled) {
    Serial.println("Error: OLED not found.");
    return;
  }
  // Every pixel on, then off: catches dead rows/columns.
  display.fillScreen(SSD1306_WHITE);
  display.display();
  delay(700);
  display.clearDisplay();
  display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
  display.display();
  delay(700);
  oledShow("Hallo!", "OLED test OK", "Sesame hw-test");
  Serial.println("OLED: full white, frame, then text. Did you see all three?");
}

void cmdImu() {
  if (!hasImu) {
    Serial.println("Error: BNO055 not found.");
    return;
  }
  Serial.println("IMU stream, 5 Hz. Tilt the board. Press Enter to stop.");
  Serial.println("cal = sys/gyro/accel/mag, each 0..3 (3 = calibrated)");
  while (!keyPressed()) {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    uint8_t sys, g, a, m;
    bno.getCalibration(&sys, &g, &a, &m);
    Serial.printf("yaw %7.1f  roll %6.1f  pitch %6.1f | grav %5.2f %5.2f %5.2f m/s2 | "
                  "gyro %6.2f %6.2f %6.2f deg/s | cal %d/%d/%d/%d\n",
                  euler.x(), euler.y(), euler.z(), grav.x(), grav.y(), grav.z(),
                  gyro.x(), gyro.y(), gyro.z(), sys, g, a, m);
    oledShow("IMU", "roll  " + String(euler.y(), 1), "pitch " + String(euler.z(), 1));
    delay(200);
  }
  Serial.println("IMU stream stopped.");
}

void cmdSweep(int id) {
  if (!requirePca()) return;
  Serial.printf("Sweeping servo %d (%s) 60..120 deg. Press Enter to stop.\n", id, servoNames[id]);
  int angle = 90;
  int step = 2;
  while (!keyPressed()) {
    pwm.writeMicroseconds(servoChannel[id], angleToMicros(angle));
    angle += step;
    if (angle >= 120 || angle <= 60) step = -step;
    delay(30);
  }
  setServoAngle(id, 90);
}

// Upstream poses (movement-sequences.h). Only use them with the joints OFF,
// or after the hips were fitted at "rest".
void cmdRest() {
  if (!requirePca()) return;
  Serial.println("REST: all servos -> 90 deg");
  for (int i = 0; i < 8; i++) setServoAngle(i, 90);
  oledShow("REST");
}

void cmdStand() {
  if (!requirePca()) return;
  Serial.println("STAND");
  setServoAngle(0, 135);  // R1
  setServoAngle(1, 45);   // R2
  setServoAngle(2, 45);   // L1
  setServoAngle(3, 135);  // L2
  setServoAngle(4, 0);    // R4
  setServoAngle(5, 180);  // R3
  setServoAngle(6, 0);    // L3
  setServoAngle(7, 180);  // L4
  oledShow("STAND");
}

void cmdSelftest() {
  Serial.println("=== Selftest (no servo movement) ===");
  bool pca = i2cPresent(ADDR_PCA9685);
  bool oled = i2cPresent(ADDR_OLED);
  bool imuOk = hasImu;
  bool still = false;
  if (imuOk) {
    // Lying still on the table, gravity should be ~9.81 m/s2.
    imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
    float g = sqrt(grav.x() * grav.x() + grav.y() * grav.y() + grav.z() * grav.z());
    still = g > 9.0 && g < 10.6;
    Serial.printf("  gravity magnitude: %.2f m/s2 (expect ~9.81)\n", g);
  }
  Serial.printf("  [%s] PCA9685 at 0x%02X\n", pca ? "PASS" : "FAIL", ADDR_PCA9685);
  Serial.printf("  [%s] OLED at 0x%02X\n", oled ? "PASS" : "FAIL", ADDR_OLED);
  Serial.printf("  [%s] BNO055 at 0x%02X responds\n", imuOk ? "PASS" : "FAIL", ADDR_BNO055);
  Serial.printf("  [%s] BNO055 gravity plausible\n", still ? "PASS" : "FAIL");
  bool all = pca && oled && imuOk && still;
  Serial.println(all ? "=== ALL PASS ===" : "=== SOME FAILED -> see docs/hardware/bringup.md ===");
  oledShow(all ? "ALL PASS" : "FAIL",
           String("PCA ") + (pca ? "ok" : "--") + "  OLED " + (oled ? "ok" : "--"),
           String("IMU ") + (imuOk && still ? "ok" : "--"));
}

void cmdStatus() {
  Serial.printf("PCA9685: %s  OLED: %s  BNO055: %s\n", hasPca ? "yes" : "NO",
                hasOled ? "yes" : "NO", hasImu ? "yes" : "NO");
  for (int i = 0; i < 8; i++) {
    if (lastAngle[i] < 0) Serial.printf("  servo %d (%s): off\n", i, servoNames[i]);
    else Serial.printf("  servo %d (%s): %d deg\n", i, servoNames[i], lastAngle[i]);
  }
}

void printHelp() {
  Serial.println("-----------------------------------------------");
  Serial.println(" Sesame hw-test  (servos are OFF until commanded)");
  Serial.println("-----------------------------------------------");
  Serial.println(" selftest       check PCA9685, OLED, BNO055 (no movement)");
  Serial.println(" scan           list all I2C devices");
  Serial.println(" oled           display test pattern");
  Serial.println(" imu            stream orientation (Enter stops)");
  Serial.println(" <id>,<angle>   move one servo, e.g. 0,90 (id 0..7)");
  Serial.println(" all,<angle>    move all servos, e.g. all,90");
  Serial.println(" sweep <id>     slow back-and-forth 60..120 deg (Enter stops)");
  Serial.println(" rest | stand   upstream calibration poses");
  Serial.println(" stop           all servos limp");
  Serial.println(" status         devices found + last servo angles");
  Serial.println(" help           this list");
  Serial.println("-----------------------------------------------");
}

// ======================================================================
// --- SETUP / LOOP ---
// ======================================================================

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);  // USB CDC needs a moment
  delay(700);  // BNO055 needs ~650 ms after power-on

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  hasPca = i2cPresent(ADDR_PCA9685);
  if (hasPca) {
    pwm.begin();
    pwm.setOscillatorFrequency(PCA9685_OSC_HZ);
    pwm.setPWMFreq(50);  // standard servo frame: one pulse every 20 ms
    allOff();            // a reset of the ESP32 alone does not reset the PCA9685
  }

  hasOled = i2cPresent(ADDR_OLED) && display.begin(SSD1306_SWITCHCAPVCC, ADDR_OLED);
  hasImu = i2cPresent(ADDR_BNO055) && bno.begin();  // default mode: NDOF (sensor fusion)
  if (hasImu) bno.setExtCrystalUse(true);

  printHelp();
  cmdStatus();
  oledShow("hw-test", String("PCA ") + (hasPca ? "ok" : "--") + "  OLED ok",
           String("IMU ") + (hasImu ? "ok" : "--"));
}

void loop() {
  if (!Serial.available()) return;
  String input = Serial.readStringUntil('\n');
  input.trim();
  input.toLowerCase();
  if (input.length() == 0) return;

  if (input == "help") printHelp();
  else if (input == "selftest") cmdSelftest();
  else if (input == "scan") cmdScan();
  else if (input == "oled") cmdOled();
  else if (input == "imu") cmdImu();
  else if (input == "rest") cmdRest();
  else if (input == "stand") cmdStand();
  else if (input == "stop" || input == "off") allOff();
  else if (input == "status") cmdStatus();
  else if (input.startsWith("sweep")) {
    int id = input.substring(5).toInt();
    if (id < 0 || id > 7) Serial.println("Error: id must be 0..7");
    else cmdSweep(id);
  } else if (input.indexOf(',') > 0) {
    // "id,angle" or "all,angle" -- same format as upstream's motor tester
    int comma = input.indexOf(',');
    String target = input.substring(0, comma);
    int angle = constrain(input.substring(comma + 1).toInt(), 0, 180);
    if (!requirePca()) return;
    if (target == "all") {
      for (int i = 0; i < 8; i++) setServoAngle(i, angle);
    } else {
      int id = target.toInt();
      bool numeric = target.length() > 0 && isDigit(target[0]);
      if (!numeric || id < 0 || id > 7) Serial.println("Error: id must be 0..7");
      else setServoAngle(id, angle);
    }
  } else {
    Serial.println("Unknown command. Type 'help'.");
  }
}
