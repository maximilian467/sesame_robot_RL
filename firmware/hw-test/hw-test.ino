// Sesame hardware test for my build: ESP32-S3 SuperMini + PCA9685 + BNO08x + SSD1306.
//
// The IMU board is sold as "GY-BNO055" but carries a BNO080/BNO085, which
// speaks a different protocol (SHTP) at 0x4A/0x4B. Its two PS0/PS1 solder
// jumpers must be closed for I2C (see docs/hardware/bringup.md).
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
// SparkFun's BNO080 library uses fixed SHTP channel numbers. The official
// CEVA driver (also inside Adafruit_BNO08x) waits for an advertisement that
// this chip doesn't send on its own after a reset, so it never connects.
#include <SparkFun_BNO080_Arduino_Library.h>

// ======================================================================
// --- CONFIGURATION ---
// ======================================================================

// I2C pins on the ESP32-S3 SuperMini (Arduino core defaults for the S3).
#define I2C_SDA 8
#define I2C_SCL 9
// 100 kHz is forgiving with long hand-soldered wires; 400 kHz only matters later
// for fast OLED updates.
#define I2C_HZ 100000

#define ADDR_PCA9685 0x40
#define ADDR_IMU     0x4B  // BNO08x with ADD high; 0x4A with ADD low
#define ADDR_OLED    0x3C  // some boards use 0x3D

// IMU report interval. The RL policy will run at 50 Hz (20 ms); 10 ms keeps
// the latest value fresh. The game rotation vector ignores the magnetometer,
// which the servo motors would disturb anyway.
const uint16_t IMU_REPORT_MS = 10;

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
BNO080 bno;

bool hasPca = false;
bool hasOled = false;
bool hasImu = false;

int lastAngle[8] = {-1, -1, -1, -1, -1, -1, -1, -1};  // -1 = limp

// Result of repeated I2C scans (defined up here so the Arduino IDE's
// auto-generated prototypes can see it).
struct ScanResult {
  int stable;  // expected devices that answered every round
  int noisy;   // other addresses that answered at least once
};

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

// Starts the IMU and its three reports. Returns false if it doesn't answer.
bool imuBegin() {
  if (!i2cPresent(ADDR_IMU) || !bno.begin(ADDR_IMU, Wire)) return false;  // soft reset + product ID check
  // Back-to-back enable commands get lost (the chip dropped the second one),
  // so give it time and read its replies after each. No gravity report: on
  // this chip it stays at a placeholder (0, 0, 9.80); projectedGravity()
  // derives it from the orientation instead.
  bno.enableGameRotationVector(IMU_REPORT_MS);
  delay(50);
  imuPoll();
  bno.enableGyro(IMU_REPORT_MS);
  delay(50);
  imuPoll();
  return true;
}

// Gravity direction in the IMU's own frame, from the orientation quaternion:
// the world "up" axis rotated into the body frame, scaled to 9.81 m/s2. Lying
// flat gives (0, 0, 9.81). This is the "projected gravity" an RL policy sees
// in simulation, and it doesn't inherit the accelerometer's scale error.
void projectedGravity(float& gx, float& gy, float& gz) {
  float w = bno.getQuatReal(), x = bno.getQuatI(), y = bno.getQuatJ(), z = bno.getQuatK();
  gx = 9.81 * 2 * (x * z - w * y);
  gy = 9.81 * 2 * (y * z + w * x);
  gz = 9.81 * (w * w - x * x - y * y + z * z);
}

// Length of the orientation quaternion: 1.0 for a valid one, 0 before the
// first report arrived.
float quatNorm() {
  float w = bno.getQuatReal(), x = bno.getQuatI(), y = bno.getQuatJ(), z = bno.getQuatK();
  return sqrt(w * w + x * x + y * y + z * z);
}

// The BNO08x pushes reports on its own; read everything that is queued so the
// getters return the newest values. Returns how many reports were read.
int imuPoll() {
  int n = 0;
  while (n < 20 && bno.dataAvailable()) n++;
  return n;
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
    if (addr < 0x08 || addr > 0x77) name = "RESERVED -> not a real device, bus is noisy (run 'i2cdiag')";
    else if (addr == ADDR_PCA9685) name = "PCA9685 servo driver";
    else if (addr == 0x70) name = "PCA9685 all-call address (normal)";
    else if (addr == 0x4A || addr == 0x4B) name = "BNO08x IMU";
    else if (addr == 0x28 || addr == 0x29) name = "BNO055 IMU (not what this firmware expects)";
    else if (addr == 0x3C || addr == 0x3D) name = "SSD1306 OLED";
    Serial.printf("  0x%02X  %s\n", addr, name);
  }
  if (found == 0) Serial.println("  nothing found -> check SDA/SCL, 3V3 and GND");
}

const uint8_t expectedAddr[3] = {ADDR_PCA9685, ADDR_IMU, ADDR_OLED};
const char* expectedName[3] = {"PCA9685", "BNO08x", "OLED"};
const int SCAN_ROUNDS = 5;

// Scans SCAN_ROUNDS times and prints how often each address answered.
// A real device answers every time; noise answers at random addresses.
ScanResult scanRounds(int sda, int scl, uint32_t hz) {
  Wire.end();
  Wire.begin(sda, scl);
  Wire.setClock(hz);
  uint8_t hits[128] = {0};
  for (int r = 0; r < SCAN_ROUNDS; r++) {
    for (uint8_t addr = 1; addr < 127; addr++) {
      if (i2cPresent(addr)) hits[addr]++;
    }
  }
  Serial.printf("  SDA=%d SCL=%d at %lu Hz, %d rounds:\n", sda, scl, (unsigned long)Wire.getClock(), SCAN_ROUNDS);
  int noisy = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    if (hits[addr] == 0) continue;
    bool expected = false;
    for (int i = 0; i < 3; i++) {
      if (addr == expectedAddr[i]) {
        Serial.printf("    0x%02X  %d/%d  %s\n", addr, hits[addr], SCAN_ROUNDS, expectedName[i]);
        expected = true;
      }
    }
    if (!expected && addr == 0x70) {
      Serial.printf("    0x%02X  %d/%d  PCA9685 all-call\n", addr, hits[addr], SCAN_ROUNDS);
    } else if (!expected) {
      noisy++;
    }
  }
  int stable = 0;
  for (int i = 0; i < 3; i++) {
    if (hits[expectedAddr[i]] == SCAN_ROUNDS) stable++;
    else Serial.printf("    missing or flaky: %s (0x%02X)\n", expectedName[i], expectedAddr[i]);
  }
  Serial.printf("    + %d other address(es) answered = noise\n", noisy);
  return {stable, noisy};
}

// Samples both lines without driving them and reports how often they were LOW
// and how often they changed. Only the ESP32 may start traffic on I2C, so
// with the ESP32 silent both lines must stay HIGH.
void sampleIdle(const char* label) {
  const uint32_t samples = 200000;
  uint32_t sdaLow = 0, sclLow = 0, sdaEdges = 0, sclEdges = 0;
  int lastSda = digitalRead(I2C_SDA);
  int lastScl = digitalRead(I2C_SCL);
  unsigned long t0 = micros();
  for (uint32_t i = 0; i < samples; i++) {
    int sda = digitalRead(I2C_SDA);
    int scl = digitalRead(I2C_SCL);
    if (!sda) sdaLow++;
    if (!scl) sclLow++;
    if (sda != lastSda) sdaEdges++;
    if (scl != lastScl) sclEdges++;
    lastSda = sda;
    lastScl = scl;
  }
  unsigned long ms = (micros() - t0) / 1000;
  Serial.printf("  %-22s SDA low %5.1f%% edges %6lu | SCL low %5.1f%% edges %6lu  (%lu ms)\n", label,
                100.0 * sdaLow / samples, (unsigned long)sdaEdges, 100.0 * sclLow / samples,
                (unsigned long)sclEdges, ms);
}

void cmdBusIdle() {
  Serial.println("=== Bus idle listen (ESP32 silent, should be 0% low, 0 edges) ===");
  Wire.end();
  pinMode(I2C_SDA, INPUT);
  pinMode(I2C_SCL, INPUT);
  delay(100);
  sampleIdle("quiet for 100 ms:");

  // Talk briefly, then listen right away: a device that keeps driving the
  // lines after a transaction (stuck, or not in I2C mode) shows up here.
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_HZ);
  for (uint8_t addr = 0x08; addr < 0x78; addr++) i2cPresent(addr);
  Wire.end();
  pinMode(I2C_SDA, INPUT);
  pinMode(I2C_SCL, INPUT);
  sampleIdle("right after traffic:");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_HZ);
}

// Finds out *why* the bus doesn't work, without a multimeter.
void cmdI2cDiag() {
  Serial.println("=== I2C diagnosis ===");

  // 1) Idle levels. Each board has pull-up resistors to 3V3 on SDA and SCL.
  //    With the ESP32's weak internal pull-DOWN on, a connected pull-up still
  //    wins and the pin reads 1. A pin that reads 0 is not connected to any
  //    pull-up (wrong pin, broken joint) or is shorted to GND.
  Wire.end();
  const int pins[2] = {I2C_SDA, I2C_SCL};
  const char* names[2] = {"SDA", "SCL"};
  bool pullupsOk = true;
  for (int i = 0; i < 2; i++) {
    pinMode(pins[i], INPUT_PULLDOWN);
    delay(5);
    int withDown = digitalRead(pins[i]);
    pinMode(pins[i], INPUT_PULLUP);
    delay(5);
    int withUp = digitalRead(pins[i]);
    pinMode(pins[i], INPUT);
    const char* verdict;
    if (withDown == 1) verdict = "OK, external pull-up connected";
    else if (withUp == 1) verdict = "NO external pull-up reaches this pin -> wrong pin or broken joint";
    else verdict = "stuck LOW even with pull-up -> short to GND or a board holds it low";
    if (withDown != 1) pullupsOk = false;
    Serial.printf("  %s (GPIO %d): %s\n", names[i], pins[i], verdict);
  }

  // 2) Repeated scans: as wired at normal speed, very slow, and swapped.
  //    If the devices show up cleanly only when slow, the wiring is right
  //    but the signal edges are bad (long/parallel wires, weak joint).
  ScanResult fast = scanRounds(I2C_SDA, I2C_SCL, I2C_HZ);
  ScanResult slow = scanRounds(I2C_SDA, I2C_SCL, 10000);
  ScanResult swapped = scanRounds(I2C_SCL, I2C_SDA, 10000);

  Serial.println("  Verdict:");
  if (fast.noisy == 0 && fast.stable == 3) {
    Serial.println("    Bus OK. Press RST so the firmware picks the devices up.");
  } else if (fast.noisy == 0 && fast.stable > 0) {
    Serial.println("    Bus clean, but a device is missing (see above) -> check that board's VIN/GND and its SDA/SCL joints.");
  } else if (slow.noisy == 0 && slow.stable == 3) {
    Serial.println("    Clean only at 10 kHz -> wiring right, signal quality poor. Send me this output.");
  } else if (swapped.stable > 0) {
    Serial.println("    Devices answer with SDA/SCL swapped -> swap the two wires.");
  } else if (!pullupsOk) {
    Serial.println("    A line has no pull-up -> see the line marked above.");
  } else {
    Serial.println("    Noisy bus -> a board drives the lines. Run 'busidle', then disconnect boards one by one.");
  }

  // Back to the normal configuration.
  Wire.end();
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_HZ);
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
    Serial.println("Error: IMU not found.");
    return;
  }
  Serial.println("IMU stream, 5 Hz. Tilt the board. Press Enter to stop.");
  Serial.println("grav = projected gravity from the orientation; acc = accuracy 0..3 of rotation/gyro");
  unsigned long last = 0;
  while (!keyPressed()) {
    imuPoll();
    if (millis() - last < 200) continue;
    last = millis();
    float roll = degrees(bno.getRoll()), pitch = degrees(bno.getPitch()), yaw = degrees(bno.getYaw());
    float gx, gy, gz;
    projectedGravity(gx, gy, gz);
    Serial.printf("roll %6.1f  pitch %6.1f  yaw %7.1f | grav %5.2f %5.2f %5.2f m/s2 | "
                  "gyro %6.1f %6.1f %6.1f deg/s | acc %d/%d\n",
                  roll, pitch, yaw, gx, gy, gz, degrees(bno.getGyroX()), degrees(bno.getGyroY()),
                  degrees(bno.getGyroZ()), bno.getQuatAccuracy(), bno.getGyroAccuracy());
    oledShow("IMU", "roll  " + String(roll, 1), "pitch " + String(pitch, 1));
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
  if (!hasImu) hasImu = imuBegin();  // e.g. after fixing a joint without a reset
  bool imuOk = hasImu;
  bool still = false;
  if (imuOk) {
    // Collect reports for half a second (two reports every 10 ms; my board
    // delivers ~77), and the orientation must be a unit quaternion.
    int reports = 0;
    unsigned long t0 = millis();
    while (millis() - t0 < 500) reports += imuPoll();
    float n = quatNorm();
    still = reports > 50 && n > 0.95 && n < 1.05;
    Serial.printf("  %d reports in 0.5 s, quaternion length %.3f (expect 1.000)\n", reports, n);
  }
  Serial.printf("  [%s] PCA9685 at 0x%02X\n", pca ? "PASS" : "FAIL", ADDR_PCA9685);
  Serial.printf("  [%s] OLED at 0x%02X\n", oled ? "PASS" : "FAIL", ADDR_OLED);
  Serial.printf("  [%s] BNO08x at 0x%02X responds\n", imuOk ? "PASS" : "FAIL", ADDR_IMU);
  Serial.printf("  [%s] BNO08x streams a valid orientation\n", still ? "PASS" : "FAIL");
  bool all = pca && oled && imuOk && still;
  Serial.println(all ? "=== ALL PASS ===" : "=== SOME FAILED -> see docs/hardware/bringup.md ===");
  oledShow(all ? "ALL PASS" : "FAIL",
           String("PCA ") + (pca ? "ok" : "--") + "  OLED " + (oled ? "ok" : "--"),
           String("IMU ") + (imuOk && still ? "ok" : "--"));
}

void cmdStatus() {
  Serial.printf("PCA9685: %s  OLED: %s  BNO08x: %s\n", hasPca ? "yes" : "NO",
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
  Serial.println(" selftest       check PCA9685, OLED, BNO08x (no movement)");
  Serial.println(" scan           list all I2C devices");
  Serial.println(" i2cdiag        find out why the I2C bus fails");
  Serial.println(" busidle        listen to the bus while the ESP32 is silent");
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
  delay(300);  // let the IMU finish booting after power-on

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_HZ);
  Wire.setTimeOut(10);  // ms; a stuck bus otherwise makes every scan take minutes

  hasPca = i2cPresent(ADDR_PCA9685);
  if (hasPca) {
    pwm.begin();
    pwm.setOscillatorFrequency(PCA9685_OSC_HZ);
    pwm.setPWMFreq(50);  // standard servo frame: one pulse every 20 ms
    allOff();            // a reset of the ESP32 alone does not reset the PCA9685
  }

  hasOled = i2cPresent(ADDR_OLED) && display.begin(SSD1306_SWITCHCAPVCC, ADDR_OLED);
  hasImu = imuBegin();

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
  else if (input == "i2cdiag") cmdI2cDiag();
  else if (input == "busidle") cmdBusIdle();
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
