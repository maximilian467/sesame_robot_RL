# Hardware bring-up

How I check every part of my build before (and while) assembling it. The firmware is [`firmware/hw-test`](../../firmware/hw-test/hw-test.ino).

The mechanical assembly follows the upstream [build guide](https://github.com/dorianborian/sesame-robot/blob/main/docs/build-guide/README.md) and its [video](https://www.youtube.com/watch?v=NIgoQVQF_Ng). The electronics differ (PCA9685 instead of servo pins on the ESP32, plus a BNO055), so the upstream wiring guide does not apply. This page replaces it.

> [!IMPORTANT]
> The test is not only for the finished robot. Upstream calibration needs the servos running **before** the hip and leg joints go on: each joint is fitted while its servo holds 90°. So flash this firmware after the pre-assembly (upstream Phase 3) and before "Attaching Hip & Leg Joints".

## Wiring

All three I2C devices share one bus. They run on 3.3 V logic; the servos get their own 5 V from the UBEC.

| From | To | Notes |
|---|---|---|
| ESP32 GPIO 8 | SDA on PCA9685, BNO055, OLED | I2C data |
| ESP32 GPIO 9 | SCL on PCA9685, BNO055, OLED | I2C clock |
| ESP32 3V3 | VCC on PCA9685, VIN on BNO055, VCC on OLED | logic power only |
| ESP32 GND | GND on all three boards **and** UBEC GND | one common ground |
| UBEC 5 V out | PCA9685 **V+** screw terminal | servo power |
| Servo 0–7 | PCA9685 channels 0–7 | brown = GND, red = V+, orange = signal |

Servo order is upstream's, so its angle diagram and poses apply:

| Index | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 |
|---|---|---|---|---|---|---|---|---|
| Joint | R1 | R2 | L1 | L2 | R4 | R3 | L3 | L4 |
| Type | hip | hip | hip | hip | leg | leg | leg | leg |

> [!CAUTION]
> - Never connect the UBEC to the PCA9685's **VCC** pin. VCC is the 3.3 V logic supply; 5 V there also lifts the I2C lines to 5 V and can damage the ESP32.
> - During bring-up, power the ESP32 from USB only. Don't also feed the UBEC into the ESP32's 5V pin.
> - Set the UBEC jumper to **5 V** and measure its output with nothing connected before wiring it in.

## Flashing

PlatformIO (installed with `uv tool install platformio`):

```bash
cd firmware/hw-test
pio run -t upload
pio device monitor
```

Arduino IDE also works: open `hw-test.ino`, board "ESP32S3 Dev Module", *USB CDC On Boot: Enabled*, flash size 4 MB, and install the libraries listed in `platformio.ini`.

If the upload doesn't find the board: hold **BOOT**, tap **RST**, release BOOT, upload again.

## Test sequence

Each step adds one thing. If a step fails, the cause is in what was just added.

| Step | Setup | Command | Expected |
|---|---|---|---|
| 1 | ESP32 alone on USB | (boot) | help text; `PCA9685: NO  OLED: NO  BNO055: NO` |
| 2 | + I2C boards, no servo power | `scan` | 0x28 BNO055, 0x3C OLED, 0x40 PCA9685 (0x70 is normal too) |
| 3 | same | `selftest` | `=== ALL PASS ===`, also on the OLED |
| 4 | same | `oled` | full white, then a frame, then text |
| 5 | same | `imu` | tilting the board changes roll/pitch; lying flat, gravity is ~9.8 on one axis |
| 6 | + UBEC on V+, **one** servo on channel 0, no horn | `0,90`, `sweep 0`, `stop` | servo moves, sweeps smoothly, goes limp |
| 7 | repeat 6 for each servo 0–7 | | catches dead servos before they're built in |
| 8 | all 8 servos, still no joints | `rest`, then `stand`, `rest` | all move, no brownout/reset |
| 9 | calibration | follow upstream "Attaching Hip & Leg Joints" with `rest`/`stand` | hips parallel at rest, X shape at stand |

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| `scan` finds nothing | SDA/SCL swapped, missing 3V3 or GND on the boards |
| `scan` finds 0x29 or 0x3D | BNO055 ADR pin high / OLED jumper set; change the address at the top of `hw-test.ino` |
| BNO055 found by `scan` but selftest FAIL | wrong chip ID (clone?) or it needs a second after power-on; type `selftest` again |
| servo doesn't move, PCA9685 found | no 5 V on V+, or UBEC GND not tied to ESP32 GND |
| ESP32 resets when servos move | supply sags; check UBEC wiring, test with fewer servos |
| servo jitters at 90° | normal for cheap MG90S under no load; worry only if it buzzes hot |
