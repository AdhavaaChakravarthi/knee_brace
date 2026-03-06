# EMG + IMU Biosignal System — Master Context for GitHub Copilot
# Paste this entire file into a Copilot Chat or as a workspace instruction file (.github/copilot-instructions.md)

---

## PROJECT OVERVIEW

Building a wearable biosignal acquisition system for gesture/movement classification using a CNN.
The hardware produces a 21-row × 100-column matrix per window, streamed over WiFi WebSocket.

---

## HARDWARE

### Microcontroller
- ELEGOO ESP32 DevKit (38-pin WROOM-32, 2.54mm pitch)
- Arduino core v3.3.7 (v3.x API — timer API changed from v2.x)
- Running Arduino IDE

### EMG Frontend — 3× SparkFun AD8232 Breakout (DEV-12650)
- Fixed gain: 100×
- Onboard HPF ~0.5 Hz, LPF ~40 Hz
- Output connected to ESP32 ADC (INPUT ONLY pins)
- Powered at 3.3V — NOT 5V
- ADC configured: 12-bit resolution, ADC_11db attenuation

```
AD8232 #1 OUTPUT → GPIO34  (ADC1_CH6)
AD8232 #2 OUTPUT → GPIO35  (ADC1_CH7)
AD8232 #3 OUTPUT → GPIO32  (ADC1_CH4)
```

### IMU — 3× GY-521 MPU6050 Breakout
- Using hardware I2C buses only (SoftwareWire is INCOMPATIBLE with ESP32 core v3.x)
- All IMUs at address 0x68 or 0x69 depending on AD0 pin

```
Bus 0 (Wire)  SDA=GPIO21 SCL=GPIO22 → IMU1 @ 0x68 (AD0=GND)
                                     → IMU2 @ 0x69 (AD0=3V3)
Bus 1 (Wire1) SDA=GPIO25 SCL=GPIO26 → IMU3 @ 0x68 (AD0=GND)
```

- 4.7kΩ pullup resistors required on each SDA and SCL line to 3.3V (6 resistors total)
- MPU6050 registers used:
  - REG_PWR_MGMT_1 = 0x6B (wake: write 0x00)
  - REG_CONFIG     = 0x1A (DLPF: write 0x03 → ~44Hz LPF)
  - REG_ACCEL_XOUT = 0x3B (burst read 14 bytes: ax,ay,az,temp,gx,gy,gz)

---

## CNN INPUT MATRIX

Shape: 21 rows × 100 columns (WINDOW_SIZE=100 samples @ 500Hz = 200ms window)

```
Row 00  → EMG1              (normalised 0.0–1.0, per-window min-max)
Row 01  → IMU1_ax
Row 02  → IMU1_ay
Row 03  → IMU1_az
Row 04  → IMU1_gx
Row 05  → IMU1_gy
Row 06  → IMU1_gz
Row 07  → EMG2
Row 08  → IMU2_ax
Row 09  → IMU2_ay
Row 10  → IMU2_az
Row 11  → IMU2_gx
Row 12  → IMU2_gy
Row 13  → IMU2_gz
Row 14  → EMG3
Row 15  → IMU3_ax
Row 16  → IMU3_ay
Row 17  → IMU3_az
Row 18  → IMU3_gx
Row 19  → IMU3_gy
Row 20  → IMU3_gz
```

Normalisation: per-row, per-window min-max to [0.0, 1.0].
Absolute values do not matter — CNN reads waveform morphology only.

---

## FIRMWARE ARCHITECTURE

### Sampling
- Hardware timer ISR fires every 2ms (500Hz)
- ISR sets a boolean flag ONLY — no I2C inside ISR (Wire uses interrupts internally)
- loop() checks flag, reads all sensors, fills circular buffer
- After WINDOW_SIZE samples: memcpy buffer → snapshot, send over WebSocket

### Timer API (ESP32 core v3.x — IMPORTANT)
```cpp
// v3.x syntax — NOT the old v2.x timerBegin(id, prescaler, countUp)
hw_timer_t* tmr = timerBegin(1000000);          // 1MHz tick
timerAttachInterrupt(tmr, &onTimerISR);         // no edge param
timerAlarm(tmr, TIMER_ALARM_US, true, 0);       // replaces timerAlarmWrite + timerAlarmEnable
```

### WiFi / WebSocket
- ESP32 connects to existing WiFi router (station mode)
- HTTP server port 80 — serves HTML dashboard
- WebSocket server port 81 — streams data
- Library: "WebSockets" by Markus Sattler

WebSocket message types (JSON):
```json
{"t":"s","emg":[v1,v2,v3],"imu1":[ax,ay,az]}   // live sample  ~50Hz display
{"t":"w","rows":[[...21 rows of 100 floats...]]} // full CNN window
```

### Serial Plotter mode (separate .ino)
- 100Hz output
- Format: `EMG1:val,EMG2:val,EMG3:val,ax1:val,...,gz1:val\n`
- No external libraries required

---

## FILES PRODUCED

| File | Purpose |
|------|---------|
| EMG_IMU_WiFi_CNN.ino     | Main firmware — WiFi WebSocket + CNN windowed output |
| EMG_IMU_SerialPlotter.ino | Debug firmware — Arduino Serial Plotter, no WiFi |
| KiCad_PCB_Reference.docx  | Full pinout, BOM, connector refs, Voltera rules |

---

## PCB / KICAD

- Target: Voltera V-One printed PCB
- EDA: KiCad 7/8
- Strategy: one main board, all breakouts plug into 2.54mm headers
- Power: USB Micro-B + LiPo (TP4056 charger + LP2985 3.3V LDO)

KiCad libraries needed:
```
espressif/kicad-libraries          → ESP32 symbol + footprint
sparkfun/SparkFun-KiCad-Libraries  → AD8232 breakout header footprint
niklasf/kicad-footprints           → GY-521 breakout header footprint
KiCad Standard (built-in)          → headers, passives, USB, power
```

Voltera design rules:
- Min trace: 8 mil (use 12 mil signal, 24 mil power)
- Min clearance: 8 mil
- Min via drill: 0.4mm
- 2 layers max: Top = signals, Bottom = GND pour

Net names used in schematic:
```
+3V3, GND, +VBAT
EMG1_OUT, EMG2_OUT, EMG3_OUT
I2C0_SDA, I2C0_SCL
I2C1_SDA, I2C1_SCL
```

---

## KNOWN ISSUES / DECISIONS MADE

1. SoftwareWire (Testato) is INCOMPATIBLE with ESP32 core v3.x
   → Solution: use both I2C addresses (0x68 + 0x69) on Bus 0 instead
   → IMU2 AD0 must be wired to 3V3, not GND

2. ESP32 timer API changed completely in core v3.x
   → Old: timerBegin(id, prescaler, true) + timerAlarmWrite + timerAlarmEnable
   → New: timerBegin(freq) + timerAlarm(tmr, val, reload, count)

3. 500Hz chosen over 1kHz
   → 3× MPU6050 I2C burst reads take ~900µs total
   → 500Hz leaves 1100µs margin per sample; 1kHz is too tight

4. AD8232 LPF is 40Hz (SparkFun default)
   → Limits EMG waveform fidelity but sufficient for gesture CNN classification
   → Normalisation is per-window so gain/offset variations are irrelevant

5. Knee biomechanics system (4× MPU6050, complementary filter) exists separately
   → Will be merged with EMG system in future
   → Kept separate until both are independently validated

---

## PYTHON PARSER (receiving CNN windows from WebSocket)

```python
import serial, json
import numpy as np

# WebSocket version
import websocket
window = {}
def on_message(ws, msg):
    data = json.loads(msg)
    if data["t"] == "w":
        x = np.array(data["rows"], dtype=np.float32)
        # x.shape == (21, 100) → feed to CNN

# Serial Plotter version (if using SerialPlotter.ino instead)
# Each line: "EMG1:val,EMG2:val,...,gz1:val\n"
```

---

## CONTEXT FOR COPILOT

When writing or modifying code for this project:
- Always use ESP32 core v3.x timer API (timerBegin, timerAttachInterrupt, timerAlarm)
- Never use SoftwareWire — use TwoWire (Wire / Wire1) with address differentiation
- readMPU() takes TwoWire& and uint8_t addr as arguments (not templated)
- ISR sets flag only — all I2C and Serial in loop()
- ADC pins 34/35/32 are INPUT ONLY — never set as OUTPUT
- All sensors powered at 3.3V
- WINDOW_SIZE=100, TOTAL_ROWS=21, SAMPLE_RATE_HZ=500
- Per-window normalisation to [0,1] before CNN — do not use global normalisation
