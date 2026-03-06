# Knee Brace Biosignal System

Wearable EMG + IMU data-acquisition firmware for the ELEGOO ESP32, streaming a
**21-row × 100-column** sensor matrix over WiFi WebSocket for gesture-classification
with a CNN.

---

## Hardware

| Component | Quantity | Notes |
|-----------|----------|-------|
| ELEGOO ESP32 DevKit (38-pin WROOM-32) | 1 | Arduino core v3.x |
| SparkFun AD8232 EMG breakout (DEV-12650) | 3 | Fixed 100× gain, 3.3 V |
| GY-521 MPU6050 IMU breakout | 3 | I²C, 3.3 V |
| 4.7 kΩ pull-up resistors | 6 | SDA + SCL for each I²C bus |

---

## Wiring

### EMG — ADC (INPUT ONLY pins)

| AD8232 | ESP32 GPIO |
|--------|-----------|
| AD8232 #1 OUTPUT | GPIO 34 (ADC1_CH6) |
| AD8232 #2 OUTPUT | GPIO 35 (ADC1_CH7) |
| AD8232 #3 OUTPUT | GPIO 32 (ADC1_CH4) |

All AD8232 boards powered at **3.3 V** (not 5 V).

### IMU — I²C

| Bus | SDA | SCL | IMU | I²C Address | AD0 pin |
|-----|-----|-----|-----|-------------|---------|
| Wire  (Bus 0) | GPIO 21 | GPIO 22 | IMU1 | 0x68 | GND |
| Wire  (Bus 0) | GPIO 21 | GPIO 22 | IMU2 | 0x69 | 3V3 |
| Wire1 (Bus 1) | GPIO 25 | GPIO 26 | IMU3 | 0x68 | GND |

4.7 kΩ pull-ups required on each SDA and SCL line to 3.3 V (6 resistors total).

---

## Sketches

### `EMG_IMU_SerialPlotter/EMG_IMU_SerialPlotter.ino`

Debug firmware — samples all sensors at **100 Hz** and prints 21 labelled values
per line to Serial for use with the Arduino Serial Plotter.

### `EMG_IMU_WiFi_CNN/EMG_IMU_WiFi_CNN.ino`

Main firmware — samples at **500 Hz**, connects to WiFi (station mode), serves an
HTTP dashboard on port 80, and streams live samples + normalised 21×100 CNN windows
over WebSocket on port 81.

---

## How to Flash (Arduino IDE)

1. Install **Arduino IDE 2.x** and the **ESP32 board package** (Espressif, v3.x).
2. In *Tools → Board*, select **ESP32 Dev Module**.
3. For the WiFi sketch, install the following libraries via *Sketch → Include Library → Manage Libraries*:
   - **WebSockets** by Markus Sattler
   - **ArduinoJson** by Benoit Blanchon (v6 or v7)
4. Open the desired `.ino` file, select the correct COM port, and click **Upload**.

---

## Serial Plotter Usage

1. Flash `EMG_IMU_SerialPlotter.ino`.
2. Open *Tools → Serial Plotter* at **115200 baud**.
3. You will see 21 traces labelled `EMG1`, `EMG2`, `EMG3`, `ax1` … `gz3`.

Output format (one line per sample, 100 lines/s):
```
EMG1:val,EMG2:val,EMG3:val,ax1:val,ay1:val,az1:val,gx1:val,gy1:val,gz1:val,ax2:val,...,gz3:val
```

---

## WiFi Dashboard Usage

1. Edit `EMG_IMU_WiFi_CNN.ino` — replace `YOUR_SSID` and `YOUR_PASSWORD` with your
   router credentials.
2. Flash the sketch and open the Serial Monitor at **115200 baud**.
3. The ESP32 will print its IP address (e.g. `192.168.1.42`).
4. Open `http://192.168.1.42` in a browser to see the live dashboard.
5. The dashboard shows:
   - Live EMG bar graphs (3 channels, 0–4095 ADC range).
   - Live IMU1 accelerometer values (ax, ay, az).
   - A status line confirming receipt of each 21×100 CNN window.

WebSocket messages (port 81):
```json
{"t":"s","emg":[v1,v2,v3],"imu1":[ax,ay,az]}
// live sample  ~50 Hz

{"t":"w","rows":[[f0,f1,...,f99],[f0,f1,...,f99],...]}
// CNN window  every 200 ms — "rows" is an array of 21 sub-arrays,
// each containing 100 normalised floats [0.0, 1.0]
```

---

## CNN Input Matrix — Row Layout

| Row | Signal |
|-----|--------|
| 0  | EMG1 |
| 1  | IMU1_ax |
| 2  | IMU1_ay |
| 3  | IMU1_az |
| 4  | IMU1_gx |
| 5  | IMU1_gy |
| 6  | IMU1_gz |
| 7  | EMG2 |
| 8  | IMU2_ax |
| 9  | IMU2_ay |
| 10 | IMU2_az |
| 11 | IMU2_gx |
| 12 | IMU2_gy |
| 13 | IMU2_gz |
| 14 | EMG3 |
| 15 | IMU3_ax |
| 16 | IMU3_ay |
| 17 | IMU3_az |
| 18 | IMU3_gx |
| 19 | IMU3_gy |
| 20 | IMU3_gz |

Normalisation: **per-row, per-window min-max** to [0.0, 1.0].  
Window size: 100 samples @ 500 Hz = 200 ms.

---

## Full Hardware and Design Details

See [`copilot_context.md`](copilot_context.md) — the single source of truth for
pinout, I²C bus configuration, timer API notes, known issues, and design decisions.
