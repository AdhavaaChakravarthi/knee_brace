// EMG_IMU_SerialPlotter.ino
// Debug firmware: reads 3 EMG channels + 3 MPU6050 IMUs and prints
// 21 labelled values per line at 100 Hz for the Arduino Serial Plotter.
//
// Hardware (from copilot_context.md):
//   EMG: GPIO34 (EMG1), GPIO35 (EMG2), GPIO32 (EMG3) — INPUT ONLY
//   IMU Bus 0 (Wire):  SDA=21 SCL=22 → IMU1 @ 0x68, IMU2 @ 0x69
//   IMU Bus 1 (Wire1): SDA=25 SCL=26 → IMU3 @ 0x68
//
// Uses ESP32 core v3.x timer API.

#include <Wire.h>

// ── Pin / address constants ──────────────────────────────────────────────────
static const int    EMG_PIN1   = 34;
static const int    EMG_PIN2   = 35;
static const int    EMG_PIN3   = 32;

static const uint8_t IMU1_ADDR = 0x68;
static const uint8_t IMU2_ADDR = 0x69;
static const uint8_t IMU3_ADDR = 0x68;

// MPU6050 register addresses
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_CONFIG     = 0x1A;
static const uint8_t REG_ACCEL_XOUT = 0x3B;

// ── Timer ────────────────────────────────────────────────────────────────────
static hw_timer_t*   tmr          = nullptr;
static volatile bool sampleReady  = false;

void IRAM_ATTR onTimerISR() {
    sampleReady = true;
}

// ── MPU6050 helper ───────────────────────────────────────────────────────────
// Writes one byte to a MPU6050 register.
static void mpuWrite(TwoWire& bus, uint8_t addr, uint8_t reg, uint8_t val) {
    bus.beginTransmission(addr);
    bus.write(reg);
    bus.write(val);
    bus.endTransmission();
}

// Burst-reads 14 bytes from REG_ACCEL_XOUT and extracts ax/ay/az and gx/gy/gz.
void readMPU(TwoWire& bus, uint8_t addr,
             int16_t& ax, int16_t& ay, int16_t& az,
             int16_t& gx, int16_t& gy, int16_t& gz) {
    bus.beginTransmission(addr);
    bus.write(REG_ACCEL_XOUT);
    bus.endTransmission(false);

    bus.requestFrom((uint8_t)addr, (uint8_t)14);

    uint8_t buf[14] = {0};
    for (int i = 0; i < 14 && bus.available(); i++) {
        buf[i] = bus.read();
    }

    // Big-endian pairs: ax[0:1], ay[2:3], az[4:5], temp[6:7], gx[8:9], gy[10:11], gz[12:13]
    ax = (int16_t)((buf[0]  << 8) | buf[1]);
    ay = (int16_t)((buf[2]  << 8) | buf[3]);
    az = (int16_t)((buf[4]  << 8) | buf[5]);
    // buf[6:7] = temperature — skipped
    gx = (int16_t)((buf[8]  << 8) | buf[9]);
    gy = (int16_t)((buf[10] << 8) | buf[11]);
    gz = (int16_t)((buf[12] << 8) | buf[13]);
}

// ── Initialise a single MPU6050 ──────────────────────────────────────────────
static void initMPU(TwoWire& bus, uint8_t addr) {
    mpuWrite(bus, addr, REG_PWR_MGMT_1, 0x00);  // wake up
    mpuWrite(bus, addr, REG_CONFIG,     0x03);  // DLPF ~44 Hz
}

// ── setup() ──────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    // ADC configuration — pins are INPUT ONLY, never set as OUTPUT
    analogSetAttenuation(ADC_11db);

    // Initialise I2C buses
    Wire.begin(21, 22);
    Wire1.begin(25, 26);

    // Wake and configure all three MPU6050s
    initMPU(Wire,  IMU1_ADDR);
    initMPU(Wire,  IMU2_ADDR);
    initMPU(Wire1, IMU3_ADDR);

    // Hardware timer: 1 MHz tick, alarm every 10 000 ticks = 100 Hz
    tmr = timerBegin(1000000);
    timerAttachInterrupt(tmr, &onTimerISR);
    timerAlarm(tmr, 10000, true, 0);
}

// ── loop() ───────────────────────────────────────────────────────────────────
void loop() {
    if (!sampleReady) return;
    sampleReady = false;

    // Read EMG channels (raw 12-bit ADC values, 0–4095)
    int emg1 = analogRead(EMG_PIN1);
    int emg2 = analogRead(EMG_PIN2);
    int emg3 = analogRead(EMG_PIN3);

    // Read IMU1 (Wire, 0x68)
    int16_t ax1, ay1, az1, gx1, gy1, gz1;
    readMPU(Wire, IMU1_ADDR, ax1, ay1, az1, gx1, gy1, gz1);

    // Read IMU2 (Wire, 0x69)
    int16_t ax2, ay2, az2, gx2, gy2, gz2;
    readMPU(Wire, IMU2_ADDR, ax2, ay2, az2, gx2, gy2, gz2);

    // Read IMU3 (Wire1, 0x68)
    int16_t ax3, ay3, az3, gx3, gy3, gz3;
    readMPU(Wire1, IMU3_ADDR, ax3, ay3, az3, gx3, gy3, gz3);

    // Print 21 labelled values, comma-separated, one line per sample
    Serial.print("EMG1:"); Serial.print(emg1);
    Serial.print(",EMG2:"); Serial.print(emg2);
    Serial.print(",EMG3:"); Serial.print(emg3);

    Serial.print(",ax1:"); Serial.print(ax1);
    Serial.print(",ay1:"); Serial.print(ay1);
    Serial.print(",az1:"); Serial.print(az1);
    Serial.print(",gx1:"); Serial.print(gx1);
    Serial.print(",gy1:"); Serial.print(gy1);
    Serial.print(",gz1:"); Serial.print(gz1);

    Serial.print(",ax2:"); Serial.print(ax2);
    Serial.print(",ay2:"); Serial.print(ay2);
    Serial.print(",az2:"); Serial.print(az2);
    Serial.print(",gx2:"); Serial.print(gx2);
    Serial.print(",gy2:"); Serial.print(gy2);
    Serial.print(",gz2:"); Serial.print(gz2);

    Serial.print(",ax3:"); Serial.print(ax3);
    Serial.print(",ay3:"); Serial.print(ay3);
    Serial.print(",az3:"); Serial.print(az3);
    Serial.print(",gx3:"); Serial.print(gx3);
    Serial.print(",gy3:"); Serial.print(gy3);
    Serial.print(",gz3:"); Serial.println(gz3);
}
