/*
 * AutoSat – Automatische Satelliten-Ausrichtung
 * Band 7 "Logbuch ohne Pose" – bigbrainlabs
 *
 * Hardware:
 *   - ESP32-C3 SuperMini
 *   - GY-91 (MPU-6500) an I2C (SDA=GPIO6, SCL=GPIO7)
 *   - A4988 Motortreiber (STEP=GPIO2, DIR=GPIO4, EN=GND fest)
 *   - MG996R Servo Elevation (GPIO5)
 *   - NEMA17 Azimut, 1/16 Microstepping, 20Z Motor / 40Z Plattform
 *
 * GPS: hardcoded (Leipzig) – später via MQTT/Webinterface
 * Satellit: Astra 19.2° Ost
 * Webportal: WLAN "AutoSat" / PW "autosat1" → http://192.168.4.1
 */

#include <Wire.h>
#include <ESP32Servo.h>
#include <math.h>

// ═══════════════════════════════════════════════
// PINS
// ═══════════════════════════════════════════════
#define STEP_PIN     2
#define DIR_PIN      4
// ENABLE dauerhaft auf GND gebrückt am TMC2209
#define SERVO_PIN    5
#define I2C_SDA      6
#define I2C_SCL      7

// ═══════════════════════════════════════════════
// MOTORKONFIGURATION
// ═══════════════════════════════════════════════
#define STEPS_PER_REV     200
#define MICROSTEPPING     16
#define GEAR_RATIO        2.0
#define STEPS_PER_DEGREE  (STEPS_PER_REV * MICROSTEPPING * GEAR_RATIO / 360.0)

#define STEP_DELAY_TRACK  300    // µs für Tracking (kleine Korrekturen)
#define STEP_MAN_MIN      800    // µs – unterhalb NEMA17 Resonanz (~39 Vollschr/s)
#define STEP_MAN_MAX     3000    // µs Anlauf/Auslauf
#define STEP_DELAY_US     STEP_DELAY_TRACK
#define MAX_ROTATIONS     1.5
#define YAW_SIGN          (+1)

// Magnetische Deklination Leipzig ≈ 3.3° Ost → mag_heading - 3.3 = true heading
// Webportal GPS/Sat-Tab → Azimut-Offset für Feinkalibrierung
#define MAG_DECLINATION   3.3f

// Komplementärfilter Yaw: dynamisch – bei Drehung mehr Gyro, bei Stillstand mehr Kompass
#define ALPHA_YAW_MOVING  0.97f   // aktive Drehung: Gyro dominiert (kurzzeitig präzise)
#define ALPHA_YAW_STILL   0.85f   // Stillstand: Kompass zieht Gyro-Drift heraus
#define GZ_MOVING_THR     3.0f    // °/s – ab hier gilt "Drehung erkannt"

// webportal.h hier einbinden: alle #defines sichtbar, Typen (WebServer etc.) verfügbar
#include "webportal.h"

// ═══════════════════════════════════════════════
// GPS + SATELLIT (Defaults, überschreibbar via Webportal)
// ═══════════════════════════════════════════════
double gps_lat = 51.3397;
double gps_lon = 12.3731;
double sat_lon = 19.2;
float  az_offset = 0.0;

// ═══════════════════════════════════════════════
// MPU REGISTER
// ═══════════════════════════════════════════════
#define MPU_ADDR     0x68
#define AK8963_ADDR  0x0C
#define PWR_MGMT_1   0x6B
#define ACCEL_XOUT_H 0x3B
#define INT_PIN_CFG  0x37
#define USER_CTRL    0x6A

// ═══════════════════════════════════════════════
// QMC5883L REGISTER (I2C 0x0D)
// ═══════════════════════════════════════════════
#define QMC_ADDR     0x0D
#define QMC_DATA     0x00
#define QMC_STATUS   0x06
#define QMC_CTRL1    0x09
#define QMC_CTRL2    0x0A
#define QMC_RESET    0x0B

// ═══════════════════════════════════════════════
// HMC5883L REGISTER (GY-271, I2C 0x1E)
// ═══════════════════════════════════════════════
#define HMC_ADDR     0x1E
#define HMC_CRA      0x00   // Config A: samples avg, data rate
#define HMC_CRB      0x01   // Config B: gain
#define HMC_MODE     0x02   // Mode register
#define HMC_DATA     0x03   // X_MSB, X_LSB, Z_MSB, Z_LSB, Y_MSB, Y_LSB

// ═══════════════════════════════════════════════
// DATENSTRUKTUREN
// ═══════════════════════════════════════════════
struct SensorData {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
};

// ═══════════════════════════════════════════════
// GLOBALE VARIABLEN
// ═══════════════════════════════════════════════
Servo        elevationServo;
WebServer    server(80);
DNSServer    dnsServer;
Preferences  prefs;

long  currentSteps  = 0;
float pitch         = 0.0;
float roll          = 0.0;
float heading_yaw   = 0.0;
float gz_offset     = 0.0;

unsigned long lastTime      = 0;
unsigned long lastMotorTime = 0;
const float   ALPHA         = 0.98;

float targetAzimuth   = 0.0;
float targetElevation = 0.0;
bool  ak8963_ok       = false;
bool  qmc_ok          = false;
bool  hmc_ok          = false;

// Servo-Kalibrierung (persistent, Defaults bis erstes Speichern)
int          servoMin         = 20;   // Sicherer Default – nie auf 0° fahren
int          servoMax         = 75;   // Sicherer Default – kein Vollausschlag
int          servoManualAngle = 20;
int          servoCalAngle    = 0;    // Kalibrierter Winkel (0 = noch nicht kalibriert)
// Elevationskorrektur-Achse: 0=+Pitch  1=+Roll  2=-Pitch  3=-Roll
int          elevAxisMode    = 0;
int          servoTarget      = 20;   // Zielwinkel für langsame Rampe
int          servoActual      = 20;   // Tatsächlich geschriebener Winkel
bool         servoEnabled     = false; // Servo erst nach explizitem Aktivieren im Portal bewegen
bool         manualControl    = false;
unsigned long lastServoCmd    = 0;
unsigned long lastServoStep   = 0;    // Zeitstempel letzter Rampen-Schritt

// Motorposition persistent
long         lastSavedSteps   = 0;
bool         positionDirty    = false;

// WLAN-Credentials (persistent)
char         wlan_ssid[64]    = "";
char         wlan_pass[64]    = "";

// Debug-Daten (letzte Sensor-Messung, für Webportal)
float dbg_ax=0, dbg_ay=0, dbg_az=0;
float dbg_gx=0, dbg_gy=0, dbg_gz=0;
float dbg_mx=0, dbg_my=0, dbg_mz=0;
float dbg_gz_net=0;
bool  dbg_dir=false;
float loopHz=0;

// Tracking
bool  trackingEnabled = false;
float azMountOffset   = 0.0f;   // Montageoffset: Kompassnord→Schüssel bei steps=0

// Azimut-Softlimits (Steps), kalibrierbar über Webportal
// Default = voller mechanischer Bereich (MAX_ROTATIONS)
long azStepMin = -(long)(1.5f * 200 * 16 * 2.0f);   // -9600
long azStepMax =  (long)(1.5f * 200 * 16 * 2.0f);   // +9600

// ═══════════════════════════════════════════════
// MPU INIT
// ═══════════════════════════════════════════════
void mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 1, true);
  byte id = Wire.read();
  Serial.print("MPU WHO_AM_I: 0x"); Serial.println(id, HEX);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(USER_CTRL);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(10);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(INT_PIN_CFG);
  Wire.write(0x02);
  Wire.endTransmission();
  delay(10);

  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(0x0A);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(AK8963_ADDR);
  Wire.write(0x0A);
  Wire.write(0x16);
  ak8963_ok = (Wire.endTransmission() == 0);
  delay(50);

  Serial.print("AK8963: ");
  Serial.println(ak8963_ok ? "OK" : "nicht gefunden");
}

// ═══════════════════════════════════════════════
// QMC5883L INIT
// ═══════════════════════════════════════════════
void qmcInit() {
  // Chip-ID prüfen
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(0x0D);
  Wire.endTransmission(false);
  Wire.requestFrom(QMC_ADDR, 1, true);
  byte id = Wire.read();
  Serial.printf("QMC5883L Chip-ID: 0x%02X (erwartet 0xFF)\n", id);

  // SET/RESET period
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(QMC_RESET);
  Wire.write(0x01);
  Wire.endTransmission();
  delay(10);

  // CTRL1: OSR=512, Range=8G, ODR=200Hz, Modus=Continuous
  // 0b00011101 = 0x1D
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(QMC_CTRL1);
  Wire.write(0x1D);
  qmc_ok = (Wire.endTransmission() == 0);
  delay(10);

  Serial.print("QMC5883L: ");
  Serial.println(qmc_ok ? "OK" : "nicht gefunden");
}

// ═══════════════════════════════════════════════
// HMC5883L INIT
// ═══════════════════════════════════════════════
void hmc5883lInit() {
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(HMC_CRA);
  Wire.write(0x70);  // 8 samples avg, 15 Hz, normal measurement
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(HMC_CRB);
  Wire.write(0x20);  // Gain 1090 LSB/Gauss (±1.3 Ga)
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(HMC_MODE);
  Wire.write(0x00);  // Continuous measurement mode
  hmc_ok = (Wire.endTransmission() == 0);
  delay(10);
  Serial.print("HMC5883L: ");
  Serial.println(hmc_ok ? "OK" : "nicht gefunden");
}

// ═══════════════════════════════════════════════
// MPU LESEN
// ═══════════════════════════════════════════════
SensorData readMPU() {
  SensorData data;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  data.ax = ax / 16384.0;
  data.ay = ay / 16384.0;
  data.az = az / 16384.0;
  data.gx = gx / 131.0;
  data.gy = gy / 131.0;
  data.gz = gz / 131.0;

  data.mx = data.my = data.mz = 0.0f;

  if (qmc_ok) {
    // QMC5883L: Status prüfen, dann 6 Bytes lesen (little-endian)
    Wire.beginTransmission(QMC_ADDR);
    Wire.write(QMC_STATUS);
    Wire.endTransmission(false);
    Wire.requestFrom(QMC_ADDR, 1, true);
    byte st = Wire.read();
    if (st & 0x01) {
      Wire.beginTransmission(QMC_ADDR);
      Wire.write(QMC_DATA);
      Wire.endTransmission(false);
      if (Wire.requestFrom(QMC_ADDR, 6, true) == 6) {
        int16_t mx = (int16_t)(Wire.read() | (Wire.read() << 8));
        int16_t my = (int16_t)(Wire.read() | (Wire.read() << 8));
        int16_t mz = (int16_t)(Wire.read() | (Wire.read() << 8));
        data.mx = (float)mx;
        data.my = (float)my;
        data.mz = (float)mz;
      }
    }
  } else if (hmc_ok) {
    // HMC5883L: Reihenfolge X, Z, Y (nicht X, Y, Z!)
    Wire.beginTransmission(HMC_ADDR);
    Wire.write(HMC_DATA);
    Wire.endTransmission(false);
    if (Wire.requestFrom((uint8_t)HMC_ADDR, (uint8_t)6, (uint8_t)true) == 6) {
      int16_t mx = (int16_t)((Wire.read() << 8) | Wire.read());
      int16_t mz = (int16_t)((Wire.read() << 8) | Wire.read());
      int16_t my = (int16_t)((Wire.read() << 8) | Wire.read());
      data.mx = mx * 0.092f;  // 1090 LSB/Gauss → µT
      data.my = my * 0.092f;
      data.mz = mz * 0.092f;
    }
  } else if (ak8963_ok) {
    Wire.beginTransmission(AK8963_ADDR);
    Wire.write(0x02);
    Wire.endTransmission(false);
    if (Wire.requestFrom((uint8_t)AK8963_ADDR, (uint8_t)8, (uint8_t)true) == 8) {
      byte st1 = Wire.read();
      int16_t mx = (int16_t)(Wire.read() | (Wire.read() << 8));
      int16_t my = (int16_t)(Wire.read() | (Wire.read() << 8));
      int16_t mz = (int16_t)(Wire.read() | (Wire.read() << 8));
      Wire.read();
      if (st1 & 0x01) {
        data.mx = mx * 0.15f;
        data.my = my * 0.15f;
        data.mz = mz * 0.15f;
      }
    }
  }

  return data;
}

// ═══════════════════════════════════════════════
// KOMPLEMENTÄRFILTER (Pitch / Roll / Yaw)
// ═══════════════════════════════════════════════
void updateIMU(SensorData &d) {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0f;
  lastTime = now;

  // Pitch / Roll: Komplementärfilter aus Gyro + Beschleunigungssensor
  float accelPitch = atan2f(d.ay, d.az) * 180.0f / M_PI;
  float accelRoll  = atan2f(-d.ax, d.az) * 180.0f / M_PI;
  pitch = ALPHA * (pitch + d.gx * dt) + (1.0f - ALPHA) * accelPitch;
  roll  = ALPHA * (roll  + d.gy * dt) + (1.0f - ALPHA) * accelRoll;

  // Gyro-Heading (kurzfristig)
  float gz_net = d.gz - gz_offset;
  float gyro_hdg = heading_yaw;
  if (fabsf(gz_net) > 1.0f) {
    gyro_hdg += YAW_SIGN * gz_net * dt;
    if (gyro_hdg >= 360.0f) gyro_hdg -= 360.0f;
    if (gyro_hdg <    0.0f) gyro_hdg += 360.0f;
  }


  if ((ak8963_ok || qmc_ok) && (d.mx != 0.0f || d.my != 0.0f || d.mz != 0.0f)) {
    // Tilt-kompensiertes Magnetometer-Heading
    float cosR = cosf(roll  * M_PI / 180.0f);
    float sinR = sinf(roll  * M_PI / 180.0f);
    float cosP = cosf(pitch * M_PI / 180.0f);
    float sinP = sinf(pitch * M_PI / 180.0f);

    float Xh = d.mx * cosP + d.mz * sinP;
    float Yh = d.mx * sinR * sinP + d.my * cosR - d.mz * sinR * cosP;

    float mag_hdg = atan2f(-Yh, Xh) * 180.0f / M_PI;
    if (mag_hdg < 0.0f) mag_hdg += 360.0f;
    mag_hdg -= MAG_DECLINATION;
    if (mag_hdg < 0.0f) mag_hdg += 360.0f;

    // Komplementärfilter: dynamisches Alpha je nach Drehrate
    float alpha_yaw = (fabsf(gz_net) > GZ_MOVING_THR) ? ALPHA_YAW_MOVING : ALPHA_YAW_STILL;
    float diff = mag_hdg - gyro_hdg;
    while (diff >  180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;

    heading_yaw = gyro_hdg + (1.0f - alpha_yaw) * diff;
    if (heading_yaw >= 360.0f) heading_yaw -= 360.0f;
    if (heading_yaw <    0.0f) heading_yaw += 360.0f;
  } else {
    heading_yaw = gyro_hdg;
  }
}

// ═══════════════════════════════════════════════
// SATELLITENBERECHNUNG
// ═══════════════════════════════════════════════
void calcSatPosition(double lat, double lon, double satLon,
                     float &azimuth, float &elevation) {
  const double RE = 6371.0;
  const double RS = 42164.0;

  double latRad  = lat * M_PI / 180.0;
  double lonDiff = (satLon - lon) * M_PI / 180.0;

  double cosEl = cos(latRad) * cos(lonDiff);
  elevation = atan((cosEl - RE / RS) / sqrt(1.0 - cosEl * cosEl)) * 180.0 / M_PI;

  double az = atan2(sin(lonDiff), -sin(latRad) * cos(lonDiff));
  azimuth = az * 180.0 / M_PI;
  if (azimuth < 0) azimuth += 360.0;
}

// ═══════════════════════════════════════════════
// TMC2209 UART KONFIGURATION – Software-Bit-Bang (GPIO8 / PDN_UART)
// ═══════════════════════════════════════════════
#define TMC_UART_TX   8
#define TMC_BIT_US  104   // 9600 Baud → 104µs pro Bit

static uint8_t tmcCRC(uint8_t *buf, uint8_t n) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < n; i++) {
    uint8_t b = buf[i];
    for (uint8_t j = 0; j < 8; j++) {
      if ((crc >> 7) ^ (b & 1)) crc = (crc << 1) ^ 0x07;
      else crc <<= 1;
      b >>= 1;
    }
  }
  return crc;
}

static void tmcSendByte(uint8_t byte) {
  noInterrupts();                           // Bit-Timing sichern
  digitalWrite(TMC_UART_TX, LOW);          // Start-Bit
  delayMicroseconds(TMC_BIT_US);
  for (int i = 0; i < 8; i++) {
    digitalWrite(TMC_UART_TX, (byte >> i) & 1);
    delayMicroseconds(TMC_BIT_US);
  }
  digitalWrite(TMC_UART_TX, HIGH);         // Stop-Bit
  interrupts();
  delayMicroseconds(TMC_BIT_US);           // Stop-Bit-Dauer mit Interrupts
}

static void tmcWrite(uint8_t addr, uint8_t reg, uint32_t val) {
  uint8_t buf[8] = {
    0x05, addr, (uint8_t)(reg | 0x80),
    (uint8_t)(val >> 24), (uint8_t)(val >> 16),
    (uint8_t)(val >>  8), (uint8_t)(val), 0
  };
  buf[7] = tmcCRC(buf, 7);
  for (int i = 0; i < 8; i++) tmcSendByte(buf[i]);
  delayMicroseconds(200);
}

static void tmcWriteAll(uint8_t reg, uint32_t val) {
  // Adresse 3 = MS1=HIGH/MS2=HIGH = 1/16 Mikroschritt (bestätigt)
  // Nur eine Adresse → ~8ms Interrupt-Blackout statt 32ms
  tmcWrite(3, reg, val);
  yield();  // WiFi-Stack nach dem Send verarbeiten
}

static bool tmcCurDir = true;

void tmcSetDir(bool dir) {
  if (dir == tmcCurDir) return;
  tmcCurDir = dir;
  // GCONF: StealthChop, shaft-Bit invertiert (Hardware-Polung)
  tmcWriteAll(0x00, dir ? 0x00000008 : 0x00000000);
}

void tmcInit() {
  pinMode(TMC_UART_TX, OUTPUT);
  digitalWrite(TMC_UART_TX, HIGH);  // UART Idle = HIGH
  delay(10);
  tmcCurDir = false;
  // GCONF: shaft=0, StealthChop (entspricht dir=false nach Invertierung)
  tmcWriteAll(0x00, 0x00000000);
  // IHOLD_IRUN: IRUN=8 (25%), IHOLD=2 (6%), IHOLDDELAY=10
  // Für Satellitentracker mehr als ausreichend – minimale Wärmeentwicklung
  tmcWriteAll(0x10, 0x000A0802);
  delay(5);
  Serial.println("TMC2209: SoftUART init OK (StealthChop, IRUN=50%)");
}

// ═══════════════════════════════════════════════
// MOTOR
// ═══════════════════════════════════════════════
void stepMotor(bool dir, unsigned int stepDelay = STEP_DELAY_TRACK) {
  tmcSetDir(dir);               // Richtung per UART (shaft-Bit)
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(stepDelay);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(stepDelay);
  currentSteps  += dir ? 1 : -1;
  lastMotorTime  = millis();
  positionDirty  = true;
  dbg_dir        = dir;
}

// Ramped stepping: langsam anlaufen, schnell fahren, langsam stoppen.
// Vermeidet Riemensprung (langsam) UND NEMA17-Resonanz (schnelle Cruise-Speed).
void stepMotorManual(bool dir, int count) {
  int ramp = min(count / 3, 10);
  for (int i = 0; i < count; i++) {
    unsigned int d;
    if (i < ramp)
      d = (unsigned int)(STEP_MAN_MAX - (long)(STEP_MAN_MAX - STEP_MAN_MIN) * i / max(ramp, 1));
    else if (i >= count - ramp)
      d = (unsigned int)(STEP_MAN_MIN + (long)(STEP_MAN_MAX - STEP_MAN_MIN) * (i - (count - ramp)) / max(ramp, 1));
    else
      d = STEP_MAN_MIN;
    stepMotor(dir, d);
  }
}

// ═══════════════════════════════════════════════
// AZIMUT NACHFÜHREN
// ═══════════════════════════════════════════════
void recalcSat() {
  calcSatPosition(gps_lat, gps_lon, sat_lon, targetAzimuth, targetElevation);
  Serial.printf("Neuberechnung: Az=%.1f° El=%.1f°\n", targetAzimuth, targetElevation);
}

void updateAzimuth(float heading) {
  // Absolutes Tracking: Schüsselrichtung aus Kompass + Motorposition + Montageoffset
  float dishAz = heading + (float)currentSteps / STEPS_PER_DEGREE + azMountOffset;
  while (dishAz >= 360.0f) dishAz -= 360.0f;
  while (dishAz <    0.0f) dishAz += 360.0f;

  float error = targetAzimuth - dishAz;
  while (error >  180.0f) error -= 360.0f;
  while (error < -180.0f) error += 360.0f;

  if (fabsf(error) < 0.5f) return;
  if (currentSteps >= azStepMax && error > 0) return;
  if (currentSteps <= azStepMin && error < 0) return;

  int  steps = constrain((int)(fabsf(error) * STEPS_PER_DEGREE), 1, 10);
  bool dir   = error > 0;
  for (int i = 0; i < steps; i++) stepMotor(dir);
}

// ═══════════════════════════════════════════════
// ELEVATION NACHFÜHREN
// ═══════════════════════════════════════════════
#define SERVO_STEP_MS_MANUAL   60  // ~16°/s – manuelles Positionieren
#define SERVO_STEP_MS_TRACKING 12  // ~83°/s – nahtloses Live-Tracking
static uint8_t servoStepMs = SERVO_STEP_MS_MANUAL;

// Zielwinkel setzen. tracking=true → schnelle Rampe für Live-Tracking
void servoMoveTo(int angle, bool tracking = false) {
  if (!servoEnabled) return;
  servoStepMs = tracking ? SERVO_STEP_MS_TRACKING : SERVO_STEP_MS_MANUAL;
  servoTarget = constrain(angle, servoMin, servoMax);
}

// Rampe: 1° pro servoStepMs – in loop() aufrufen
void servoRampTick() {
  if (!servoEnabled) return;
  if (millis() - lastServoStep < servoStepMs) return;
  lastServoStep = millis();
  if (servoActual == servoTarget) return;
  servoActual += (servoActual < servoTarget) ? 1 : -1;
  servoActual = constrain(servoActual, servoMin, servoMax);
  elevationServo.write(servoActual);
  servoManualAngle = servoActual;
}

void updateElevation() {
  if (manualControl) return;
  if (servoCalAngle == 0) return;
  float corr;
  switch (elevAxisMode) {
    case 1:  corr =  roll;  break;
    case 2:  corr = -pitch; break;
    case 3:  corr = -roll;  break;
    default: corr =  pitch; break;
  }
  servoMoveTo(servoCalAngle + (int)roundf(corr), true);
}

// ═══════════════════════════════════════════════
// GYRO-OFFSET KALIBRIERUNG (aufrufbar aus Webportal)
// ═══════════════════════════════════════════════
void calibrateGyroOffset() {
  Serial.println("Kalibriere Gyro – bitte stillhalten (2.5s)...");
  float sum = 0;
  for (int i = 0; i < 500; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x47);  // GYRO_ZOUT_H
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true);
    int16_t gz = (int16_t)((Wire.read() << 8) | Wire.read());
    sum += gz / 131.0f;
    delay(5);
  }
  gz_offset = sum / 500.0f;
  Serial.printf("gz-Offset neu: %.4f°/s\n", gz_offset);
}

// ═══════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== AutoSat v0.4 ===");

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);

  // Servo bleibt beim Start DEAKTIVIERT – kein automatisches Fahren beim Boot.
  // Erst nach explizitem "Servo aktivieren" im Portal wird er angehängt.
  servoActual      = servoMin;
  servoTarget      = servoMin;
  servoManualAngle = servoMin;

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);  // 400 kHz → ~3x schnellere I2C-Reads
  mpuInit();
  qmcInit();
  hmc5883lInit();

  // Alle persistenten Werte laden
  prefs.begin("autosat", true);
  servoMin      = constrain(prefs.getInt("servoMin",  20), 0,  60);  // max 60° als Untergrenze
  servoMax      = constrain(prefs.getInt("servoMax",  75), 20, 90);  // max 90° als Obergrenze
  servoCalAngle = prefs.getInt("srvCalAng",  0);
  elevAxisMode  = prefs.getInt("elvAxis",   0);
  if (servoCalAngle != 0)
    servoCalAngle = constrain(servoCalAngle, servoMin, servoMax);
  servoManualAngle = servoMin;
  servoActual      = servoMin;
  servoTarget      = servoMin;
  currentSteps  = prefs.getLong  ("steps",     0);
  gps_lat       = prefs.getDouble("gpsLat",    51.3397);
  gps_lon       = prefs.getDouble("gpsLon",    12.3731);
  sat_lon       = prefs.getDouble("satLon",    19.2);
  az_offset     = prefs.getFloat ("azOffset",  0.0f);
  azMountOffset = prefs.getFloat ("azMntOff",  0.0f);
  prefs.getString("wifiSSID", "").toCharArray(wlan_ssid, sizeof(wlan_ssid));
  prefs.getString("wifiPass", "").toCharArray(wlan_pass, sizeof(wlan_pass));
  azStepMin = prefs.getLong("azMin", -(long)(1.5f * 200 * 16 * 2.0f));
  azStepMax = prefs.getLong("azMax",  (long)(1.5f * 200 * 16 * 2.0f));
  prefs.end();
  lastSavedSteps = currentSteps;
  Serial.printf("Servo: MIN=%d MAX=%d  Steps:%ld\n", servoMin, servoMax, currentSteps);
  Serial.printf("GPS: %.4fN %.4fE  Sat:%.1fO  Offset:%.1f\n",
    gps_lat, gps_lon, sat_lon, az_offset);

  setupWebPortal("AutoSat", "autosat1");
  connectWiFiSTA();

  // Nach WiFi-Init Pins neu setzen – WiFi.softAP() kann GPIO-Config überschreiben
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN,  LOW);

  tmcInit();  // TMC2209 per UART konfigurieren (GPIO8 → PDN_UART)

  recalcSat();
  Serial.printf("Ziel-Azimut: %.1f°  Elevation: %.1f°\n", targetAzimuth, targetElevation);

  // WiFi-Stack stabilisieren bevor loop() startet
  { unsigned long t = millis();
    while(millis()-t < 2000){ dnsServer.processNextRequest(); server.handleClient(); delay(10); } }

  lastTime = millis();
  updateElevation();
  Serial.println("Bereit.");
}

// ═══════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════
void loop() {
  servoRampTick();   // Servo langsam zum Ziel bewegen (20ms/°)
  processWebPortal();

  // Manual-Timeout: 30s ohne Web-Befehl → Tracking übernimmt wieder
  if (manualControl && (millis() - lastServoCmd > 30000)) {
    manualControl = false;
    Serial.println("Manuelle Servo-Kontrolle beendet – Tracking aktiv.");
  }

  // Motorposition speichern: 5s nach letzter Bewegung, nur wenn geändert
  if (positionDirty && (millis() - lastMotorTime > 5000)) {
    prefs.begin("autosat", false);
    prefs.putLong("steps", currentSteps);
    prefs.end();
    lastSavedSteps = currentSteps;
    positionDirty  = false;
  }

  SensorData data = readMPU();
  updateIMU(data);
  if (trackingEnabled) {
    updateAzimuth(heading_yaw);
    updateElevation();
  }

  // Debug-Snapshot für Webportal
  dbg_ax = data.ax; dbg_ay = data.ay; dbg_az = data.az;
  dbg_gx = data.gx; dbg_gy = data.gy; dbg_gz = data.gz;
  dbg_mx = data.mx; dbg_my = data.my; dbg_mz = data.mz;
  dbg_gz_net = data.gz - gz_offset;

  // Loop-Frequenz messen
  static unsigned long hzTimer = 0;
  static int hzCount = 0;
  hzCount++;
  if (millis() - hzTimer >= 1000) {
    loopHz  = hzCount;
    hzCount = 0;
    hzTimer = millis();
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    lastPrint = millis();
    float dish = heading_yaw + (float)currentSteps / STEPS_PER_DEGREE + azMountOffset;
    while (dish >= 360.0f) dish -= 360.0f;
    while (dish <    0.0f) dish += 360.0f;
    float err = targetAzimuth - dish;
    while (err >  180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    Serial.printf("Hdg:%.1f° gz:%.2f  Err:%.1f°  Steps:%ld  Srv:%d°  Mag:%s%s\n",
      heading_yaw, data.gz - gz_offset, err, currentSteps,
      servoManualAngle,
      qmc_ok ? "QMC" : (ak8963_ok ? "AK" : "OFF"),
      manualControl ? " [MANUELL]" : "");
  }
}
