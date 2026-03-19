/*
 * Ultisense Development Board — Combined Firmware
 *
 * Sensors on I2C (SDA=8, SCL=9):
 *   SHT41   0x44  — Temperature & Humidity
 *   SGP40   0x59  — VOC Gas Index
 *   TCS3400 0x39  — RGBC + IR Color / Lux
 *   QMC6309 0x7C  — Magnetometer / Heading
 *   ST25DV  0x53  — NFC tag (one-time NDEF write in setup)
 *
 * Loop runs at 1 Hz (required by the SGP40 VOC algorithm).
 */

#include <Wire.h>
#include <SensirionGasIndexAlgorithm.h>   // SGP40 VOC index
#include <VOCGasIndexAlgorithm.h>

// ── I²C pins ─────────────────────────────────────────────────────────────────
#define SDA_PIN 8
#define SCL_PIN 9

// ── Device addresses ──────────────────────────────────────────────────────────
#define SHT41_ADDR   0x44
#define SGP40_ADDR   0x59
#define TCS3400_ADDR 0x39
#define QMC6309_ADDR 0x7C
#define ST25DV_ADDR  0x53

// ── SGP40 VOC algorithm state ─────────────────────────────────────────────────
GasIndexAlgorithmParams vocParams;

// =============================================================================
//  SHARED UTILITY
// =============================================================================

/*
 * CRC-8 used by SHT41 and SGP40 (Sensirion standard).
 * Polynomial 0x31, initial value 0xFF.
 */
uint8_t sensirion_crc(uint8_t *data, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
  }
  return crc;
}

// =============================================================================
//  SHT41 — Temperature & Humidity
// =============================================================================

#define SHT41_MEAS_HIGH 0xFD   // high-repeatability measurement command

/* Trigger measurement and read 6 bytes (T_MSB T_LSB CRC RH_MSB RH_LSB CRC). */
bool sht41_read(float &tempC, float &rh) {
  Wire.beginTransmission(SHT41_ADDR);
  Wire.write(SHT41_MEAS_HIGH);
  if (Wire.endTransmission() != 0) return false;
  delay(10);   // datasheet: ~9 ms for high-repeatability

  Wire.requestFrom((uint8_t)SHT41_ADDR, (uint8_t)6);
  if (Wire.available() != 6) return false;

  uint8_t tb[2] = { Wire.read(), Wire.read() };
  uint8_t crc_t  = Wire.read();
  uint8_t rb[2] = { Wire.read(), Wire.read() };
  uint8_t crc_r  = Wire.read();

  // CRC validation
  if (sensirion_crc(tb, 2) != crc_t) return false;
  if (sensirion_crc(rb, 2) != crc_r) return false;

  uint16_t t_ticks  = ((uint16_t)tb[0] << 8) | tb[1];
  uint16_t rh_ticks = ((uint16_t)rb[0] << 8) | rb[1];

  // Datasheet conversion formulas
  tempC = -45.0f + 175.0f * (t_ticks  / 65535.0f);
  rh    = constrain(-6.0f  + 125.0f * (rh_ticks / 65535.0f), 0.0f, 100.0f);
  return true;
}

// =============================================================================
//  SGP40 — VOC Gas Index
// =============================================================================

/* Send measure-raw command with fixed 25 °C / 50 %RH compensation bytes. */
bool sgp40_read(int32_t &vocIndex) {
  uint8_t rh[2] = {0x80, 0x00};   // 50 %RH ticks
  uint8_t t[2]  = {0x66, 0x66};   // 25 °C ticks

  Wire.beginTransmission(SGP40_ADDR);
  Wire.write(0x26); Wire.write(0x0F);   // measure-raw command
  Wire.write(rh, 2); Wire.write(sensirion_crc(rh, 2));
  Wire.write(t,  2); Wire.write(sensirion_crc(t,  2));
  Wire.endTransmission();
  delay(30);   // datasheet: max 30 ms conversion

  Wire.requestFrom(SGP40_ADDR, 3);
  if (Wire.available() != 3) return false;

  uint8_t raw[2] = { Wire.read(), Wire.read() };
  uint8_t crc    = Wire.read();
  if (sensirion_crc(raw, 2) != crc) return false;

  uint16_t sraw = ((uint16_t)raw[0] << 8) | raw[1];
  GasIndexAlgorithm_process(&vocParams, sraw, &vocIndex);
  return true;
}

// =============================================================================
//  TCS3400 — RGBC + IR Color Sensor
// =============================================================================

// Register addresses (command-bit 0x80 already OR'd in)
#define TCS_ENABLE 0x80
#define TCS_ATIME  0x81
#define TCS_CTRL   0x8F
#define TCS_CDATAL 0x94   // clear
#define TCS_RDATAL 0x96
#define TCS_GDATAL 0x98
#define TCS_BDATAL 0x9A
#define TCS_IR     0xC0

void tcs_wreg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(TCS3400_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

uint16_t tcs_rreg16(uint8_t reg) {
  Wire.beginTransmission(TCS3400_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(TCS3400_ADDR, (uint8_t)2);
  if (Wire.available() < 2) return 0;
  uint8_t lo = Wire.read(), hi = Wire.read();
  return (hi << 8) | lo;
}

void tcs3400_init() {
  tcs_wreg(TCS_ENABLE, 0x01);          // power on
  delay(10);
  tcs_wreg(TCS_ATIME,  0xF6);          // ~27.8 ms integration
  tcs_wreg(TCS_CTRL,   0x02);          // 16× gain
  tcs_wreg(TCS_ENABLE, 0x03);          // power on + RGBC enable
  delay(50);
}

bool tcs3400_read(uint8_t &r8, uint8_t &g8, uint8_t &b8, float &lux) {
  uint16_t clear  = tcs_rreg16(TCS_CDATAL);
  uint16_t red    = tcs_rreg16(TCS_RDATAL);
  uint16_t green  = tcs_rreg16(TCS_GDATAL);
  uint16_t blue   = tcs_rreg16(TCS_BDATAL);

  // Read IR by temporarily routing it to the clear channel
  tcs_wreg(TCS_IR, 0x80); delay(5);
  tcs_wreg(TCS_IR, 0x00);

  if (clear == 0) return false;

  // Normalize to 0-255
  r8 = constrain((float)red   / clear * 255, 0, 255);
  g8 = constrain((float)green / clear * 255, 0, 255);
  b8 = constrain((float)blue  / clear * 255, 0, 255);

  // Empirical lux approximation from datasheet app note
  lux = (0.136f * red) + (1.000f * green) + (-0.444f * blue);
  return true;
}

// =============================================================================
//  QMC6309 — Magnetometer
// =============================================================================

void qmc_wreg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(QMC6309_ADDR);
  Wire.write(reg); Wire.write(val);
  Wire.endTransmission();
}

void qmc_rregs(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(QMC6309_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);   // repeated start
  Wire.requestFrom(QMC6309_ADDR, len);
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
}

void qmc6309_init() {
  delay(10);
  qmc_wreg(0x0B, 0x80); delay(2);   // soft reset
  qmc_wreg(0x0B, 0x00);
  qmc_wreg(0x0B, 0x40);   // ±32 G range, 200 Hz ODR, Set/Reset ON
  qmc_wreg(0x0A, 0x61);   // OSR1=8, OSR2=8, normal mode
}

bool qmc6309_read(float &heading, float &xG, float &yG, float &zG) {
  uint8_t status;
  qmc_rregs(0x09, &status, 1);
  if (!(status & 0x01)) return false;   // data-not-ready

  uint8_t buf[6];
  qmc_rregs(0x01, buf, 6);

  int16_t x = (int16_t)((buf[1] << 8) | buf[0]);
  int16_t y = (int16_t)((buf[3] << 8) | buf[2]);
  int16_t z = (int16_t)((buf[5] << 8) | buf[4]);

  // ±32 G range → 1000 LSB/G sensitivity
  xG = x / 1000.0f;
  yG = y / 1000.0f;
  zG = z / 1000.0f;

  heading = atan2(yG, xG) * 180.0f / PI;
  if (heading < 0) heading += 360.0f;
  return true;
}

// =============================================================================
//  ST25DV04KC — NFC Tag (one-time NDEF URL write)
// =============================================================================

void st25dv_write_block(uint16_t block, uint8_t *data) {
  uint16_t addr = block * 4;   // each block is 4 bytes
  Wire.beginTransmission(ST25DV_ADDR);
  Wire.write(addr >> 8);
  Wire.write(addr & 0xFF);
  for (int i = 0; i < 4; i++) Wire.write(data[i]);
  Wire.endTransmission();
  delay(6);   // EEPROM write cycle time
}

void st25dv_write_ndef_url() {
  // Block 0: Capability Container (CC) — marks tag as NFC Forum Type 5
  uint8_t cc[4]     = {0xE1, 0x40, 0x40, 0x00};

  // Blocks 1-5: NDEF message containing URL "darsh.app"
  //   03 = NDEF TLV tag, 11 = length 17 bytes
  //   D1 01 0D = TNF_WELL_KNOWN, type length 1, payload length 13
  //   55 = URI record type 'U', 03 = prefix "https://"
  //   "darsh.app" in ASCII, FE = terminator TLV
  uint8_t block1[4] = {0x03, 0x11, 0xD1, 0x01};
  uint8_t block2[4] = {0x0D, 0x55, 0x03, 'd' };
  uint8_t block3[4] = {'a',  'r',  's',  'h' };
  uint8_t block4[4] = {'.',  'a',  'p',  'p' };
  uint8_t block5[4] = {0xFE, 0x00, 0x00, 0x00};

  st25dv_write_block(0, cc);
  st25dv_write_block(1, block1);
  st25dv_write_block(2, block2);
  st25dv_write_block(3, block3);
  st25dv_write_block(4, block4);
  st25dv_write_block(5, block5);
}

// =============================================================================
//  SETUP
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Single shared I²C bus
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  pinMode(42, OUTPUT); // external led debug

  // Initialize each sensor
  tcs3400_init();
  qmc6309_init();
  GasIndexAlgorithm_init(&vocParams, GasIndexAlgorithm_ALGORITHM_TYPE_VOC);

  // Write NFC NDEF URL once at startup
  Serial.println("Writing NFC tag...");
  st25dv_write_ndef_url();
  Serial.println("NFC tag ready — tap phone to open darsh.app");

  Serial.println("Sensor | Reading");
  Serial.println("-------|---------");
}

// =============================================================================
//  LOOP  (1 Hz — required by SGP40 VOC algorithm)
// =============================================================================

void loop() {
  digitalWrite(42, HIGH);

  // ── SHT41: Temperature & Humidity ──────────────────────────────────────────
  float tempC, rh;
  if (sht41_read(tempC, rh))
    Serial.printf("SHT41  | %.1f°C  %.1f%%RH\n", tempC, rh);
  else
    Serial.println("SHT41  | ERR");

  // ── SGP40: VOC Index ────────────────────────────────────────────────────────
  int32_t vocIdx;
  if (sgp40_read(vocIdx))
    Serial.printf("SGP40  | VOC Index: %d\n", vocIdx);
  else
    Serial.println("SGP40  | ERR");

  // ── TCS3400: Color / Lux ────────────────────────────────────────────────────
  uint8_t r8, g8, b8;
  float lux;
  if (tcs3400_read(r8, g8, b8, lux)) {
    // Pick dominant color label
    const char *dom = (r8 > g8 && r8 > b8) ? "Red"
                    : (g8 > r8 && g8 > b8) ? "Green"
                    : (b8 > r8 && b8 > g8) ? "Blue"
                    : "White";
    Serial.printf("TCS340 | R:%d G:%d B:%d  Lux:%.1f  [%s]\n", r8, g8, b8, lux, dom);
  } else {
    Serial.println("TCS340 | ERR");
  }

  // ── QMC6309: Magnetometer / Heading ─────────────────────────────────────────
  float hdg, xG, yG, zG;
  if (qmc6309_read(hdg, xG, yG, zG))
    Serial.printf("QMC630 | X:%.3f Y:%.3f Z:%.3f G  Hdg:%.1f°\n", xG, yG, zG, hdg);
  else
    Serial.println("QMC630 | no data");

  Serial.println("---");

  digitalWrite(42, LOW);

  delay(1000);   // 1 Hz — do not reduce (SGP40 VOC algorithm requirement)
}
