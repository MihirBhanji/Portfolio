// ================================
// BQ79600 (bridge) → BQ79616 stack bring-up (patched)
// - Uses Teensy 4.1 SPI0: MOSI=11, MISO=12, SCK=13
// - CS=10, SPI_RDY (nUART/SPI)=2, FAULT=4 (opt), RESET=3 (opt)
// - SPI Mode0, 4 MHz, BQ79600 FRMWRT frames + CRC16(ITU)
// - Keeps your registers.h map, VCELL/NTC math, and high-level flow
// ================================

#include "registers.h"
#include <Arduino.h>
#include <SPI.h>
#include <math.h>

// -------------------- Pins & SPI (bridge pinout) --------------------
static const uint8_t PIN_CS      = 10;   // -> BQ79600 nCS
static const uint8_t PIN_SPI_RDY = 2;    // <- BQ79600 nUART/SPI (SPI_RDY), pull-up
static const uint8_t PIN_RESET   = 3;    // optional (active-low to the stack/bridge if routed)
static const uint8_t PIN_FAULT   = 4;    // optional (NFAULT)

SPISettings bqSPI(4000000, MSBFIRST, SPI_MODE0);  // Bridge wants MODE0, ~4 MHz

// -------------------- NTC parameters (yours) --------------------
typedef struct {
  double R1_ohm;  // Divider top resistor
  double R0_ohm;  // NTC nominal @ T0
  double T0_K;    // Nominal temp in Kelvin
  double Beta;    // Beta constant
} ntc_params_t;

ntc_params_t my_ntc_params = {
  .R1_ohm = 10000.0,
  .R0_ohm = 10000.0,
  .T0_K   = 298.15,
  .Beta   = 3435.0
};

// -------------------- Small utils --------------------
static inline void delay_us(uint32_t us) { delayMicroseconds(us); }
static inline void csLow()  { digitalWrite(PIN_CS, LOW); }
static inline void csHigh() { digitalWrite(PIN_CS, HIGH); }

static bool waitSpiRdy(uint32_t timeout_ms = 50) {
  uint32_t t0 = millis();
  while (digitalRead(PIN_SPI_RDY) == LOW) {
    if ((millis() - t0) > timeout_ms) return false;
  }
  return true;
}

// Comm Clear: nCS low, 8 zero bits, nCS high
static void spiCommClear() {
  SPI.beginTransaction(bqSPI);
  csLow();
  SPI.transfer(0x00);
  csHigh();
  SPI.endTransaction();
  delayMicroseconds(50);
}

// Bridge pings (wake/standby/shutdown/hwreset): MOSI LOW while CS LOW, no clocks
static void spiPingLowUS(uint32_t us) {
  SPI.end(); // temporarily own pins
  pinMode(11, OUTPUT); // MOSI
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(11, HIGH);
  digitalWrite(PIN_CS, LOW);
  delayMicroseconds(2);
  digitalWrite(11, LOW);
  delayMicroseconds(us);
  digitalWrite(PIN_CS, HIGH);
  digitalWrite(11, HIGH);
  SPI.begin(); // restore SPI
}

// -------------------- Bridge protocol (FRMWRT) + CRC16(ITU) --------------------
#define FRMWRT_SGL_R       0x00
#define FRMWRT_SGL_W       0x10
#define FRMWRT_STK_R       0x20
#define FRMWRT_STK_W       0x30
#define FRMWRT_ALL_R       0x40
#define FRMWRT_ALL_W       0x50
#define FRMWRT_REV_ALL_W   0x70

static const uint16_t crc16_table[256] = {
  0x0000,0xC0C1,0xC181,0x0140,0xC301,0x03C0,0x0280,0xC241,0xC601,0x06C0,0x0780,0xC741,0x0500,0xC5C1,0xC481,0x0440,
  0xCC01,0x0CC0,0x0D80,0xCD41,0x0F00,0xCFC1,0xCE81,0x0E40,0x0A00,0xCAC1,0xCB81,0x0B40,0xC901,0x09C0,0x0880,0xC841,
  0xD801,0x18C0,0x1980,0xD941,0x1B00,0xDBC1,0xDA81,0x1A40,0x1E00,0xDEC1,0xDF81,0x1F40,0xDD01,0x1DC0,0x1C80,0xDC41,
  0x1400,0xD4C1,0xD581,0x1540,0xD701,0x17C0,0x1680,0xD641,0xD201,0x12C0,0x1380,0xD341,0x1100,0xD1C1,0xD081,0x1040,
  0xF001,0x30C0,0x3180,0xF141,0x3300,0xF3C1,0xF281,0x3240,0x3600,0xF6C1,0xF781,0x3740,0xF501,0x35C0,0x3480,0xF441,
  0x3C00,0xFCC1,0xFD81,0x3D40,0xFF01,0x3FC0,0x3E80,0xFE41,0xFA01,0x3AC0,0x3B80,0xFB41,0x3900,0xF9C1,0xF881,0x3840,
  0x2800,0xE8C1,0xE981,0x2940,0xEB01,0x2BC0,0x2A80,0xEA41,0xEE01,0x2EC0,0x2F80,0xEF41,0x2D00,0xEDC1,0xEC81,0x2C40,
  0xE401,0x24C0,0x2580,0xE541,0x2700,0xE7C1,0xE681,0x2640,0x2200,0xE2C1,0xE381,0x2340,0xE101,0x21C0,0x2080,0xE041,
  0xA001,0x60C0,0x6180,0xA141,0x6300,0xA3C1,0xA281,0x6240,0x6600,0xA6C1,0xA781,0x6740,0xA501,0x65C0,0x6480,0xA441,
  0x6C00,0xACC1,0xAD81,0x6D40,0xAF01,0x6FC0,0x6E80,0xAE41,0xAA01,0x6AC0,0x6B80,0xAB41,0x6900,0xA9C1,0xA881,0x6840,
  0x7800,0xB8C1,0xB981,0x7940,0xBB01,0x7BC0,0x7A80,0xBA41,0xBE01,0x7EC0,0x7F80,0xBF41,0x7D00,0xBDC1,0xBC81,0x7C40,
  0xB401,0x74C0,0x7580,0xB541,0x7700,0xB7C1,0xB681,0x7640,0x7200,0xB2C1,0xB381,0x7340,0xB101,0x71C0,0x7080,0xB041,
  0x5000,0x90C1,0x9181,0x5140,0x9301,0x53C0,0x5280,0x9241,0x9601,0x56C0,0x5780,0x9741,0x5500,0x95C1,0x9481,0x5440,
  0x9C01,0x5CC0,0x5D80,0x9D41,0x5F00,0x9FC1,0x9E81,0x5E40,0x5A00,0x9AC1,0x9B81,0x5B40,0x9901,0x59C0,0x5880,0x9841,
  0x8801,0x48C0,0x4980,0x8941,0x4B00,0x8BC1,0x8A81,0x4A40,0x4E00,0x8EC1,0x8F81,0x4F40,0x8D01,0x4DC0,0x4C80,0x8C41,
  0x4400,0x84C1,0x8581,0x4540,0x8701,0x47C0,0x4680,0x8641,0x8201,0x42C0,0x4380,0x8341,0x4100,0x81C1,0x8081,0x4040
};
static inline uint16_t CRC16(const uint8_t* p, size_t n) {
  uint16_t crc = 0xFFFF;
  while (n--) { crc ^= *p++; crc = crc16_table[crc & 0xFF] ^ (crc >> 8); }
  return crc;
}

// Build bridge frame and send
static int WriteFrame(uint8_t devID, uint16_t addr, const uint8_t* data, uint8_t len, uint8_t wrType) {
  uint8_t frame[64];
  uint8_t* p = frame;

  // Header = 0x80 | wrType | (len-1) on WRITE (wrType has bit 0x10 set)
  *p++ = (uint8_t)(0x80 | wrType | ((wrType & 0x10) ? (uint8_t)(len - 1) : 0x00));

  // devID for SGL ops
  if (wrType == FRMWRT_SGL_W || wrType == FRMWRT_SGL_R) *p++ = (uint8_t)(devID & 0xFF);

  // Address
  *p++ = (uint8_t)((addr >> 8) & 0xFF);
  *p++ = (uint8_t)(addr & 0xFF);

  // Payload (writes only)
  if (wrType & 0x10) {
    for (uint8_t i = 0; i < len; ++i) *p++ = data[i];
  }

  // CRC
  uint16_t crc = CRC16(frame, (size_t)(p - frame));
  *p++ = (uint8_t)(crc & 0xFF);
  *p++ = (uint8_t)((crc >> 8) & 0xFF);

  size_t pktLen = (size_t)(p - frame);

  if (!waitSpiRdy(50)) delayMicroseconds(50);
  SPI.beginTransaction(bqSPI);
  csLow();
  for (size_t i = 0; i < pktLen; ++i) SPI.transfer(frame[i]);
  csHigh();
  SPI.endTransaction();

  return (int)pktLen;
}

// Read request is same header with single-byte payload = (len-1)
static inline int ReadFrameReq(uint8_t devID, uint16_t addr, uint8_t bytesToReturn, uint8_t wrType) {
  uint8_t ret = (uint8_t)(bytesToReturn - 1);
  return WriteFrame(devID, addr, &ret, 1, wrType);
}

// Read response into pData; for ALL, response repeats per board
static int ReadReg(uint8_t devID, uint16_t addr, uint8_t* pData, uint8_t len, uint8_t wrType, uint8_t numBoards = 1) {
  if (!pData || len == 0) return 0;
  int bytesPerBoard = len + 6; // approx header(4)+payload(len)+crc(2)
  int exp = (wrType == FRMWRT_ALL_R) ? bytesPerBoard * numBoards : bytesPerBoard;

  ReadFrameReq(devID, addr, len, wrType);
  memset(pData, 0, exp);

  if (!waitSpiRdy(10)) delayMicroseconds(50);

  SPI.beginTransaction(bqSPI);
  csLow();
  for (int i = 0; i < exp; ++i) pData[i] = SPI.transfer(0x00);
  csHigh();
  SPI.endTransaction();

  // (Optional) CRC verify per chunk here
  return exp;
}

// -------------------- “bq_*” API shim (keeps your existing code) --------------------
static uint8_t g_dev = 0x00; // selected device id (0..63)
static inline void bq_select_device(uint8_t dev) { g_dev = (dev & 0x3F); }

// Single-device WRITE/READ via bridge
static bool bq_writeN_dev(uint8_t dev, uint16_t reg, const uint8_t *data, uint8_t n) {
  return WriteFrame(dev, reg, data, n, FRMWRT_SGL_W) > 0;
}
static inline bool bq_write8_dev(uint8_t dev, uint16_t reg, uint8_t val) {
  return bq_writeN_dev(dev, reg, &val, 1);
}
static inline bool bq_write8_bcast(uint16_t reg, uint8_t val) {
  return WriteFrame(0, reg, &val, 1, FRMWRT_ALL_W) > 0;
}
static inline bool bq_writeN(uint16_t reg, const uint8_t *data, uint8_t n) {
  return bq_writeN_dev(g_dev, reg, data, n);
}
static inline bool bq_write8(uint16_t reg, uint8_t val) {
  return bq_write8_dev(g_dev, reg, val);
}

static bool bq_readN(uint16_t reg, uint8_t *out, uint8_t n) {
  uint8_t buf[64] = {0};
  int got = ReadReg(g_dev, reg, buf, n, FRMWRT_SGL_R);
  if (got <= 0) return false;
  // First board payload starts after 4 bytes header
  for (uint8_t i = 0; i < n; ++i) out[i] = buf[4 + i];
  return true;
}

static inline bool bq_read8(uint16_t reg, uint8_t &val) {
  uint8_t t = 0;
  bool ok = bq_readN(reg, &t, 1);
  if (ok) val = t;
  return ok;
}

static inline bool bq_read16(uint16_t reg_hi, uint16_t reg_lo, uint16_t &val) {
  uint8_t hi = 0, lo = 0;
  if (!bq_read8(reg_hi, hi)) return false;
  if (!bq_read8(reg_lo, lo)) return false;
  val = ((uint16_t)hi << 8) | lo;
  return true;
}

// -------------------- Wake / Reset via bridge --------------------
#define CONTROL1_ADDR_WR   (1u << 0)  // matches your registers.h bit note
#define CONTROL1_SEND_WAKE 0x20       // bridge bit to forward WAKE up the isoSPI

static void bq_hw_reset_pulse() {
  // If wired active-low, pulse LOW briefly
  pinMode(PIN_RESET, OUTPUT);
  digitalWrite(PIN_RESET, HIGH);
  delay(2);
  digitalWrite(PIN_RESET, LOW);
  delay(3);
  digitalWrite(PIN_RESET, HIGH);
  delay(10);
}

static void bq_wake_bridge(uint8_t num_devs) {
  // Double WAKE ping at bridge level
  spiPingLowUS(2750); delay(4);
  spiPingLowUS(2750); delay(4);

  // Comm Clear
  spiCommClear();

  // Forward WAKE onto the stack
  uint8_t v = CONTROL1_SEND_WAKE;
  WriteFrame(0, CONTROL1, &v, 1, FRMWRT_SGL_W);

  // Allow ≈11.6 ms per device to wake DLLs
  delay(12 * max<uint8_t>(1, num_devs));
}

// -------------------- Conversion helpers (yours) --------------------
static inline int16_t bq_raw_to_i16(uint16_t raw) { return (int16_t)raw; }
static inline double bq_vcell_uV(uint16_t raw16)   { return (double)bq_raw_to_i16(raw16) * 190.73; }
static inline double bq_gpio_uV(uint16_t raw16)    { return (double)bq_raw_to_i16(raw16) * 152.59; }
static inline double bq_tsref_uV(uint16_t raw16)   { return (double)bq_raw_to_i16(raw16) * 169.54; }
static inline double bq_busbar_uV(uint16_t raw16)  { return (double)bq_raw_to_i16(raw16) * 30.52; }

static inline double bq_uV_to_V(double uV)         { return uV * 1e-6; }
static inline double bq_gpio_V(uint16_t raw16)     { return bq_uV_to_V(bq_gpio_uV(raw16));  }
static inline double bq_tsref_V(uint16_t raw16)    { return bq_uV_to_V(bq_tsref_uV(raw16)); }

static inline double bq_ntc_ratio(uint16_t gpio_raw, uint16_t tsref_raw) {
  const double ts_uV = bq_tsref_uV(tsref_raw);
  const double gp_uV = bq_gpio_uV(gpio_raw);
  if (ts_uV <= 0.0) return NAN;
  return gp_uV / ts_uV; // ratiometric (GPIO/TSREF)
}
static inline double bq_ntc_resistance_ohm(uint16_t gpio_raw, uint16_t tsref_raw, double R1_ohm) {
  const double r = bq_ntc_ratio(gpio_raw, tsref_raw);
  if (!(r > 0.0 && r < 1.0)) return NAN;
  return (r * R1_ohm) / (1.0 - r);
}
static inline double ntc_beta_to_C(double R_ntc, const ntc_params_t *p) {
  if (!(R_ntc > 0.0) || p->R0_ohm <= 0.0 || p->Beta <= 0.0 || p->T0_K <= 0.0) return NAN;
  const double lnR  = log(R_ntc / p->R0_ohm);
  const double invT = (1.0 / p->T0_K) + (lnR / p->Beta);
  return (1.0 / invT) - 273.15;
}
static inline double bq_ntc_C(uint16_t gpio_raw, uint16_t tsref_raw, const ntc_params_t *p) {
  return ntc_beta_to_C(bq_ntc_resistance_ohm(gpio_raw, tsref_raw, p->R1_ohm), p);
}

// -------------------- MAIN ADC control --------------------
static bool bq_main_adc_start_continuous() {
  uint8_t v = 0x00;
  v |= (1u << 2);  // MAIN_GO
  v |= 0x02;       // MAIN_MODE = 10b (continuous)
  return bq_write8(ADC_CTRL1, v);
}

static bool bq_wait_main_adc_ready(uint32_t timeout_ms = 100) {
  const uint32_t t0 = millis();
  while ((millis() - t0) < timeout_ms) {
    uint8_t s = 0;
    if (bq_read8(ADC_STAT1, s) && (s & DRDY_MAIN_ADC_MASK)) return true;
    delay(1);
  }
  return false;
}

// -------------------- GPIO config for NTCs --------------------
static uint8_t set_field_2pins(uint8_t existing, uint8_t mode_low3, uint8_t mode_high3) {
  uint8_t v = existing & 0xC0;
  v |= ((mode_low3  & 0x7) << 0);
  v |= ((mode_high3 & 0x7) << 3);
  return v;
}
static inline uint8_t mode_for_ntc(bool enable) {
  return enable ? GPIO_MODE_ADC_OTUT : GPIO_MODE_DISABLE;
}
static bool bq_config_ntc_gpios(uint8_t gpio_mask) {
  const uint8_t m1 = mode_for_ntc(gpio_mask & (1u << 0));
  const uint8_t m2 = mode_for_ntc(gpio_mask & (1u << 1));
  const uint8_t m3 = mode_for_ntc(gpio_mask & (1u << 2));
  const uint8_t m4 = mode_for_ntc(gpio_mask & (1u << 3));
  const uint8_t m5 = mode_for_ntc(gpio_mask & (1u << 4));
  const uint8_t m6 = mode_for_ntc(gpio_mask & (1u << 5));
  const uint8_t m7 = mode_for_ntc(gpio_mask & (1u << 6));
  const uint8_t m8 = mode_for_ntc(gpio_mask & (1u << 7));

  uint8_t conf1 = set_field_2pins(0, m1, m2);
  uint8_t conf2 = set_field_2pins(0, m3, m4);
  uint8_t conf3 = set_field_2pins(0, m5, m6);
  uint8_t conf4 = set_field_2pins(0, m7, m8);

  return bq_write8(GPIO_CONF1, conf1)
      && bq_write8(GPIO_CONF2, conf2)
      && bq_write8(GPIO_CONF3, conf3)
      && bq_write8(GPIO_CONF4, conf4);
}

// -------------------- High-level bring-up --------------------
static bool bq_basic_config_all_temps(uint8_t ntc_gpio_mask) {
  // (1) 16S (verify encoding for your PG; you used 0x0A previously)
  if (!bq_write8(ACTIVE_CELL, 0x0A)) return false;

  // (2) TSREF enable
  uint8_t c2 = 0;
  if (!bq_read8(CONTROL2, c2)) return false;
  c2 |= (1u << 0);
  if (!bq_write8(CONTROL2, c2)) return false;

  // (3) Thermistor GPIOs
  if (!bq_config_ntc_gpios(ntc_gpio_mask)) return false;

  // (4) MAIN ADC continuous + wait
  if (!bq_main_adc_start_continuous()) return false;
  (void)bq_wait_main_adc_ready(100);
  return true;
}

// -------------------- Reading routines (unchanged) --------------------
static bool read_tsref_raw(uint16_t &ts_raw) { return bq_read16(TSREF_HI, TSREF_LO, ts_raw); }

static bool read_gpio_raw(int gpioN, uint16_t &g_raw) {
  switch (gpioN) {
    case 1: return bq_read16(GPIO1_HI, GPIO1_LO, g_raw);
    case 2: return bq_read16(GPIO2_HI, GPIO2_LO, g_raw);
    case 3: return bq_read16(GPIO3_HI, GPIO3_LO, g_raw);
    case 4: return bq_read16(GPIO4_HI, GPIO4_LO, g_raw);
    case 5: return bq_read16(GPIO5_HI, GPIO5_LO, g_raw);
    case 6: return bq_read16(GPIO6_HI, GPIO6_LO, g_raw);
    case 7: return bq_read16(GPIO7_HI, GPIO7_LO, g_raw);
    case 8: return bq_read16(GPIO8_HI, GPIO8_LO, g_raw);
    default: return false;
  }
}

static void read_all_vcells() {
  for (int ch = 16; ch >= 1; --ch) {
    uint16_t v_raw = 0;
    switch (ch) {
      case 16: bq_read16(VCELL16_HI, VCELL16_LO, v_raw); break;
      case 15: bq_read16(VCELL15_HI, VCELL15_LO, v_raw); break;
      case 14: bq_read16(VCELL14_HI, VCELL14_LO, v_raw); break;
      case 13: bq_read16(VCELL13_HI, VCELL13_LO, v_raw); break;
      case 12: bq_read16(VCELL12_HI, VCELL12_LO, v_raw); break;
      case 11: bq_read16(VCELL11_HI, VCELL11_LO, v_raw); break;
      case 10: bq_read16(VCELL10_HI, VCELL10_LO, v_raw); break;
      case  9: bq_read16(VCELL9_HI,  VCELL9_LO,  v_raw); break;
      case  8: bq_read16(VCELL8_HI,  VCELL8_LO,  v_raw); break;
      case  7: bq_read16(VCELL7_HI,  VCELL7_LO,  v_raw); break;
      case  6: bq_read16(VCELL6_HI,  VCELL6_LO,  v_raw); break;
      case  5: bq_read16(VCELL5_HI,  VCELL5_LO,  v_raw); break;
      case  4: bq_read16(VCELL4_HI,  VCELL4_LO,  v_raw); break;
      case  3: bq_read16(VCELL3_HI,  VCELL3_LO,  v_raw); break;
      case  2: bq_read16(VCELL2_HI,  VCELL2_LO,  v_raw); break;
      case  1: bq_read16(VCELL1_HI,  VCELL1_LO,  v_raw); break;
    }
    Serial.print("VCELL"); Serial.print(ch);
    Serial.print(" = ");   Serial.print(bq_vcell_uV(v_raw) * 1e-6, 4);
    Serial.println(" V");
  }
}

static void read_all_ntc_gpios(uint8_t ntc_gpio_mask) {
  (void)bq_wait_main_adc_ready(50);

  uint16_t ts_raw = 0;
  if (!read_tsref_raw(ts_raw)) {
    Serial.println("TSREF read failed");
    return;
  }

  for (int n = 1; n <= 8; ++n) {
    if (!(ntc_gpio_mask & (1u << (n - 1)))) continue;
    uint16_t g_raw = 0;
    if (!read_gpio_raw(n, g_raw)) {
      Serial.print("GPIO"); Serial.print(n); Serial.println(" read failed");
      continue;
    }
    const double C    = bq_ntc_C(g_raw, ts_raw, &my_ntc_params);
    const double Rntc = bq_ntc_resistance_ohm(g_raw, ts_raw, my_ntc_params.R1_ohm);

    Serial.print("GPIO"); Serial.print(n); Serial.print(" : ");
    if (isnan(C))  Serial.print("Temp=Invalid");
    else           { Serial.print(C, 2); Serial.print(" °C"); }

    Serial.print("  (Rntc=");
    if (isnan(Rntc)) Serial.print("Invalid");
    else             Serial.print(Rntc, 2);
    Serial.println(" Ω)");
  }
}

// -------------------- Auto-addressing via bridge --------------------
#define COMM_CTRL     0x0308
#define CONTROL1      0x0309
#define DIR0_ADDR     0x0306
#define COMM_STACK_DEV     (1u << 1)
#define COMM_TOP_STACK     (1u << 0)

static bool bq_auto_address_chain(uint8_t base_addr, uint8_t num_devs) {
  if (num_devs == 0) return false;
  Serial.println("Auto-addressing start...");

  // (1) Arm address capture on ALL
  uint8_t c1 = CONTROL1_ADDR_WR;
  if (WriteFrame(0, CONTROL1, &c1, 1, FRMWRT_ALL_W) <= 0) return false;
  delay(2);

  // (2) Broadcast sequential addresses
  for (uint8_t i = 0; i < num_devs; ++i) {
    uint8_t addr = (uint8_t)(base_addr + i);
    if (WriteFrame(0, DIR0_ADDR, &addr, 1, FRMWRT_ALL_W) <= 0) return false;
    delay(2);
  }

  // (3) Everyone STACK_DEV=1
  uint8_t comm_all = COMM_STACK_DEV;
  if (WriteFrame(0, COMM_CTRL, &comm_all, 1, FRMWRT_ALL_W) <= 0) return false;

  // Base device STACK_DEV=0
  uint8_t comm_base = (uint8_t)(comm_all & ~COMM_STACK_DEV);
  if (WriteFrame(base_addr, COMM_CTRL, &comm_base, 1, FRMWRT_SGL_W) <= 0) return false;

  // Top device TOP_STACK=1
  const uint8_t top_addr = (uint8_t)(base_addr + num_devs - 1);
  uint8_t comm_top = (uint8_t)(COMM_TOP_STACK);
  if (WriteFrame(top_addr, COMM_CTRL, &comm_top, 1, FRMWRT_SGL_W) <= 0) return false;

  // (4) Disarm ADDR_WR (best-effort read-modify-write on base)
  bq_select_device(base_addr);
  uint8_t c1r = 0;
  if (bq_read8(CONTROL1, c1r)) {
    c1r &= ~CONTROL1_ADDR_WR;
    (void)bq_write8(CONTROL1, c1r);
  }

  Serial.print("Auto-addressing done. Base=0x");
  Serial.print(base_addr, HEX);
  Serial.print(", Top=0x");
  Serial.println(top_addr, HEX);
  return true;
}

// -------------------- Arduino entry points --------------------
static const uint8_t NTC_GPIO_MASK_DEFAULT = 0xFF; // GPIO1..8 thermistors

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  pinMode(PIN_CS, OUTPUT);   csHigh();
  pinMode(PIN_SPI_RDY, INPUT_PULLUP);
  pinMode(PIN_FAULT, INPUT_PULLUP);
  pinMode(PIN_RESET, OUTPUT); digitalWrite(PIN_RESET, HIGH);

  SPI.begin();
  delay(10);

  // Optional: small HW reset pulse if wired
  bq_hw_reset_pulse();

  // Wake the bridge + forward wake to stack (set your device count)
  const uint8_t num_devs = 6; // <-- set to your actual stack size
  bq_wake_bridge(num_devs);

  // Auto-address through the bridge
  const uint8_t base = 0x00;
  if (!bq_auto_address_chain(base, num_devs)) {
    Serial.println("Auto-addressing failed");
  }
  bq_select_device(base);

  // Configure temps & start MAIN ADC (via bridge now)
  if (!bq_basic_config_all_temps(NTC_GPIO_MASK_DEFAULT)) {
    Serial.println("Basic config failed");
  }

  Serial.println("BQ79600 bridge + BQ79616 bring-up complete.");
}

void loop() {
  (void)bq_wait_main_adc_ready(50);
  read_all_vcells();
  read_all_ntc_gpios(NTC_GPIO_MASK_DEFAULT);
  Serial.println("----");
  delay(200);
}