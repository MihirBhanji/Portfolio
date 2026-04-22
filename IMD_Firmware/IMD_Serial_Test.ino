#include <Arduino.h>
#include <FlexCAN_T4.h>

// Teensy 4.1 pins:
// CAN3 TX = pin 30
// CAN3 RX = pin 31
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can3;

// Default Bender-Standard CAN IDs
static constexpr uint32_t ID_IMD_INFO_GENERAL = 0x37;
static constexpr uint32_t ID_IMD_REQUEST      = 0x22;
static constexpr uint32_t ID_IMD_RESPONSE     = 0x23;

// GET indices
static constexpr uint8_t IDX_R_ISO_STATUS      = 0x44;
static constexpr uint8_t IDX_THRESH_ERROR      = 0x46; // Isolation: Threshold_Error
static constexpr uint8_t IDX_THRESH_WARNING    = 0x4A; // Isolation: Threshold_Warning
static constexpr uint8_t IDX_R_ISO_CORRECTED   = 0x4C;
static constexpr uint8_t IDX_R_ISO_ORIGINAL    = 0x4E;
static constexpr uint8_t IDX_TIME_SINCE_LAST   = 0x50;
static constexpr uint8_t IDX_DEVICE_ACTIVITY   = 0x68;
static constexpr uint8_t IDX_WARN_ALARMS       = 0x6C;

// Keep request rate well under 10 Hz and only issue one request at a time.
elapsedMillis reqTimer;
static constexpr uint32_t REQ_PERIOD_MS = 150;   // ~6.7 Hz max request attempt rate
static constexpr uint32_t RESP_TIMEOUT_MS = 120; // spec says typical response < 100 ms

static const uint8_t requestList[] = {
  IDX_DEVICE_ACTIVITY,
  IDX_R_ISO_STATUS,
  IDX_THRESH_ERROR,
  IDX_THRESH_WARNING,
  IDX_R_ISO_CORRECTED,
  IDX_R_ISO_ORIGINAL,
  IDX_TIME_SINCE_LAST,
  IDX_WARN_ALARMS
};

static constexpr size_t NUM_REQUESTS = sizeof(requestList) / sizeof(requestList[0]);

size_t requestIdx = 0;
bool waitingForResponse = false;
uint8_t lastRequestedIndex = 0;
elapsedMillis responseTimer;

static inline uint16_t u16le(uint8_t lo, uint8_t hi) {
  return (uint16_t)lo | ((uint16_t)hi << 8);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}

  Serial.println("Teensy 4.1 iso175 CAN test starting...");

  Can3.setRX(31);
  Can3.setTX(30);
  Can3.begin();
  Can3.setBaudRate(500000);

  Can3.setMaxMB(16);
  Can3.enableFIFO();
  Can3.enableFIFOInterrupt();
  Can3.onReceive(canSniff);

  Serial.println("CAN3 started @ 500k");
  Serial.println("Listening for 0x37 and polling 0x22 -> 0x23 one request at a time.");
}

void loop() {
  Can3.events();

  // Timeout protection so polling doesn't stall forever if one response is missed.
  if (waitingForResponse && responseTimer > RESP_TIMEOUT_MS) {
    Serial.print("[WARN] Response timeout for index 0x");
    Serial.println(lastRequestedIndex, HEX);
    waitingForResponse = false;
  }

  if (!waitingForResponse && reqTimer >= REQ_PERIOD_MS) {
    reqTimer = 0;
    sendGet(requestList[requestIdx]);
    requestIdx = (requestIdx + 1) % NUM_REQUESTS;
  }
}

void printWarningsAlarmsBits(uint16_t bits) {
  Serial.print("Warnings/Alarms bits: 0x");
  Serial.print(bits, HEX);

  auto show = [&](uint8_t b, const char* name) {
    if (bits & (1u << b)) {
      Serial.print(" [");
      Serial.print(name);
      Serial.print("]");
    }
  };

  show(0,  "DeviceError");
  show(1,  "HV+ConnFail");
  show(2,  "HV-ConnFail");
  show(3,  "EarthConnFail");
  show(4,  "IsoAlarm");
  show(5,  "IsoWarning");
  show(6,  "IsoOutdated");
  show(7,  "UnbalanceAlarm");
  show(8,  "Undervoltage");
  show(9,  "UnsafeToStart");
  show(10, "EarthliftOpen");
  Serial.println();
}


void handleInfoGeneral(const CAN_message_t &msg) {
  if (msg.len < 6) {
    Serial.println("[0x37] Short frame");
    return;
  }

  uint16_t r_iso_corr_kohm = u16le(msg.buf[0], msg.buf[1]);
  uint8_t  r_iso_status    = msg.buf[2];
  uint8_t  meas_ctr        = msg.buf[3];
  uint8_t  warn_alarms_lsb = msg.buf[4];
  uint8_t  dev_activity    = msg.buf[5];

  Serial.print("[0x37 IMD_Info_General] R_iso_corrected=");
  if (r_iso_corr_kohm == 0xFFFF) {
    Serial.print("SNV");
  } else {
    Serial.print(r_iso_corr_kohm);
    Serial.print(" kOhm");
  }

  Serial.print(" | R_iso_status=0x");
  Serial.print(r_iso_status, HEX);

  Serial.print(" | measCtr=");
  Serial.print(meas_ctr);

  Serial.print(" | warnLSB=0x");
  Serial.print(warn_alarms_lsb, HEX);

  Serial.print(" | devActivity=");
  Serial.println(dev_activity);
}


void sendGet(uint8_t index) {
  CAN_message_t tx{};
  tx.id  = ID_IMD_REQUEST;
  tx.len = 8;
  tx.buf[0] = index;
  for (int i = 1; i < 8; i++) tx.buf[i] = 0xFF;

  if (Can3.write(tx)) {
    lastRequestedIndex = index;
    waitingForResponse = true;
    responseTimer = 0;

    Serial.print("[TX GET] index=0x");
    Serial.println(index, HEX);
  } else {
    Serial.println("[TX GET] write failed");
  }
}


void handleResponse(const CAN_message_t &msg) {
  if (msg.len < 1) return;

  uint8_t index = msg.buf[0];
  waitingForResponse = false;

  Serial.print("[0x23 IMD_Response] index=0x");
  Serial.print(index, HEX);
  Serial.print(" data:");

  for (int i = 1; i < msg.len; i++) {
    Serial.print(" ");
    if (msg.buf[i] < 0x10) Serial.print("0");
    Serial.print(msg.buf[i], HEX);
  }
  Serial.println();

  if (index == IDX_R_ISO_CORRECTED) {
    if (msg.len >= 3) {
      uint16_t v = u16le(msg.buf[1], msg.buf[2]);
      Serial.print("  -> R_iso_corrected = ");
      if (v == 0xFFFF) Serial.println("SNV");
      else {
        Serial.print(v);
        Serial.println(" kOhm");
      }
    }
  } else if (index == IDX_R_ISO_ORIGINAL) {
    if (msg.len >= 3) {
      uint16_t v = u16le(msg.buf[1], msg.buf[2]);
      Serial.print("  -> R_iso_original = ");
      if (v == 0xFFFF) Serial.println("SNV");
      else {
        Serial.print(v);
        Serial.println(" kOhm");
      }
    }
  } else if (index == IDX_R_ISO_STATUS) {
    if (msg.len >= 2) {
      Serial.print("  -> R_iso_status = 0x");
      Serial.println(msg.buf[1], HEX);
    }
  } else if (index == IDX_THRESH_ERROR) {
    if (msg.len >= 3) {
      uint16_t v = u16le(msg.buf[1], msg.buf[2]);
      Serial.print("  -> Isolation Threshold_Error = ");
      if (v == 0xFFFF) Serial.println("SNV");
      else {
        Serial.print(v);
        Serial.println(" kOhm");
      }
    }
  } else if (index == IDX_THRESH_WARNING) {
    if (msg.len >= 3) {
      uint16_t v = u16le(msg.buf[1], msg.buf[2]);
      Serial.print("  -> Isolation Threshold_Warning = ");
      if (v == 0xFFFF) Serial.println("SNV");
      else {
        Serial.print(v);
        Serial.println(" kOhm");
      }
    }
  } else if (index == IDX_DEVICE_ACTIVITY) {
    if (msg.len >= 2) {
      Serial.print("  -> Device_Activity = ");
      Serial.println(msg.buf[1]); // 0 init, 1 normal, 2 self test
    }
  } else if (index == IDX_TIME_SINCE_LAST) {
    if (msg.len >= 3) {
      uint16_t secs = u16le(msg.buf[1], msg.buf[2]);
      Serial.print("  -> Time since last measurement = ");
      if (secs == 0xFFFF) Serial.println("SNV");
      else {
        Serial.print(secs);
        Serial.println(" s");
      }
    }
  } else if (index == IDX_WARN_ALARMS) {
    uint16_t bits = 0;
    if (msg.len >= 3) bits = u16le(msg.buf[1], msg.buf[2]);
    else if (msg.len >= 2) bits = msg.buf[1];
    printWarningsAlarmsBits(bits);
  } else if (index == 0xFF) {
    // Error format:
    // byte0 = 0xFF, byte1 = error code, byte2 = requested index
    if (msg.len >= 3) {
      Serial.print("  -> IMD error code 0x");
      Serial.print(msg.buf[1], HEX);
      Serial.print(" for requested index 0x");
      Serial.println(msg.buf[2], HEX);
    }
  }
}


void canSniff(const CAN_message_t &msg) {
  if (msg.id == ID_IMD_INFO_GENERAL) {
    handleInfoGeneral(msg);
  } else if (msg.id == ID_IMD_RESPONSE) {
    handleResponse(msg);
  }
}