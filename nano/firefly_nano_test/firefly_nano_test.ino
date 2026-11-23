// ============================================================
// Fireflies Nano — BENCH TEST (no radio transmit)  [RAK3172 / RUI3]
//
//   SAFE TO RUN WITHOUT AN ANTENNA: this sketch never transmits.
//   (Transmitting with nothing on the U.FL connector can damage the
//   RAK3172's power amplifier.)
//
// Same sensors, GPS, timing and packet building as firefly_nano_tx,
// but everything goes to the Serial Monitor (115200) instead of the air:
//   - every reading (temperature, humidity, GPS) as it is taken
//   - the exact 128-byte packet the real firmware would send, in hex
//   - a status line every 10 s
//   - the radio settings read back from the module (configured, not used)
//
// Arduino IDE: board "WisDuo RAK3172 Evaluation Board"
// (or "WisDuo RAK3172-T Board" for the -T module). Library: TinyGPSPlus.
// ============================================================

#include <Wire.h>
#include <TinyGPSPlus.h>
#include "gps_status.h"     // GPS state bits for the flags byte (same folder)

#define NODE_ID   1          // only used to label the test packets

// ---------- Pin map (Fireflies Nano PCB) ----------
#define PIN_LED   PA9        // LED via 470R, active high
#define PIN_PPS   PA0        // MAX-M10S TIMEPULSE

// ---------- Test output options ----------
#define PRINT_READINGS  1    // 1 = print every reading (10 lines per second at CYCLE_MS 1000)
#define PRINT_PACKETS   1    // 1 = print each would-be packet as hex
#define ECHO_NMEA       0    // 1 = also echo the raw GPS NMEA sentences (very chatty)
#define CHECK_RADIO     1    // 1 = configure the radio and read it back (no transmit)

// ---------- Radio settings (checked, never used to transmit) ----------
#define LORA_FREQ_HZ     915000000
#define LORA_BW_KHZ      125
#define LORA_SF          7
#define LORA_CR_CODE     0        // 0 = 4/5
#define LORA_PREAMBLE    8
#define LORA_SYNCWORD    0x1424   // = SX1276 sync word 0x12
#define CONFIG_ID        "FF-915.0-SF7-BW125-CR45-SW12-PRE8-CRC"

// ---------- Timing (same as firefly_nano_tx) ----------
#define CYCLE_MS           1000
#define BATCH_SIZE         10
#define SAMPLE_INTERVAL_MS (CYCLE_MS / BATCH_SIZE)
#define SLOT_MS            500
#define SLOT_WINDOW_MS     120
#define GPS_STALE_MS       5000
#define STATUS_EVERY_MS    10000
#define LED_FLASH_MS       50

// ---------- Packet format (identical to firefly_nano_tx; flags bits 2-5 = GPS state, see gps_status.h) ----------
#pragma pack(1)
struct Reading {
  int16_t  tempC_x10;
  uint8_t  humidity;
  int32_t  lat_x1e6;
  int32_t  lng_x1e6;
  uint8_t  gpsValid;
};
#pragma pack()
static_assert(sizeof(struct Reading) == 12, "Reading must be 12 bytes to match receiver.py");

#define RH_HEADER_LEN   4
#define APP_HEADER_LEN  4
#define FLAG_TIME_SYNC  0x01
#define FLAG_GPS_FIX    0x02
#define TEMP_SENTINEL   (-32768)
#define RH_SENTINEL     255

TinyGPSPlus gps;

struct Reading batch[BATCH_SIZE];
uint8_t  batchIndex = 0;
uint16_t pktSeq     = 0;

// ---------- Time discipline (same as firefly_nano_tx) ----------
unsigned long secondBoundaryMs  = 0;
unsigned long lastGpsTimeMs     = 0;
bool          haveTimeRef       = false;
bool          lastAnchorUsedPps = false;
long          lastFiredCycle    = -1;
unsigned long anchorUtcSec      = 0;
uint32_t      lastAnchorTimeVal = 0xFFFFFFFF;
unsigned long lastFireMs        = 0;

volatile unsigned long ppsMs    = 0;
volatile uint32_t      ppsCount = 0;

unsigned long lastSample   = 0;
unsigned long lastStatus   = 0;
unsigned long ledOnSinceMs = 0;
bool          ledOn        = false;

bool     sensorFound = false;
uint32_t sensorErr   = 0, sensorOk = 0;
float    lastTempC   = NAN, lastRh = NAN;

void onPps(void) { ppsMs = millis(); ppsCount++; }

// ============================================================
// SHT4x driver (plain Wire)
// ============================================================
uint8_t shtAddr = 0x44;   // -A parts 0x44, -B parts 0x45; both are tried
#define SHT4X_MEASURE_MED 0xF6
#define SHT4X_SOFT_RESET  0x94
#define SHT4X_READ_SERIAL 0x89
uint32_t shtSerial = 0;

uint8_t shtCrc8(const uint8_t *d, uint8_t n) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < n; i++) {
    crc ^= d[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31) : (uint8_t)(crc << 1);
  }
  return crc;
}

bool shtCommand(uint8_t cmd) {
  Wire.beginTransmission(shtAddr);
  Wire.write(cmd);
  return Wire.endTransmission() == 0;
}

bool shtRead6(uint8_t *buf) {
  if (Wire.requestFrom(shtAddr, (uint8_t)6) != 6) return false;
  for (uint8_t i = 0; i < 6; i++) buf[i] = Wire.read();
  return shtCrc8(buf, 2) == buf[2] && shtCrc8(buf + 3, 2) == buf[5];
}

bool shtProbe(uint8_t addr) {
  shtAddr = addr;
  if (!shtCommand(SHT4X_SOFT_RESET)) return false;
  delay(2);
  if (!shtCommand(SHT4X_READ_SERIAL)) return false;
  delay(2);
  uint8_t buf[6];
  if (!shtRead6(buf)) return false;
  shtSerial = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16)
            | ((uint32_t)buf[3] << 8)  | buf[4];
  return true;
}

bool shtBegin() { return shtProbe(0x44) || shtProbe(0x45); }

bool shtMeasure(float &tempC, float &rh) {
  if (!shtCommand(SHT4X_MEASURE_MED)) return false;
  delay(6);
  uint8_t buf[6];
  if (!shtRead6(buf)) return false;
  uint16_t rawT  = ((uint16_t)buf[0] << 8) | buf[1];
  uint16_t rawRh = ((uint16_t)buf[3] << 8) | buf[4];
  tempC = -45.0f + 175.0f * rawT / 65535.0f;
  rh    = -6.0f + 125.0f * rawRh / 65535.0f;
  if (rh < 0.0f)   rh = 0.0f;
  if (rh > 100.0f) rh = 100.0f;
  return true;
}

// ============================================================
void i2cScan() {
  Serial.print("I2C scan:");
  uint8_t found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print(" 0x"); if (a < 16) Serial.print("0"); Serial.print(a, HEX);
      found++;
    }
  }
  Serial.println(found ? "" : " nothing found (check SHT4x, pull-ups, PA11/PA12)");
}

#if CHECK_RADIO
void checkRadio() {
  if (api.lora.nwm.get() != 0) {
    Serial.println("Switching RAK3172 to LoRa P2P mode, rebooting...");
    api.lora.nwm.set();
    delay(200);
    api.system.reboot();
  }
  api.lora.precv(0);   // make sure the radio is idle (not receiving, not sending)

  bool ok = true;
  ok &= api.lora.pfreq.set(LORA_FREQ_HZ);
  ok &= api.lora.psf.set(LORA_SF);
  ok &= api.lora.pbw.set(LORA_BW_KHZ);
  ok &= api.lora.pcr.set(LORA_CR_CODE);
  ok &= api.lora.ppl.set(LORA_PREAMBLE);
  ok &= api.lora.syncword.set(LORA_SYNCWORD);
  ok &= api.lora.iqInver.set(false);
  ok &= api.lora.encry.set(false);

  Serial.print("Radio check (NOT transmitting): ");
  Serial.println(ok ? "settings accepted" : "a setting was REJECTED");
  Serial.print("  read-back: ");
  Serial.print(api.lora.pfreq.get());  Serial.print(" Hz  SF");
  Serial.print(api.lora.psf.get());    Serial.print("  BW");
  uint32_t bw = api.lora.pbw.get();    // returns a code: 0 = 125, 1 = 250, 2 = 500 kHz
  Serial.print(bw == 0 ? 125 : bw == 1 ? 250 : bw == 2 ? 500 : bw); Serial.print("  CRcode ");
  Serial.print(api.lora.pcr.get());    Serial.print("  pre ");
  Serial.print(api.lora.ppl.get());    Serial.print("  sync 0x");
  Serial.println(api.lora.syncword.get(), HEX);
}
#endif

// ============================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  delay(2000);
  digitalWrite(PIN_LED, LOW);

  Serial.println("========================================");
  Serial.println("Fireflies Nano BENCH TEST - radio transmit DISABLED");
  Serial.println("Safe to run with no antenna. All data goes to Serial.");
  Serial.print ("Config "); Serial.println(CONFIG_ID);
  Serial.print ("Sampling every "); Serial.print(SAMPLE_INTERVAL_MS);
  Serial.print (" ms, one test packet per "); Serial.print(CYCLE_MS); Serial.println(" ms");
  Serial.print ("Firmware: "); Serial.println(api.system.firmwareVersion.get());
  Serial.println("========================================");

#if CHECK_RADIO
  checkRadio();
#endif

  Serial1.begin(9600, RAK_CUSTOM_MODE);   // GPS, PB6/PB7
  Serial.println("GPS: Serial1 (PB6/PB7) @ 9600");

  pinMode(PIN_PPS, INPUT);
  attachInterrupt(PIN_PPS, onPps, RISING);

  Wire.begin();
  i2cScan();
  sensorFound = shtBegin();
  if (sensorFound) {
    Serial.print("SHT4x: ready at 0x"); Serial.print(shtAddr, HEX);
    Serial.print(", serial 0x"); Serial.println(shtSerial, HEX);
  } else {
    Serial.println("SHT4x: NOT FOUND - readings will show the sentinel values");
  }

  Serial.println();
#if PRINT_READINGS
  Serial.println("R  <pkt>.<n>  temp_C  RH_%  lat  lng  gps  | sats  hdop  UTC  time");
#endif
}

// ============================================================
void updateTimeRef() {
  if (!gps.time.isValid() || gps.time.age() >= 200) return;
  if (!gps.date.isValid() || gps.date.year() < 2024) return;
  uint32_t t = gps.time.value();
  if (t == lastAnchorTimeVal) return;
  lastAnchorTimeVal = t;

  unsigned long nmeaMs   = millis() - gps.time.age();
  unsigned long boundary = nmeaMs;
  lastAnchorUsedPps = false;

  noInterrupts();
  unsigned long p  = ppsMs;
  uint32_t      pc = ppsCount;
  interrupts();
  if (pc > 0 && (long)(nmeaMs - p) >= 0 && (nmeaMs - p) < 1000) {
    boundary = p;
    lastAnchorUsedPps = true;
  }

  secondBoundaryMs = boundary;
  anchorUtcSec     = gps.time.hour() * 3600UL
                   + gps.time.minute() * 60UL
                   + gps.time.second();
  lastGpsTimeMs    = millis();
  haveTimeRef      = true;
}

bool timeIsSynced() {
  return haveTimeRef && (millis() - lastGpsTimeMs) < GPS_STALE_MS;
}

unsigned long utcMsOfDay() {
  return anchorUtcSec * 1000UL + (millis() - secondBoundaryMs);
}

unsigned long msIntoCycle() {
  if (!haveTimeRef) return millis() % CYCLE_MS;
  return utcMsOfDay() % CYCLE_MS;
}

long cycleNumber() {
  if (!haveTimeRef) return (long)(millis() / CYCLE_MS);
  return (long)(utcMsOfDay() / CYCLE_MS);
}

bool gpsFixFresh() {
  return gps.location.isValid() && gps.location.age() < 5000;
}

const char *timeLabel() {
  if (!timeIsSynced())  return "free-run";
  return lastAnchorUsedPps ? "GPS+PPS" : "GPS";
}

void print2(int v) { if (v < 10) Serial.print("0"); Serial.print(v); }

// ============================================================
void takeSample() {
  struct Reading &r = batch[batchIndex];

  float tempC, rh;
  bool shtOk = sensorFound && shtMeasure(tempC, rh);
  if (shtOk) {
    sensorOk++;
    lastTempC   = tempC;
    lastRh      = rh;
    r.tempC_x10 = (int16_t)lroundf(tempC * 10.0f);
    r.humidity  = (uint8_t)lroundf(rh);
  } else {
    if (sensorFound) sensorErr++;
    r.tempC_x10 = TEMP_SENTINEL;
    r.humidity  = RH_SENTINEL;
  }

  bool fix = gpsFixFresh();
  r.lat_x1e6 = fix ? (int32_t)lround(gps.location.lat() * 1e6) : 0;
  r.lng_x1e6 = fix ? (int32_t)lround(gps.location.lng() * 1e6) : 0;
  r.gpsValid = fix ? 1 : 0;

#if PRINT_READINGS
  Serial.print("R  "); Serial.print(pktSeq); Serial.print("."); Serial.print(batchIndex);
  Serial.print("  ");
  if (shtOk) { Serial.print(tempC, 2); Serial.print("  "); Serial.print(rh, 1); }
  else       Serial.print("SHT-FAIL  -");
  Serial.print("  ");
  if (fix) {
    Serial.print(gps.location.lat(), 6); Serial.print("  ");
    Serial.print(gps.location.lng(), 6); Serial.print("  1");
  } else {
    Serial.print("-  -  0");
  }
  Serial.print("  | ");
  Serial.print(gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
  Serial.print("  ");
  if (gps.hdop.isValid()) Serial.print(gps.hdop.hdop(), 1); else Serial.print("-");
  Serial.print("  ");
  if (gps.time.isValid() && gps.date.isValid() && gps.date.year() >= 2024) {
    print2(gps.time.hour()); Serial.print(":");
    print2(gps.time.minute()); Serial.print(":");
    print2(gps.time.second());
  } else {
    Serial.print("--:--:--");
  }
  Serial.print("  "); Serial.println(timeLabel());
#endif

  batchIndex++;
}

// ============================================================
// Build the packet exactly as firefly_nano_tx does, then print it instead of sending.
void emitPacket() {
  if (batchIndex == 0) return;

  uint8_t pkt[RH_HEADER_LEN + APP_HEADER_LEN + BATCH_SIZE * sizeof(struct Reading)];
  pkt[0] = 0xFF;
  pkt[1] = (uint8_t)NODE_ID;
  pkt[2] = (uint8_t)(pktSeq & 0xFF);
  pkt[3] = 0x00;

  uint8_t flags = 0;
  if (timeIsSynced()) flags |= FLAG_TIME_SYNC;
  if (gpsFixFresh())  flags |= FLAG_GPS_FIX;
  noInterrupts();
  unsigned long pMs = ppsMs;
  uint32_t      pCt = ppsCount;
  interrupts();
  flags |= gpsStatusFlags(gpsFixFresh(), pCt, pMs);

  pkt[4] = (uint8_t)NODE_ID;
  pkt[5] = (uint8_t)(pktSeq & 0xFF);
  pkt[6] = (uint8_t)((pktSeq >> 8) & 0xFF);
  pkt[7] = flags;
  memcpy(pkt + RH_HEADER_LEN + APP_HEADER_LEN, batch, batchIndex * sizeof(struct Reading));
  uint8_t pktLen = RH_HEADER_LEN + APP_HEADER_LEN + batchIndex * sizeof(struct Reading);

#if PRINT_PACKETS
  Serial.print("PKT seq "); Serial.print(pktSeq);
  Serial.print("  slot "); Serial.print(msIntoCycle());
  Serial.print(" ms  "); Serial.print(pktLen);
  Serial.print(" B  flags 0x"); Serial.print(flags, HEX);
  Serial.print("  GPS "); Serial.print(gpsStateName((flags & GPS_STATE_MASK) >> GPS_STATE_SHIFT));
  Serial.print(", "); Serial.print(gpsSatsHeard()); Serial.print(" heard");
  Serial.print((flags & FLAG_PPS_LOCK) ? ", PPS" : "");
  Serial.print("  ["); Serial.print(timeLabel()); Serial.println("]  (not transmitted)");
  for (uint8_t i = 0; i < pktLen; i++) {
    if (i % 32 == 0) Serial.print("    ");
    if (pkt[i] < 16) Serial.print("0");
    Serial.print(pkt[i], HEX);
    Serial.print((i % 32 == 31 || i == pktLen - 1) ? "\r\n" : " ");
  }
#endif

  digitalWrite(PIN_LED, HIGH);
  ledOn = true;
  ledOnSinceMs = millis();

  pktSeq++;
  batchIndex = 0;
}

// ============================================================
void printStatus() {
  Serial.print("[status] up "); Serial.print(millis() / 1000);
  Serial.print(" s  GPS "); Serial.print(gpsStateName(gpsState(gpsFixFresh())));
  Serial.print("  heard "); Serial.print(gpsSatsHeard());
  Serial.print("  used ");
  Serial.print(gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
  Serial.print(gpsFixFresh() ? "  FIX" : "  no fix");
  Serial.print("  time "); Serial.print(timeLabel());
  if (gps.date.isValid() && gps.date.year() >= 2024) {
    Serial.print("  date "); Serial.print(gps.date.year()); Serial.print("-");
    print2(gps.date.month()); Serial.print("-"); print2(gps.date.day());
  }
  if (gps.altitude.isValid()) { Serial.print("  alt "); Serial.print(gps.altitude.meters(), 1); Serial.print(" m"); }
  Serial.print("  pps "); Serial.print(ppsCount);
  Serial.print("  gps bytes "); Serial.print(gps.charsProcessed());
  Serial.print("  nmea ok/bad "); Serial.print(gps.passedChecksum());
  Serial.print("/"); Serial.print(gps.failedChecksum());
  Serial.print("  sht ok/err "); Serial.print(sensorOk);
  Serial.print("/"); Serial.println(sensorErr);
  if (millis() > 15000 && gps.charsProcessed() < 10)
    Serial.println("  ! no bytes from GPS - check PB6/PB7 wiring and GPS power");
  else if (millis() > 15000 && gps.passedChecksum() == 0)
    Serial.println("  ! GPS bytes but no valid NMEA - wrong baud rate or TX/RX swapped?");
}

// ============================================================
void loop() {
  while (Serial1.available()) {
    char c = Serial1.read();
#if ECHO_NMEA
    Serial.write(c);
#endif
    gps.encode(c);
    gpsStatusFeed(c);
  }

  updateTimeRef();

  if (millis() - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = millis();
    if (batchIndex < BATCH_SIZE) takeSample();
  }

  unsigned long slotStart = (unsigned long)(NODE_ID - 1) * SLOT_MS;
  unsigned long pos       = msIntoCycle();
  long          cycle     = cycleNumber();
  if (cycle != lastFiredCycle &&
      (millis() - lastFireMs) >= CYCLE_MS / 2 &&
      pos >= slotStart && pos < slotStart + SLOT_WINDOW_MS) {
    lastFiredCycle = cycle;
    lastFireMs     = millis();
    emitPacket();
  }

  if (ledOn && millis() - ledOnSinceMs >= LED_FLASH_MS) {
    digitalWrite(PIN_LED, LOW);
    ledOn = false;
  }

  if (millis() - lastStatus >= STATUS_EVERY_MS) {
    lastStatus = millis();
    printStatus();
  }
}
