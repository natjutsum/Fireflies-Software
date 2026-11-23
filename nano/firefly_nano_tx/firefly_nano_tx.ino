// ============================================================
// Fireflies Nano — LoRa Transmitter  [RAK3172 / STM32WL, RUI3]
// Port of Code/Fireflies_Latest_2026-10-03/Transmitter/firefly_tx_v2
// (the perfboard XIAO RP2040 + RFM95W firmware) to the Fireflies Nano PCB.
//
//   RAK3172 (STM32WLE5, built-in SX126x radio) + MAX-M10S GPS + SHT4x
//
//   *** SET NODE_ID BELOW — 1 ON THE FIRST BOARD, 2 ON THE SECOND ***
//
// Same radio settings, packet layout and GPS-slotted TDMA as the
// perfboard nodes, so Nano and perfboard nodes can share the channel.
// The flags byte also carries the GPS state (bits 2-5, gps_status.h):
// needs receiver.py from 21 Oct 2026 or later, which accepts those bits.
//
// Arduino IDE: board package "RAKwireless RUI STM32 Boards",
// board "WisDuo RAK3172 Evaluation Board". Library: TinyGPSPlus.
// ============================================================

#include <Wire.h>
#include <TinyGPSPlus.h>
#include "gps_status.h"     // GPS state bits for the flags byte (same folder)

// ============================================================
//  >>>>>>>>>>  CHANGE THIS PER BOARD  <<<<<<<<<<
// ============================================================
#define NODE_ID   3          // board 1 -> 1,  board 2 -> 2
// ============================================================

// ---------- Pin map (Fireflies Nano PCB, from the KiCad netlist) ----------
//   GPS UART   : RAK3172 PB6 (UART1_TX) -> GPS RXD,  PB7 (UART1_RX) <- GPS TXD  = Serial1
//   GPS PPS    : MAX-M10S TIMEPULSE -> RAK3172 PA0 (pin 29)
//   SHT4x I2C  : PA11 SDA / PA12 SCL, 4.7k pull-ups                           = Wire
//   LED        : PA9 -> 470R -> LED (active high)
//   Debug/flash: UART2 PA2/PA3 test pads                                      = Serial
#define PIN_LED   PA9
#define PIN_PPS   PA0

#define USE_PPS   1    // 1 = use the GPS timepulse for exact second edges.
                       // 0 = time from NMEA arrival only (same as the perfboard nodes).
#define LED_ON_TX 1    // flash the LED while transmitting (costs ~5 mA for 215 ms)

// ---------- Radio — MUST MATCH receiver.py ----------
#define LORA_FREQ_HZ     915000000
#define LORA_BW_KHZ      125
#define LORA_SF          7
#define LORA_CR_CODE     0        // RUI3 coding rate code: 0 = 4/5 (1 = 4/6, 2 = 4/7, 3 = 4/8)
#define LORA_PREAMBLE    8
// The SX1276 sync word 0x12 (private) is written as 0x1424 on the SX126x
// radio inside the STM32WL. They are the same on air.
#define LORA_SYNCWORD    0x1424
#define CONFIG_ID        "FF-915.0-SF7-BW125-CR45-SW12-PRE8-CRC"   // must match receiver.py

// TX power, 5-22 dBm (conducted). The RAK3172 draws roughly 45 mA at 14 dBm
// and 90-120 mA at 20-22 dBm while transmitting. A CR2450 coin cell cannot
// supply those peaks — see README before raising this on coin-cell power.
#define LORA_POWER_DBM   5

// ---------- Timing / TDMA (identical to the perfboard firmware) ----------
// CYCLE_MS is the ONLY timing value to edit. Sampling is derived from it so
// each packet holds exactly one cycle of readings (none dropped):
//   1000 ms -> 10 Hz sampling    2000 -> 5 Hz    5000 -> 2 Hz    10000 -> 1 Hz
// Use a value that divides 86 400 000 (ms per day) so slots stay aligned
// across UTC midnight: 1000, 2000, 2500, 4000, 5000, 10000 are all fine.
// All nodes on the channel must use the same CYCLE_MS.
#define CYCLE_MS           1000  // one transmit per node per cycle
#define BATCH_SIZE         10    // readings per packet (10 x 12 B = 120 B)
#define SAMPLE_INTERVAL_MS (CYCLE_MS / BATCH_SIZE)
#define SLOT_MS            500   // node N fires at (N-1) * SLOT_MS into each cycle
#define SLOT_WINDOW_MS     120   // how long the slot stays "open"
#define GPS_STALE_MS       5000  // no GPS time for this long -> free-run
#define TX_TIMEOUT_MS      3000  // no TX-done callback after this -> give up
#define STATUS_EVERY_MS    10000 // status line on Serial

// Airtime at SF7/BW125/CR4-5, 128 B = 215 ms.
// Node 1 fires at   0 ms, finishes ~215 ms.
// Node 2 fires at 500 ms, finishes ~715 ms.  285 ms of guard either side.

// ============================================================
// Packet format — same layout as firefly_tx_v2 (flags bits 2-5 added)
//
//   RadioHead header, 4 B — stripped by adafruit_rfm9x.receive():
//     [0] 0xFF   broadcast     [1] NODE_ID
//     [2] seq & 0xFF           [3] 0x00
//
//   Application payload — what the receiver parses:
//     [0]    uint8   node_id
//     [1..2] uint16  seq            little-endian
//     [3]    uint8   flags          bit0 = time synced to GPS
//                                   bit1 = GPS position fix valid
//                                   bit2 = GPS timepulse locked
//                                   bits3-5 = GPS state (see gps_status.h)
//                                     1 no data  2 searching  3 few sats
//                                     4 acquiring (4+ heard)  5 fix
//     [4..]  Reading x N            12 B each
//
//   Total on air = 4 + 4 + 10*12 = 128 B
// ============================================================
#pragma pack(1)
struct Reading {
  int16_t  tempC_x10;   // temperature x 10   (25.3 C -> 253)
  uint8_t  humidity;    // 0-100 %RH
  int32_t  lat_x1e6;    // latitude  x 1e6
  int32_t  lng_x1e6;    // longitude x 1e6
  uint8_t  gpsValid;    // 0 or 1
};                      // 12 bytes
#pragma pack()
static_assert(sizeof(struct Reading) == 12, "Reading must be 12 bytes to match receiver.py");

#define RH_HEADER_LEN   4
#define APP_HEADER_LEN  4
#define FLAG_TIME_SYNC  0x01
#define FLAG_GPS_FIX    0x02

// Sent when the SHT4x is missing or a read fails. They are outside the
// receiver's valid ranges, so it logs the row with a TEMP/RH warning
// instead of logging a fake 0.0 C / 0 %.
#define TEMP_SENTINEL   (-32768)
#define RH_SENTINEL     255

TinyGPSPlus gps;

struct Reading batch[BATCH_SIZE];
uint8_t  batchIndex = 0;
uint16_t txSeq      = 0;

// ---------- Time discipline ----------
// secondBoundaryMs estimates the millis() value at the last UTC second tick.
unsigned long secondBoundaryMs  = 0;
unsigned long lastGpsTimeMs     = 0;
bool          haveTimeRef       = false;
bool          lastAnchorUsedPps = false;
long          lastFiredCycle    = -1;
unsigned long anchorUtcSec      = 0;           // GPS UTC seconds-of-day at the last anchor
uint32_t      lastAnchorTimeVal = 0xFFFFFFFF;  // raw GPS time at the last anchor
unsigned long lastFireMs        = 0;           // millis() of the last slot fire

volatile unsigned long ppsMs    = 0;           // millis() at the last timepulse edge
volatile uint32_t      ppsCount = 0;

unsigned long lastSample = 0;
unsigned long lastStatus = 0;

// ---------- Transmit state ----------
volatile bool txDoneFlag = false;
bool          txBusy     = false;
unsigned long txStartMs  = 0;
uint32_t      txOk = 0, txErr = 0, txMissedSlot = 0;

// ---------- Sensor state ----------
bool     sensorFound = false;
uint32_t sensorErr   = 0;
float    lastTempC   = NAN, lastRh = NAN;

void onTxDone(void) { txDoneFlag = true; }
void onPps(void)    { ppsMs = millis(); ppsCount++; }

// ============================================================
// SHT4x driver (plain Wire, no extra library needed)
// ============================================================
// SHT40-A / SHT41-A / SHT45-A answer at 0x44, the -B parts at 0x45. Both are tried.
uint8_t shtAddr = 0x44;
#define SHT4X_ADDR        shtAddr
#define SHT4X_MEASURE_MED 0xF6   // medium precision, ~4.5 ms
#define SHT4X_SOFT_RESET  0x94
#define SHT4X_READ_SERIAL 0x89

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
  Wire.beginTransmission(SHT4X_ADDR);
  Wire.write(cmd);
  return Wire.endTransmission() == 0;
}

bool shtRead6(uint8_t *buf) {
  if (Wire.requestFrom((uint8_t)SHT4X_ADDR, (uint8_t)6) != 6) return false;
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
  return shtRead6(buf);
}

bool shtBegin() {
  return shtProbe(0x44) || shtProbe(0x45);
}

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
// Stop here with a fast LED blink so a fault is visible without a laptop.
// ============================================================
void halt(const char *why) {
  Serial.print("HALT: "); Serial.println(why);
  while (true) {
    digitalWrite(PIN_LED, HIGH); delay(100);
    digitalWrite(PIN_LED, LOW);  delay(100);
  }
}

bool checkSet(const char *name, bool ok) {
  if (!ok) { Serial.print("  radio setting FAILED: "); Serial.println(name); }
  return ok;
}

// ============================================================
void setup() {
  Serial.begin(115200);       // stays in AT mode so Arduino IDE can still upload
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  delay(2000);
  digitalWrite(PIN_LED, LOW);

  Serial.println("========================================");
  Serial.print ("Fireflies Nano TX [RAK3172]  -  NODE "); Serial.println(NODE_ID);
  Serial.print ("SF"); Serial.print(LORA_SF);
  Serial.print ("  BW"); Serial.print(LORA_BW_KHZ);
  Serial.print ("  CR4/5  "); Serial.print(LORA_POWER_DBM); Serial.println(" dBm");
  Serial.print ("Config "); Serial.println(CONFIG_ID);
  Serial.print ("TDMA slot "); Serial.print((NODE_ID - 1) * SLOT_MS);
  Serial.print (" ms of every "); Serial.print(CYCLE_MS); Serial.println(" ms");
  Serial.println("========================================");

  // ---- Radio: LoRa P2P (not LoRaWAN) ----
  // A new RAK3172 boots in LoRaWAN mode. Switching mode needs a reboot,
  // after which this check passes.
  if (api.lora.nwm.get() != 0) {
    Serial.println("Switching RAK3172 to LoRa P2P mode, rebooting...");
    api.lora.nwm.set();
    delay(200);
    api.system.reboot();
  }

  bool ok = true;
  ok &= checkSet("frequency",   api.lora.pfreq.set(LORA_FREQ_HZ));
  ok &= checkSet("SF",          api.lora.psf.set(LORA_SF));
  ok &= checkSet("bandwidth",   api.lora.pbw.set(LORA_BW_KHZ));
  ok &= checkSet("coding rate", api.lora.pcr.set(LORA_CR_CODE));
  ok &= checkSet("preamble",    api.lora.ppl.set(LORA_PREAMBLE));
  ok &= checkSet("TX power",    api.lora.ptp.set(LORA_POWER_DBM));
  ok &= checkSet("sync word",   api.lora.syncword.set(LORA_SYNCWORD));
  ok &= checkSet("IQ normal",   api.lora.iqInver.set(false));
  ok &= checkSet("no encrypt",  api.lora.encry.set(false));
  if (!ok) halt("radio settings rejected - check LORA_* values");

  api.lora.registerPSendCallback(onTxDone);
  api.lora.precv(0);          // radio out of receive, ready to transmit

  Serial.print("Radio read-back: ");
  Serial.print(api.lora.pfreq.get());  Serial.print(" Hz  SF");
  Serial.print(api.lora.psf.get());    Serial.print("  BW");
  uint32_t bw = api.lora.pbw.get();    // returns a code: 0 = 125, 1 = 250, 2 = 500 kHz
  Serial.print(bw == 0 ? 125 : bw == 1 ? 250 : bw == 2 ? 500 : bw); Serial.print("  CRcode ");
  Serial.print(api.lora.pcr.get());    Serial.print("  pre ");
  Serial.print(api.lora.ppl.get());    Serial.print("  ");
  Serial.print(api.lora.ptp.get());    Serial.print(" dBm  sync 0x");
  Serial.println(api.lora.syncword.get(), HEX);

  // ---- GPS: MAX-M10S on UART1, factory default 9600 baud NMEA ----
  // RAK_CUSTOM_MODE stops RUI3 treating the GPS's NMEA as AT commands.
  Serial1.begin(9600, RAK_CUSTOM_MODE);
  Serial.println("GPS: Serial1 (PB6/PB7) @ 9600");

#if USE_PPS
  pinMode(PIN_PPS, INPUT);
  attachInterrupt(PIN_PPS, onPps, RISING);
  Serial.println("GPS timepulse: PA0 (starts after the GPS has a time fix)");
#endif

  // ---- SHT4x ----
  Wire.begin();
  sensorFound = shtBegin();
  if (sensorFound) { Serial.print("SHT4x: ready at 0x"); Serial.println(shtAddr, HEX); }
  else Serial.println("SHT4x: NOT FOUND - sending sentinel values");

  Serial.println("Waiting for GPS time to align TDMA slot...\n");
}

// ============================================================
// Update the second-boundary reference from GPS time.
// gps.time.age() is how long ago that time was parsed, so
// millis() - age estimates when the sentence arrived. If a timepulse
// edge came just before it, that edge is the exact UTC second.
// ============================================================
void updateTimeRef() {
  // Before it has satellites the GPS sends an EMPTY time field, which
  // TinyGPSPlus still reports as valid and fresh (00:00:00, or frozen at
  // the last real time). Only anchor when the date is real and the time
  // has actually moved on; otherwise keep extrapolating the last anchor.
  if (!gps.time.isValid() || gps.time.age() >= 200) return;   // need a fresh sentence
  if (!gps.date.isValid() || gps.date.year() < 2024) return;  // no real date yet
  uint32_t t = gps.time.value();                              // hhmmsscc
  if (t == lastAnchorTimeVal) return;                         // time not advancing
  lastAnchorTimeVal = t;

  unsigned long nmeaMs   = millis() - gps.time.age();
  unsigned long boundary = nmeaMs;
  lastAnchorUsedPps = false;

#if USE_PPS
  noInterrupts();
  unsigned long p  = ppsMs;
  uint32_t      pc = ppsCount;
  interrupts();
  // The NMEA for second S always follows the pulse for second S.
  if (pc > 0 && (long)(nmeaMs - p) >= 0 && (nmeaMs - p) < 1000) {
    boundary = p;
    lastAnchorUsedPps = true;
  }
#endif

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

// Slot timing uses absolute UTC time-of-day, so it keeps counting up and
// all nodes line up. Free-runs from the last anchor if GPS is lost.
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

// ============================================================
void takeSample() {
  struct Reading &r = batch[batchIndex];

  float tempC, rh;
  if (sensorFound && shtMeasure(tempC, rh)) {
    lastTempC   = tempC;
    lastRh      = rh;
    r.tempC_x10 = (int16_t)lroundf(tempC * 10.0f);
    r.humidity  = (uint8_t)lroundf(rh);
  } else {
    if (sensorFound) sensorErr++;
    r.tempC_x10 = TEMP_SENTINEL;
    r.humidity  = RH_SENTINEL;
  }

  if (gpsFixFresh()) {
    r.lat_x1e6 = (int32_t)lround(gps.location.lat() * 1e6);
    r.lng_x1e6 = (int32_t)lround(gps.location.lng() * 1e6);
    r.gpsValid = 1;
  } else {
    r.lat_x1e6 = 0;
    r.lng_x1e6 = 0;
    r.gpsValid = 0;
  }
  batchIndex++;
}

// ============================================================
void startTransmitBatch() {
  if (batchIndex == 0) return;
  if (txBusy) { txMissedSlot++; return; }

  uint8_t pkt[RH_HEADER_LEN + APP_HEADER_LEN + BATCH_SIZE * sizeof(struct Reading)];

  pkt[0] = 0xFF;                       // RH destination: broadcast
  pkt[1] = (uint8_t)NODE_ID;           // RH source
  pkt[2] = (uint8_t)(txSeq & 0xFF);    // RH identifier
  pkt[3] = 0x00;                       // RH flags

  uint8_t flags = 0;
  if (timeIsSynced()) flags |= FLAG_TIME_SYNC;
  if (gpsFixFresh())  flags |= FLAG_GPS_FIX;
  noInterrupts();
  unsigned long pMs = ppsMs;
  uint32_t      pCt = ppsCount;
  interrupts();
  flags |= gpsStatusFlags(gpsFixFresh(), pCt, pMs);

  pkt[4] = (uint8_t)NODE_ID;
  pkt[5] = (uint8_t)(txSeq & 0xFF);
  pkt[6] = (uint8_t)((txSeq >> 8) & 0xFF);
  pkt[7] = flags;

  memcpy(pkt + RH_HEADER_LEN + APP_HEADER_LEN,
         batch, batchIndex * sizeof(struct Reading));

  uint8_t pktLen = RH_HEADER_LEN + APP_HEADER_LEN
                 + batchIndex * sizeof(struct Reading);

  txDoneFlag = false;
  if (api.lora.psend(pktLen, pkt)) {
    txBusy    = true;
    txStartMs = millis();
#if LED_ON_TX
    digitalWrite(PIN_LED, HIGH);
#endif
    Serial.print("TX seq "); Serial.print(txSeq);
    Serial.print("  slot "); Serial.print(msIntoCycle());
    Serial.print(" ms  "); Serial.print(pktLen);
    Serial.print(" B  flags 0x"); Serial.print(flags, HEX);
    Serial.print("  GPS "); Serial.print(gpsStateName((flags & GPS_STATE_MASK) >> GPS_STATE_SHIFT));
    Serial.print(", "); Serial.print(gpsSatsHeard()); Serial.print(" heard");
    Serial.print((flags & FLAG_PPS_LOCK) ? ", PPS  " : "  ");
    if (!timeIsSynced())       Serial.println("[free]");
    else if (lastAnchorUsedPps) Serial.println("[GPS+PPS]");
    else                        Serial.println("[GPS]");
  } else {
    txErr++;
    Serial.println("psend failed (radio busy or not in P2P mode)");
  }

  txSeq++;
  batchIndex = 0;
}

// ============================================================
void serviceTransmit() {
  if (!txBusy) return;

  if (txDoneFlag) {
    txDoneFlag = false;
    txBusy = false;
    txOk++;
    digitalWrite(PIN_LED, LOW);
    Serial.print("  TxDone "); Serial.print(millis() - txStartMs);
    Serial.print(" ms   ok="); Serial.print(txOk);
    Serial.print(" err="); Serial.print(txErr);
    Serial.print(" missed="); Serial.println(txMissedSlot);
    return;
  }

  if (millis() - txStartMs > TX_TIMEOUT_MS) {
    Serial.println("  TX TIMEOUT - no TX-done callback");
    txBusy = false;
    txErr++;
    digitalWrite(PIN_LED, LOW);
    api.lora.precv(0);
  }
}

// ============================================================
void printStatus() {
  Serial.print("[status] GPS "); Serial.print(gpsStateName(gpsState(gpsFixFresh())));
  Serial.print("  heard "); Serial.print(gpsSatsHeard());
  Serial.print("  used ");
  Serial.print(gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
  Serial.print(gpsFixFresh() ? "  fix " : "  no fix");
  if (gpsFixFresh()) {
    Serial.print(gps.location.lat(), 6); Serial.print(",");
    Serial.print(gps.location.lng(), 6);
  }
  Serial.print(timeIsSynced() ? "  time OK" : "  time free-run");
  Serial.print("  pps "); Serial.print(ppsCount);
  Serial.print("  nmea "); Serial.print(gps.passedChecksum());
  Serial.print("/bad "); Serial.print(gps.failedChecksum());
  Serial.print("  T "); Serial.print(lastTempC, 1);
  Serial.print(" C  RH "); Serial.print(lastRh, 0);
  Serial.print(" %  sht_err "); Serial.println(sensorErr);
  if (millis() > 15000 && gps.charsProcessed() < 10)
    Serial.println("  ! no bytes from GPS - check PB6/PB7 wiring and GPS power");
}

// ============================================================
void loop() {
  while (Serial1.available()) {
    char c = Serial1.read();
    gps.encode(c);
    gpsStatusFeed(c);
  }

  updateTimeRef();
  serviceTransmit();

  if (millis() - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = millis();
    if (batchIndex < BATCH_SIZE) takeSample();
  }

  // ---- TDMA slot ----
  // Fire once per cycle, inside this node's window.
  unsigned long slotStart = (unsigned long)(NODE_ID - 1) * SLOT_MS;
  unsigned long pos       = msIntoCycle();
  long          cycle     = cycleNumber();

  // The half-cycle hold-off stops a double fire at the moment the clock
  // switches from free-run to GPS time (the cycle numbering changes then).
  if (cycle != lastFiredCycle &&
      (millis() - lastFireMs) >= CYCLE_MS / 2 &&
      pos >= slotStart && pos < slotStart + SLOT_WINDOW_MS) {
    lastFiredCycle = cycle;
    lastFireMs     = millis();
    startTransmitBatch();
  }

  if (millis() - lastStatus >= STATUS_EVERY_MS) {
    lastStatus = millis();
    printStatus();
  }
}
