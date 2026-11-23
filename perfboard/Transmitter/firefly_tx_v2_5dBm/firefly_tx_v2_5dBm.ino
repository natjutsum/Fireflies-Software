// ============================================================
// Fireflies — LoRa Transmitter v2  [multi-node, GPS-slotted TDMA]
// POWER VARIANT: 5 dBm (LOW - bench/short-range testing)  —  OCP 60 mA
// Built from Claude outputs/firefly_tx_v2.ino (17 Sep 2026); only the
// TX power / current-limit lines differ between the three variants.
// XIAO RP2040 + RFM95W + SHT41 (I2C) + u-blox GPS (UART)
//
//   *** SET NODE_ID BELOW — 1 ON THE FIRST BOARD, 2 ON THE SECOND ***
//
// Radio config MUST MATCH receiver.py exactly.
// SF7 / BW125 / CR4-5, 128 B packet = 215 ms airtime.
// Two nodes at 1 Hz = 43% channel load. 500 ms slots, 285 ms guard.
// ============================================================

#include <RadioLib.h>
#include <Adafruit_SHT4x.h>
#include <Wire.h>
#include <TinyGPSPlus.h>

// ============================================================
//  >>>>>>>>>>  CHANGE THIS PER BOARD  <<<<<<<<<<
// ============================================================
#define NODE_ID   1          // board 1 -> 1,  board 2 -> 2
// ============================================================

// ---------- Pin map (XIAO RP2040) ----------
#define PIN_SCK    2   // D8
#define PIN_MOSI   3   // D10
#define PIN_MISO   4   // D9
#define PIN_NSS   27   // D1
#define PIN_RESET 26   // D0
#define PIN_DIO0  28   // D2

// ---------- Radio — MUST MATCH receiver.py ----------
#define LORA_FREQ_MHZ   915.0
#define LORA_BW_KHZ     125.0
#define LORA_SF         7
#define LORA_CR         5
#define LORA_SYNCWORD   0x12
#define LORA_PREAMBLE   8
// LDRO: RadioLib sets it automatically — off at SF7/BW125. CRC on. Explicit header.
#define CONFIG_ID       "FF-915.0-SF7-BW125-CR45-SW12-PRE8-CRC"   // must match receiver.py
#define LORA_POWER_DBM  5     // PA_BOOST. Valid: 2-17, or 20. (18, 19, 21+ are rejected)
#define LORA_OCP_MA     60    // over-current limit; RadioLib's begin() default is 60 mA,
                               // below the ~90 mA (17 dBm) / ~120 mA (20 dBm) PA_BOOST draw

// ---------- Timing / TDMA ----------
// CYCLE_MS is the ONLY timing value to edit. Sampling is derived from it so
// each packet holds exactly one cycle of readings (none dropped):
//   1000 ms -> 10 Hz sampling    2000 -> 5 Hz    5000 -> 2 Hz    10000 -> 1 Hz
// Use a value that divides 86 400 000 (ms per day) so slots stay aligned
// across UTC midnight: 1000, 2000, 2500, 4000, 5000, 10000 are all fine.
// Record any change in the test spreadsheet (default in RADIO_SETTINGS = 1000).
#define CYCLE_MS           1000  // one transmit per node per cycle
#define BATCH_SIZE         10    // readings per packet (10 x 12 B = 120 B; max ~20 fits 255 B)
#define SAMPLE_INTERVAL_MS (CYCLE_MS / BATCH_SIZE)
#define SLOT_MS            500   // node N fires at (N-1) * SLOT_MS into each cycle
#define SLOT_WINDOW_MS     120   // how long the slot stays "open"
#define GPS_STALE_MS       5000  // no GPS time for this long -> free-run

// Airtime at SF7/BW125/CR4-5, 128 B = 215 ms.
// Node 1 fires at   0 ms, finishes ~215 ms.
// Node 2 fires at 500 ms, finishes ~715 ms.  285 ms of guard either side.

SX1276 radio = new Module(PIN_NSS, PIN_DIO0, PIN_RESET, RADIOLIB_NC);
TinyGPSPlus gps;
Adafruit_SHT4x sht4;
bool sensorFound = false;

// ============================================================
// Packet format
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

#define RH_HEADER_LEN   4
#define APP_HEADER_LEN  4
#define FLAG_TIME_SYNC  0x01
#define FLAG_GPS_FIX    0x02

struct Reading batch[BATCH_SIZE];
uint8_t  batchIndex = 0;
uint16_t txSeq      = 0;

// ---------- Time discipline ----------
// secondBoundaryMs estimates the millis() value at the last UTC second tick.
unsigned long secondBoundaryMs = 0;
unsigned long lastGpsTimeMs    = 0;
bool          haveTimeRef      = false;
long          lastFiredCycle   = -1;
unsigned long anchorUtcSec     = 0;   // GPS UTC seconds-of-day at the last anchor
uint32_t      lastAnchorTimeVal = 0xFFFFFFFF;  // raw GPS time at the last anchor
unsigned long lastFireMs       = 0;   // millis() of the last slot fire
unsigned long utcMsOfDay();           // defined below

unsigned long lastSample = 0;

// ---------- Transmit state ----------
volatile bool txDoneFlag = false;
bool          txBusy     = false;
unsigned long txStartMs  = 0;
uint32_t      txOk = 0, txErr = 0, txMissedSlot = 0;

void onTxDone() { txDoneFlag = true; }

// ============================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println(F("========================================"));
  Serial.print (F("Fireflies TX v2 [5 dBm LOW - bench/short-range testing]  —  NODE ")); Serial.println(NODE_ID);
  Serial.print (F("SF")); Serial.print(LORA_SF);
  Serial.print (F("  BW")); Serial.print(LORA_BW_KHZ);
  Serial.print (F("  CR4/")); Serial.print(LORA_CR);
  Serial.print (F("  ")); Serial.print(LORA_POWER_DBM); Serial.println(F(" dBm"));
  Serial.print (F("Config ")); Serial.println(F(CONFIG_ID));
  Serial.print (F("TDMA slot ")); Serial.print((NODE_ID - 1) * SLOT_MS);
  Serial.print (F(" ms of every ")); Serial.print(CYCLE_MS); Serial.println(F(" ms"));
  Serial.println(F("========================================"));

  Serial1.begin(9600);
  Serial.println(F("GPS: Serial1 @ 9600"));

  Wire.begin();
  if (!sht4.begin(&Wire)) {
    Serial.println(F("SHT41: NOT FOUND — continuing without it"));
    sensorFound = false;
  } else {
    sht4.setPrecision(SHT4X_MED_PRECISION);
    sht4.setHeater(SHT4X_NO_HEATER);
    Serial.println(F("SHT41: ready"));
    sensorFound = true;
  }

  SPI.setSCK(PIN_SCK);
  SPI.setTX(PIN_MOSI);
  SPI.setRX(PIN_MISO);
  SPI.begin();

  Serial.print(F("Radio init... "));
  int state = radio.begin(LORA_FREQ_MHZ, LORA_BW_KHZ, LORA_SF, LORA_CR,
                          LORA_SYNCWORD, LORA_POWER_DBM, LORA_PREAMBLE);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("FAILED, code ")); Serial.println(state);
    Serial.println(F("Check NSS/RESET/DIO0 wiring and 3V3. Halting."));
    while (true) { delay(1000); }
  }
  Serial.println(F("OK"));

  // PA_BOOST — the RFM95W's ANT pin is wired to it, not RFO.
  state = radio.setOutputPower(LORA_POWER_DBM, false);   // false = PA_BOOST
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("setOutputPower failed, code ")); Serial.println(state);
    Serial.println(F("Power must be 2-17 or 20 dBm. Halting."));
    while (true) { delay(1000); }
  }
  state = radio.setCurrentLimit(LORA_OCP_MA);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("setCurrentLimit failed, code ")); Serial.println(state);
  }
  Serial.print(F("TX power ")); Serial.print(LORA_POWER_DBM);
  Serial.print(F(" dBm, OCP ")); Serial.print(LORA_OCP_MA); Serial.println(F(" mA"));
  radio.setCRC(true);
  radio.setPacketSentAction(onTxDone);

  Serial.println(F("Waiting for GPS time to align TDMA slot...\n"));
}

// ============================================================
// Update the second-boundary reference from GPS time.
// gps.time.age() is how long ago that time was parsed, so
// millis() - age estimates when the UTC second actually ticked.
// ============================================================
void updateTimeRef() {
  // FIX 3 Oct 2026 (2): before it has satellites the GPS sends RMC/GGA with an
  // EMPTY time field. TinyGPSPlus still marks time valid + fresh each sentence,
  // at 00:00:00 or frozen at the last real time. Anchoring to that froze the
  // slot clock and the node went silent after a restart. Now we only anchor
  // when (a) the date is real and (b) the GPS time has actually moved on.
  // Otherwise we keep extrapolating from the last good anchor (or free-run).
  if (!gps.time.isValid() || gps.time.age() >= 200) return;   // need a fresh sentence
  if (!gps.date.isValid() || gps.date.year() < 2024) return;  // no real date yet
  uint32_t t = gps.time.value();                              // hhmmsscc
  if (t == lastAnchorTimeVal) return;                         // time not advancing
  lastAnchorTimeVal = t;

  secondBoundaryMs = millis() - gps.time.age();
  anchorUtcSec     = gps.time.hour() * 3600UL
                   + gps.time.minute() * 60UL
                   + gps.time.second();
  lastGpsTimeMs    = millis();
  haveTimeRef      = true;
}

bool timeIsSynced() {
  return haveTimeRef && (millis() - lastGpsTimeMs) < GPS_STALE_MS;
}

// Milliseconds into the current cycle. Free-runs from the last
// GPS anchor if the fix is lost — drift is ~2 ms per 2 minutes.
unsigned long msIntoCycle() {
  if (!haveTimeRef) return millis() % CYCLE_MS;
  return utcMsOfDay() % CYCLE_MS;
}

// FIX 3 Oct 2026: slot timing is taken from absolute UTC time-of-day, so it
// keeps counting up and works for any CYCLE_MS. (Previously the anchor was
// refreshed every GPS sentence, the cycle number was always 0, and the node
// sent once then went silent.) All nodes share UTC, so slots line up.
unsigned long utcMsOfDay() {
  return anchorUtcSec * 1000UL + (millis() - secondBoundaryMs);
}

long cycleNumber() {
  if (!haveTimeRef) return (long)(millis() / CYCLE_MS);
  return (long)(utcMsOfDay() / CYCLE_MS);
}

// ============================================================
void takeSample() {
  float  tempC = 0.0f, humidity = 0.0f;
  double lat = 0.0, lng = 0.0;
  bool   gpsValid = false;

  if (sensorFound) {
    sensors_event_t humEvent, tempEvent;
    sht4.getEvent(&humEvent, &tempEvent);
    tempC    = tempEvent.temperature;
    humidity = humEvent.relative_humidity;
  }

  if (gps.location.isValid() && gps.location.age() < 5000) {
    lat = gps.location.lat();
    lng = gps.location.lng();
    gpsValid = true;
  }

  batch[batchIndex].tempC_x10 = (int16_t)(tempC * 10.0f);
  batch[batchIndex].humidity  = (uint8_t)constrain(humidity, 0, 100);
  batch[batchIndex].lat_x1e6  = (int32_t)(lat * 1e6);
  batch[batchIndex].lng_x1e6  = (int32_t)(lng * 1e6);
  batch[batchIndex].gpsValid  = gpsValid ? 1 : 0;
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
  if (timeIsSynced())                       flags |= FLAG_TIME_SYNC;
  if (gps.location.isValid() &&
      gps.location.age() < 5000)            flags |= FLAG_GPS_FIX;

  pkt[4] = (uint8_t)NODE_ID;
  pkt[5] = (uint8_t)(txSeq & 0xFF);
  pkt[6] = (uint8_t)((txSeq >> 8) & 0xFF);
  pkt[7] = flags;

  memcpy(pkt + RH_HEADER_LEN + APP_HEADER_LEN,
         batch, batchIndex * sizeof(struct Reading));

  size_t pktLen = RH_HEADER_LEN + APP_HEADER_LEN
                + batchIndex * sizeof(struct Reading);

  int state = radio.startTransmit(pkt, pktLen);
  if (state == RADIOLIB_ERR_NONE) {
    txBusy     = true;
    txDoneFlag = false;
    txStartMs  = millis();
    Serial.print(F("TX seq ")); Serial.print(txSeq);
    Serial.print(F("  slot ")); Serial.print(msIntoCycle());
    Serial.print(F(" ms  ")); Serial.print(pktLen);
    Serial.print(F(" B  ")); Serial.print(timeIsSynced() ? F("[GPS]") : F("[free]"));
    Serial.println();
  } else {
    txErr++;
    Serial.print(F("startTransmit failed, code ")); Serial.println(state);
  }

  txSeq++;
  batchIndex = 0;
}

// ============================================================
void serviceTransmit() {
  if (!txBusy) return;

  if (txDoneFlag) {
    txDoneFlag = false;
    radio.finishTransmit();
    txBusy = false;
    txOk++;
    Serial.print(F("  TxDone ")); Serial.print(millis() - txStartMs);
    Serial.print(F(" ms   ok=")); Serial.print(txOk);
    Serial.print(F(" err=")); Serial.print(txErr);
    Serial.print(F(" missed=")); Serial.println(txMissedSlot);
    return;
  }

  if (millis() - txStartMs > 3000) {
    Serial.println(F("  TX TIMEOUT — no TxDone. Check DIO0 (D2/GP28)."));
    radio.finishTransmit();
    txBusy = false;
    txErr++;
  }
}

// ============================================================
void loop() {
  while (Serial1.available()) gps.encode(Serial1.read());

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
}
