// ============================================================
// Fireflies Nano — GPS DIAGNOSTIC  [RAK3172 / RUI3 + MAX-M10S]
//
//   Never uses the LoRa radio — safe with no LoRa antenna.
//
// Every 2 s prints to the Serial Monitor (115200):
//   - every satellite the GPS reports, with its signal strength (dB-Hz) as a bar
//   - fix status, UTC time, position
//   - both serial directions: GPS -> RAK (NMEA arriving) and
//     RAK -> GPS (the GPS answers our UBX commands)
//   - GPS model / firmware (UBX-MON-VER) and RF front-end readings (UBX-MON-RF)
//   - a one-line hint on what to check next
//
// Signal guide (dB-Hz):  < 20 too weak   20-30 weak   30-40 good   > 40 excellent
// A fix normally needs 4+ satellites at about 25 dB-Hz or better.
//
// Arduino IDE: board "WisDuo RAK3172 Evaluation Board". Library: TinyGPSPlus.
// ============================================================

#include <TinyGPSPlus.h>

#define PIN_LED           PA9
#define PIN_PPS           PA0
#define GPS_BAUD          9600       // MAX-M10S factory default
#define REPORT_EVERY_MS   2000
#define RF_POLL_EVERY_MS  5000
#define ECHO_NMEA         0          // 1 = also print every raw NMEA sentence
#define STALE_SAT_MS      5000       // drop a satellite not reported for this long

TinyGPSPlus gps;

// ---------- Satellite table (filled from $xxGSV sentences) ----------
struct Sat {
  uint16_t talker;      // two NMEA talker letters, e.g. 'G','P'
  uint16_t prn;
  int16_t  elev;        // degrees, -1 unknown
  int16_t  az;          // degrees, -1 unknown
  int16_t  cno;         // dB-Hz, -1 = not heard
  unsigned long seenMs;
};
#define MAX_SATS 64
Sat sats[MAX_SATS];
uint8_t nSats = 0;

// ---------- Counters ----------
uint32_t nmeaOk = 0, nmeaBad = 0, gpsBytes = 0;
volatile uint32_t ppsCount = 0;
void onPps(void) { ppsCount++; }

// ---------- UBX replies ----------
bool     verReceived = false;
char     verSw[31] = "", verHw[11] = "", verMod[31] = "", verProt[31] = "";
bool     rfReceived = false;
unsigned long rfLastMs = 0;
uint8_t  rfAntStatus = 255, rfAntPower = 255, rfJamState = 0, rfCwSupp = 0;
uint16_t rfNoise = 0, rfAgc = 0;
uint32_t ubxReplies = 0;

unsigned long lastReport = 0, lastRfPoll = 0, lastVerPoll = 0;

// ============================================================
// Helpers
// ============================================================
const char *sysName(uint16_t t) {
  char a = t >> 8, b = t & 0xFF;
  if (a == 'G' && b == 'P') return "GPS ";
  if (a == 'G' && b == 'L') return "GLO ";
  if (a == 'G' && b == 'A') return "GAL ";
  if (a == 'G' && b == 'B') return "BDS ";
  if (a == 'B' && b == 'D') return "BDS ";
  if (a == 'G' && b == 'Q') return "QZSS";
  if (a == 'G' && b == 'I') return "NAVIC";
  if (a == 'G' && b == 'N') return "MULTI";
  return "??? ";
}

void printPadded(long v, uint8_t width) {
  char buf[12];
  snprintf(buf, sizeof(buf), "%*ld", width, v);
  Serial.print(buf);
}

void print2(int v) { if (v < 10) Serial.print("0"); Serial.print(v); }

// ============================================================
// UBX (u-blox binary) — send polls, parse replies
// ============================================================
void ubxPoll(uint8_t cls, uint8_t id) {
  uint8_t m[8] = {0xB5, 0x62, cls, id, 0, 0, 0, 0};
  uint8_t a = 0, b = 0;
  for (uint8_t i = 2; i < 6; i++) { a += m[i]; b += a; }
  m[6] = a; m[7] = b;
  for (uint8_t i = 0; i < 8; i++) Serial1.write(m[i]);
}

void copyField(char *dst, const uint8_t *src, uint8_t maxLen) {
  uint8_t i = 0;
  for (; i < maxLen && src[i]; i++) dst[i] = (char)src[i];
  dst[i] = 0;
}

void handleUbx(uint8_t cls, uint8_t id, const uint8_t *p, uint16_t len) {
  ubxReplies++;
  if (cls == 0x0A && id == 0x04 && len >= 40) {          // MON-VER
    verReceived = true;
    copyField(verSw, p, 30);
    copyField(verHw, p + 30, 10);
    for (uint16_t off = 40; off + 30 <= len; off += 30) {
      char ext[31];
      copyField(ext, p + off, 30);
      if (strncmp(ext, "MOD=", 4) == 0)     strncpy(verMod, ext + 4, 30);
      if (strncmp(ext, "PROTVER=", 8) == 0) strncpy(verProt, ext + 8, 30);
    }
  } else if (cls == 0x0A && id == 0x38 && len >= 4 + 24) { // MON-RF, first block
    const uint8_t *blk = p + 4;
    rfReceived  = true;
    rfLastMs    = millis();
    rfJamState  = blk[1] & 0x03;
    rfAntStatus = blk[2];
    rfAntPower  = blk[3];
    rfNoise     = blk[12] | (blk[13] << 8);
    rfAgc       = blk[14] | (blk[15] << 8);
    rfCwSupp    = blk[16];
  }
}

// Returns true if the byte belonged to a UBX frame.
uint8_t  ubxState = 0, ubxCls, ubxId, ubxCkA, ubxCkB;
uint16_t ubxLen, ubxPos;
uint8_t  ubxBuf[256];

bool ubxByte(uint8_t c) {
  switch (ubxState) {
    case 0: if (c == 0xB5) { ubxState = 1; return true; } return false;
    case 1: if (c == 0x62) { ubxState = 2; ubxCkA = ubxCkB = 0; return true; }
            ubxState = 0; return false;
    default: break;
  }
  if (ubxState >= 2 && ubxState <= 6) { ubxCkA += c; ubxCkB += ubxCkA; }
  switch (ubxState) {
    case 2: ubxCls = c; ubxState = 3; break;
    case 3: ubxId  = c; ubxState = 4; break;
    case 4: ubxLen = c; ubxState = 5; break;
    case 5: ubxLen |= (uint16_t)c << 8; ubxPos = 0;
            if (ubxLen > sizeof(ubxBuf)) ubxState = 0;      // too big, drop
            else ubxState = ubxLen ? 6 : 7;
            break;
    case 6: ubxBuf[ubxPos++] = c; if (ubxPos >= ubxLen) ubxState = 7; break;
    case 7: ubxState = (c == ubxCkA) ? 8 : 0; break;
    case 8: if (c == ubxCkB) handleUbx(ubxCls, ubxId, ubxBuf, ubxLen);
            ubxState = 0; break;
  }
  return true;
}

// ============================================================
// NMEA — checksum check, then GSV satellites and TXT messages
// ============================================================
void removeTalker(uint16_t talker) {
  uint8_t j = 0;
  for (uint8_t i = 0; i < nSats; i++)
    if (sats[i].talker != talker) sats[j++] = sats[i];
  nSats = j;
}

int fieldInt(const char *f, int dflt) { return (f && *f) ? atoi(f) : dflt; }

void handleGsv(uint16_t talker, char **f, uint8_t n) {
  if (n < 4) return;
  int msgNum = fieldInt(f[2], 0);
  if (msgNum == 1) removeTalker(talker);    // start of a new GSV cycle for this system
  // satellites come in groups of 4 fields: PRN, elevation, azimuth, C/N0
  for (uint8_t g = 4; g + 3 < n; g += 4) {
    if (!*f[g]) continue;
    if (nSats >= MAX_SATS) return;
    Sat &s   = sats[nSats++];
    s.talker = talker;
    s.prn    = fieldInt(f[g], 0);
    s.elev   = fieldInt(f[g + 1], -1);
    s.az     = fieldInt(f[g + 2], -1);
    s.cno    = fieldInt(f[g + 3], -1);
    s.seenMs = millis();
  }
}

void processNmea(char *line) {
  // line = "$GPGSV,....*hh" (no CR/LF)
  char *star = strrchr(line, '*');
  if (!star || star - line < 6) { nmeaBad++; return; }
  uint8_t cs = 0;
  for (char *p = line + 1; p < star; p++) cs ^= (uint8_t)*p;
  if (strtol(star + 1, NULL, 16) != cs) { nmeaBad++; return; }
  nmeaOk++;

#if ECHO_NMEA
  Serial.println(line);
#endif

  uint16_t talker = ((uint16_t)line[1] << 8) | (uint8_t)line[2];
  const char *type = line + 3;

  if (strncmp(type, "TXT", 3) == 0) {          // u-blox status text — always worth showing
    Serial.print("[GPS says] "); Serial.println(line);
    return;
  }
  if (strncmp(type, "GSV", 3) != 0) return;

  *star = 0;
  char *f[24];
  uint8_t n = 0;
  char *p = line;
  f[n++] = p;
  while (*p && n < 24) {
    if (*p == ',') { *p = 0; f[n++] = p + 1; }
    p++;
  }
  handleGsv(talker, f, n);
}

char    nmeaLine[100];
uint8_t nmeaLen = 0;
bool    inNmea = false;

void nmeaChar(char c) {
  if (c == '$') { inNmea = true; nmeaLen = 0; }
  if (!inNmea) return;
  if (c == '\r' || c == '\n') {
    nmeaLine[nmeaLen] = 0;
    inNmea = false;
    if (nmeaLen > 6) processNmea(nmeaLine);
    return;
  }
  if (nmeaLen < sizeof(nmeaLine) - 1) nmeaLine[nmeaLen++] = c;
  else inNmea = false;                         // too long, not NMEA
}

// ============================================================
// Report
// ============================================================
const char *antStatusName(uint8_t s) {
  switch (s) { case 0: return "INIT"; case 1: return "unknown (no antenna sensing on this board)";
               case 2: return "OK"; case 3: return "SHORT"; case 4: return "OPEN"; }
  return "-";
}

void report() {
  unsigned long now = millis();

  // drop satellites that have not been reported recently
  uint8_t j = 0;
  for (uint8_t i = 0; i < nSats; i++)
    if (now - sats[i].seenMs < STALE_SAT_MS) sats[j++] = sats[i];
  nSats = j;

  // sort strongest first
  uint8_t idx[MAX_SATS];
  for (uint8_t i = 0; i < nSats; i++) idx[i] = i;
  for (uint8_t i = 1; i < nSats; i++) {
    uint8_t k = idx[i]; int8_t m = i - 1;
    while (m >= 0 && sats[idx[m]].cno < sats[k].cno) { idx[m + 1] = idx[m]; m--; }
    idx[m + 1] = k;
  }

  uint8_t heard = 0, good = 0; int16_t best = -1;
  for (uint8_t i = 0; i < nSats; i++) {
    if (sats[i].cno > 0)   heard++;
    if (sats[i].cno >= 25) good++;
    if (sats[i].cno > best) best = sats[i].cno;
  }
  bool fix = gps.location.isValid() && gps.location.age() < 3000;

  Serial.println();
  Serial.println("==================================================================");
  Serial.print("GPS DIAGNOSTIC   up "); Serial.print(now / 1000); Serial.println(" s");

  // --- links ---
  Serial.print("GPS -> RAK (PB7): ");
  if (gpsBytes == 0)      Serial.print("NO DATA");
  else if (nmeaOk == 0)   Serial.print("bytes but no valid NMEA");
  else                    Serial.print("OK");
  Serial.print("   bytes "); Serial.print(gpsBytes);
  Serial.print("  NMEA ok/bad "); Serial.print(nmeaOk); Serial.print("/"); Serial.println(nmeaBad);

  Serial.print("RAK -> GPS (PB6): ");
  Serial.println(ubxReplies ? "OK (GPS answered our commands)"
                            : (now > 8000 ? "NO REPLY yet - check PB6 -> GPS RXD" : "waiting for reply..."));
  if (verReceived) {
    Serial.print("GPS model: "); Serial.print(verMod[0] ? verMod : "?");
    Serial.print("   firmware: "); Serial.print(verSw);
    Serial.print("   hw: "); Serial.print(verHw);
    if (verProt[0]) { Serial.print("   protocol: "); Serial.print(verProt); }
    Serial.println();
  }

  // --- fix / time ---
  Serial.print("Fix: "); Serial.print(fix ? "YES" : "no");
  Serial.print("   satellites used "); Serial.print(gps.satellites.isValid() ? (int)gps.satellites.value() : 0);
  Serial.print("   HDOP ");
  if (gps.hdop.isValid() && gps.hdop.value() < 9999) Serial.print(gps.hdop.hdop(), 1); else Serial.print("-");
  Serial.print("   timepulses "); Serial.println(ppsCount);

  Serial.print("UTC: ");
  if (gps.date.isValid() && gps.date.year() >= 2024 && gps.time.isValid()) {
    Serial.print(gps.date.year()); Serial.print("-"); print2(gps.date.month()); Serial.print("-");
    print2(gps.date.day()); Serial.print(" ");
    print2(gps.time.hour()); Serial.print(":"); print2(gps.time.minute()); Serial.print(":");
    print2(gps.time.second());
  } else {
    Serial.print("not known yet");
  }
  if (fix) {
    Serial.print("   pos "); Serial.print(gps.location.lat(), 6); Serial.print(", ");
    Serial.print(gps.location.lng(), 6);
    if (gps.altitude.isValid()) { Serial.print("   alt "); Serial.print(gps.altitude.meters(), 1); Serial.print(" m"); }
  }
  Serial.println();

  // --- RF front end ---
  if (rfReceived && now - rfLastMs < 3 * RF_POLL_EVERY_MS) {
    Serial.print("RF: noise "); Serial.print(rfNoise);
    Serial.print("   AGC gain "); Serial.print((uint32_t)rfAgc * 100UL / 8191UL); Serial.print("%");
    Serial.print("   jamming ");
    const char *js[] = {"unknown", "none", "warning", "CRITICAL"};
    Serial.print(js[rfJamState]);
    Serial.print("   antenna "); Serial.println(antStatusName(rfAntStatus));
  }

  // --- satellite table ---
  Serial.print("Satellites reported "); Serial.print(nSats);
  Serial.print("   heard "); Serial.print(heard);
  Serial.print("   >=25 dB-Hz "); Serial.print(good);
  Serial.print("   strongest "); if (best > 0) { Serial.print(best); Serial.println(" dB-Hz"); } else Serial.println("-");

  if (nSats) {
    Serial.println("  SYS    PRN  ELEV  AZIM  dB-Hz");
    for (uint8_t i = 0; i < nSats; i++) {
      Sat &s = sats[idx[i]];
      Serial.print("  "); Serial.print(sysName(s.talker));
      printPadded(s.prn, 6);
      if (s.elev >= 0) printPadded(s.elev, 6); else Serial.print("     -");
      if (s.az   >= 0) printPadded(s.az, 6);   else Serial.print("     -");
      if (s.cno > 0) {
        printPadded(s.cno, 7);
        Serial.print("  ");
        for (int16_t b = 0; b < s.cno / 2; b++) Serial.print('#');
      } else {
        Serial.print("      -  (not heard)");
      }
      Serial.println();
    }
  }

  // --- hint ---
  Serial.print("Hint: ");
  if (gpsBytes == 0)
    Serial.println("no data from the GPS - check 3V3 at the GPS and GPS TXD -> RAK PB7.");
  else if (nmeaOk == 0)
    Serial.println("garbled data - wrong baud rate (GPS_BAUD) or a noisy connection.");
  else if (fix)
    Serial.println("FIX OK - GPS and antenna are working.");
  else if (good >= 4)
    Serial.println("good signals - a fix should arrive within a minute or two.");
  else if (heard > 0)
    Serial.println("satellites heard but weak/few - go outside with open sky; window glass costs 5-15 dB.");
  else if (now < 120000UL)
    Serial.println("searching (cold start). Outside with open sky, give it 2 minutes.");
  else {
    Serial.println("NO satellites heard. Check: antenna is in J1 (the PCB's U.FL, not the RAK3172's);");
    Serial.println("      it is a PASSIVE antenna (active ones get no power - VCC_RF is not wired);");
    Serial.println("      you are outside with open sky; J1 and the GPS RF_IN pad are soldered well.");
    if (rfReceived && rfAgc * 100UL / 8191UL > 90)
      Serial.println("      AGC gain is near maximum: almost no RF is reaching the GPS input.");
  }
}

// ============================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  delay(2000);
  digitalWrite(PIN_LED, LOW);

  Serial.println("==================================================================");
  Serial.println("Fireflies Nano GPS DIAGNOSTIC (LoRa radio not used)");
  Serial.println("Signal: <20 too weak | 20-30 weak | 30-40 good | >40 excellent (dB-Hz)");
  Serial.println("==================================================================");

  Serial1.begin(GPS_BAUD, RAK_CUSTOM_MODE);     // GPS on PB6/PB7
  pinMode(PIN_PPS, INPUT);
  attachInterrupt(PIN_PPS, onPps, RISING);

  delay(500);
  ubxPoll(0x0A, 0x04);                         // MON-VER: model + firmware
  lastVerPoll = millis();
}

void loop() {
  while (Serial1.available()) {
    uint8_t c = Serial1.read();
    gpsBytes++;
    if (ubxByte(c)) continue;                  // binary reply, not NMEA
    nmeaChar((char)c);
    gps.encode((char)c);
  }

  unsigned long now = millis();

  if (!verReceived && now - lastVerPoll >= 3000) { lastVerPoll = now; ubxPoll(0x0A, 0x04); }
  if (now - lastRfPoll >= RF_POLL_EVERY_MS)     { lastRfPoll = now;  ubxPoll(0x0A, 0x38); }

  // LED: solid with a fix, short blink per report while searching
  bool fix = gps.location.isValid() && gps.location.age() < 3000;
  if (fix) digitalWrite(PIN_LED, HIGH);
  else     digitalWrite(PIN_LED, (now - lastReport) < 50 ? HIGH : LOW);

  if (now - lastReport >= REPORT_EVERY_MS) {
    lastReport = now;
    report();
  }
}
