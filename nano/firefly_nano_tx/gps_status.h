// ============================================================
// gps_status.h — GPS progress for the packet flags byte
// (identical copy in firefly_nano_tx/ and firefly_nano_test/)
//
// Flags byte (app header [3]):
//   bit0     TIME_SYNC   slot timing locked to GPS time      (unchanged)
//   bit1     GPS_FIX     position fix valid                  (unchanged)
//   bit2     PPS_LOCK    GPS timepulse seen in the last 1.5 s
//   bits3-5  GPS_STATE   0 = not reported (perfboard firmware)
//                        1 = NO DATA    nothing from the GPS for 3 s
//                        2 = SEARCHING  GPS talking, no satellites heard
//                        3 = FEW SATS   1-3 satellites heard
//                        4 = ACQUIRING  4+ satellites heard, no fix yet
//                        5 = FIX        position fix
//                        6, 7 reserved
//   bits6-7  reserved, always 0
//
// "Heard" = the GPS reports a signal strength (C/N0) for the satellite in
// its GSV sentences, i.e. it is actually tracking it.
//
// Usage: gpsStatusFeed(c) for every byte from the GPS, then
//        gpsStatusFlags(fix, ppsCount, ppsMs) when building a packet.
// ============================================================
#pragma once
#include <Arduino.h>

#define FLAG_PPS_LOCK     0x04
#define GPS_STATE_SHIFT   3
#define GPS_STATE_MASK    0x38

#define GPS_STATE_UNKNOWN   0
#define GPS_STATE_NO_DATA   1
#define GPS_STATE_SEARCHING 2
#define GPS_STATE_FEW_SATS  3
#define GPS_STATE_ACQUIRING 4
#define GPS_STATE_FIX       5

#define GPS_NODATA_MS     3000   // no valid NMEA for this long -> NO DATA
#define GPS_SATS_STALE_MS 3000   // drop a GSV count older than this
#define PPS_LOCK_MS       1500   // timepulse must be this recent for PPS_LOCK

// One entry per constellation (talker) and signal ID from the GSV sentences.
struct GsvCount {
  char          talker[2];
  char          sigId;
  uint8_t       building;   // satellites heard so far in the current GSV cycle
  uint8_t       count;      // satellites heard in the last complete cycle
  unsigned long doneMs;     // when that cycle completed (0 = never)
};

#define GSV_SLOTS 10
static GsvCount      gsvTab[GSV_SLOTS];
static uint8_t       gsvUsed     = 0;
static unsigned long lastNmeaMs  = 0;   // last sentence with a good checksum
static char          gsLine[100];
static uint8_t       gsLen       = 0;
static bool          gsIn        = false;

static GsvCount *gsvSlot(char t0, char t1, char sig) {
  for (uint8_t i = 0; i < gsvUsed; i++)
    if (gsvTab[i].talker[0] == t0 && gsvTab[i].talker[1] == t1 && gsvTab[i].sigId == sig)
      return &gsvTab[i];
  if (gsvUsed >= GSV_SLOTS) return NULL;
  GsvCount *s = &gsvTab[gsvUsed++];
  s->talker[0] = t0; s->talker[1] = t1; s->sigId = sig;
  s->building = 0; s->count = 0; s->doneMs = 0;
  return s;
}

static void gsvSentence(char *line) {
  // line: "$GPGSV,2,1,07,02,45,120,31,...[,sig]" with the checksum already removed
  char   *f[24];
  uint8_t n = 0;
  char   *p = line;
  f[n++] = p;
  while (*p && n < 24) {
    if (*p == ',') { *p = 0; f[n++] = p + 1; }
    p++;
  }
  if (n < 4) return;
  int total  = atoi(f[1]);
  int msgNum = atoi(f[2]);
  if (total < 1 || msgNum < 1 || msgNum > total) return;

  // NMEA 4.10+ adds a signal ID as the last field: 4 + 4k + 1 fields.
  uint8_t sats = n - 4;
  char    sig  = '0';
  if (sats % 4 == 1) { sig = f[n - 1][0] ? f[n - 1][0] : '0'; sats--; }

  GsvCount *s = gsvSlot(line[1], line[2], sig);
  if (!s) return;
  if (msgNum == 1) s->building = 0;
  for (uint8_t g = 4; g + 3 < 4 + sats; g += 4) {
    if (!f[g][0]) continue;                       // empty PRN
    if (f[g + 3][0] && atoi(f[g + 3]) > 0) s->building++;
  }
  if (msgNum == total) { s->count = s->building; s->doneMs = millis(); }
}

static void gpsStatusLine(char *line) {
  char *star = strrchr(line, '*');
  if (!star || star - line < 6) return;
  uint8_t cs = 0;
  for (char *q = line + 1; q < star; q++) cs ^= (uint8_t)*q;
  if ((uint8_t)strtol(star + 1, NULL, 16) != cs) return;
  lastNmeaMs = millis();
  if (lastNmeaMs == 0) lastNmeaMs = 1;
  if (strncmp(line + 3, "GSV,", 4) != 0) return;
  *star = 0;
  gsvSentence(line);
}

// Call for every byte received from the GPS.
static inline void gpsStatusFeed(char c) {
  if (c == '$') { gsIn = true; gsLen = 0; }
  if (!gsIn) return;
  if (c == '\r' || c == '\n') {
    gsLine[gsLen] = 0;
    gsIn = false;
    if (gsLen > 6) gpsStatusLine(gsLine);
    return;
  }
  if (gsLen < sizeof(gsLine) - 1) gsLine[gsLen++] = c;
  else gsIn = false;                              // too long — not NMEA
}

// Satellites currently heard, all constellations. For a constellation sent on
// several signal IDs the largest count is used, so a satellite is not counted twice.
static uint8_t gpsSatsHeard() {
  unsigned long now = millis();
  uint16_t total = 0;
  for (uint8_t i = 0; i < gsvUsed; i++) {
    GsvCount &a = gsvTab[i];
    if (!a.doneMs || now - a.doneMs > GPS_SATS_STALE_MS) continue;
    bool dup = false; uint8_t best = a.count;
    for (uint8_t j = 0; j < gsvUsed; j++) {
      GsvCount &b = gsvTab[j];
      if (j == i || b.talker[0] != a.talker[0] || b.talker[1] != a.talker[1]) continue;
      if (!b.doneMs || now - b.doneMs > GPS_SATS_STALE_MS) continue;
      if (j < i) { dup = true; break; }           // constellation already counted
      if (b.count > best) best = b.count;
    }
    if (!dup) total += best;
  }
  return total > 255 ? 255 : (uint8_t)total;
}

static uint8_t gpsState(bool fix) {
  if (!lastNmeaMs || millis() - lastNmeaMs > GPS_NODATA_MS) return GPS_STATE_NO_DATA;
  if (fix) return GPS_STATE_FIX;
  uint8_t heard = gpsSatsHeard();
  if (heard == 0) return GPS_STATE_SEARCHING;
  if (heard < 4)  return GPS_STATE_FEW_SATS;
  return GPS_STATE_ACQUIRING;
}

static bool ppsLocked(uint32_t ppsCount, unsigned long ppsMs) {
  return ppsCount > 0 && millis() - ppsMs < PPS_LOCK_MS;
}

// Bits 2-5 of the flags byte. OR this into the TIME_SYNC / GPS_FIX bits.
static uint8_t gpsStatusFlags(bool fix, uint32_t ppsCount, unsigned long ppsMs) {
  uint8_t f = (uint8_t)(gpsState(fix) << GPS_STATE_SHIFT);
  if (ppsLocked(ppsCount, ppsMs)) f |= FLAG_PPS_LOCK;
  return f;
}

static const char *gpsStateName(uint8_t s) {
  switch (s) {
    case GPS_STATE_NO_DATA:   return "NO DATA";
    case GPS_STATE_SEARCHING: return "searching";
    case GPS_STATE_FEW_SATS:  return "few sats";
    case GPS_STATE_ACQUIRING: return "acquiring";
    case GPS_STATE_FIX:       return "FIX";
  }
  return "-";
}
