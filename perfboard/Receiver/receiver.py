#!/usr/bin/env python3
"""
Fireflies — LoRa multi-node receiver with live dashboard
Raspberry Pi 5 + XenoLabs RFM95W HAT (915 MHz)

  python3 receiver.py            full-screen live dashboard
  python3 receiver.py --plain    scrolling line output (for logs / debugging)

Writes sensor_log.csv (one row per reading) and sensor_log.json (one object
per line, for server.py). History lives in the files; the dashboard is a
live view only.

Radio config MUST MATCH firefly_tx_v2_*dBm.ino exactly.
Matched-set tag: FF-915.0-SF7-BW125-CR45-SW12-PRE8-CRC (printed by both ends).
"""

import os
import sys
import csv
import json
import time
import curses
import struct
from collections import deque
from datetime import datetime, timezone

import board
import busio
import digitalio
import adafruit_rfm9x

# ---------------------------------------------------------------
# GPIO pins — XenoLabs RFM95W HAT.  VERIFY against your HAT docs.
# ---------------------------------------------------------------
CS_PIN    = digitalio.DigitalInOut(board.D7)   # GPIO7,  physical pin 26
RESET_PIN = digitalio.DigitalInOut(board.D5)   # GPIO5,  physical pin 29

# ---------------------------------------------------------------
# RF config — MUST MATCH THE TRANSMITTERS
# ---------------------------------------------------------------
FREQUENCY        = 915.0
BANDWIDTH        = 125000
SPREADING_FACTOR = 7          # SF7 — 215 ms airtime for a 128 B packet
CODING_RATE      = 5
PREAMBLE_LENGTH  = 8
ENABLE_CRC       = True
SYNC_WORD        = 0x12       # SX127x default / RadioLib default (private network)
LOW_DATARATE_OPT = False      # LDRO — off at SF7/BW125 (symbol 1.024 ms); TX side auto = off
AUTO_AGC         = False      # receiver LNA AGC — kept at library default (off) for consistency

# One tag for the whole matched set. The transmitters print the same tag
# at boot — if the two don't match, they will not hear each other.
CONFIG_ID = "FF-915.0-SF7-BW125-CR45-SW12-PRE8-CRC"
SETTINGS_FILE_NAME = "radio_settings_log.txt"

# ---------------------------------------------------------------
# Packet filter — stops noise / foreign LoRa showing up as fake nodes
# ---------------------------------------------------------------
# Only these node IDs are accepted. Add IDs here when you build more boards.
ALLOWED_NODES = {1, 2}
# Expected seconds between packets from one node (= CYCLE_MS / 1000 on the TX).
# Only used for the dashboard live / stale / OFFLINE colours.
TX_INTERVAL_S = 1.0
REJECT_FILE_NAME = "rejected_packets.csv"

HERE     = os.path.dirname(os.path.abspath(__file__))
LOG_FILE = os.path.join(HERE, "sensor_log.json")
SETTINGS_FILE = os.path.join(HERE, SETTINGS_FILE_NAME)
REJECT_FILE   = os.path.join(HERE, REJECT_FILE_NAME)
CSV_FILE = os.path.join(HERE, "sensor_log.csv")

# ---------------------------------------------------------------
# Packet format (RadioHead's 4-byte header is stripped by receive())
#
#   [0]    uint8   node_id
#   [1..2] uint16  seq        little-endian
#   [3]    uint8   flags      bit0 time-synced, bit1 GPS fix,
#                             bit2 GPS timepulse locked (Nano only),
#                             bits3-5 GPS state (Nano only, 0 = not reported):
#                               1 no data  2 searching  3 few sats (1-3 heard)
#                               4 acquiring (4+ heard, no fix)  5 fix
#   [4..]  Reading x N        12 bytes each
# ---------------------------------------------------------------
APP_HEADER_FORMAT = '<BHB'
APP_HEADER_SIZE   = struct.calcsize(APP_HEADER_FORMAT)    # 4
READING_FORMAT    = '<hBiiB'
READING_SIZE      = struct.calcsize(READING_FORMAT)       # 12

FLAG_TIME_SYNC = 0x01
FLAG_GPS_FIX   = 0x02
FLAG_PPS_LOCK  = 0x04
GPS_STATE_SHIFT = 3
GPS_STATE_MASK  = 0x38
FLAGS_KNOWN     = FLAG_TIME_SYNC | FLAG_GPS_FIX | FLAG_PPS_LOCK | GPS_STATE_MASK   # 0x3F
GPS_STATE_NAMES = {0: "-", 1: "no data", 2: "searching", 3: "few sats",
                   4: "acquiring", 5: "fix"}


def gps_state_of(flags):
    """GPS state code from the flags byte (0 = not reported, e.g. perfboard nodes)."""
    return (flags & GPS_STATE_MASK) >> GPS_STATE_SHIFT

CSV_FIELDS = [
    "timestamp", "node_id", "seq", "reading_index",
    "temp_c", "humidity", "latitude", "longitude", "gps_valid",
    "time_synced", "rssi", "snr", "warnings",
    "gps_state", "pps_lock",
]

# ---------------------------------------------------------------
# Data validation bounds
# ---------------------------------------------------------------
TEMP_MIN, TEMP_MAX = -40.0, 125.0      # SHT40 specified range
RH_MIN,   RH_MAX   = 0.0, 100.0
# Rough Australian bounding box — widen or disable if testing elsewhere.
LAT_MIN, LAT_MAX   = -44.0, -9.0
LNG_MIN, LNG_MAX   = 112.0, 154.0
CHECK_BOUNDS       = True

# Treat a backwards sequence jump bigger than this as a node restart
# rather than catastrophic loss.
RESTART_THRESHOLD = 1000

# CSV buffering — never fsync per packet, it starves the radio
FLUSH_SECONDS = 2.0
FLUSH_ROWS    = 50


# ===============================================================
class NodeState:
    def __init__(self, node_id):
        self.node_id   = node_id
        self.first_seen = time.monotonic()
        self.last_seen  = 0.0
        self.last_seq   = None
        self.packets    = 0
        self.lost       = 0
        self.readings   = 0
        self.restarts   = 0
        self.rssi       = None
        self.rssi_min   = None
        self.rssi_sum   = 0.0
        self.snr        = None
        self.temp       = None
        self.hum        = None
        self.lat        = None
        self.lng        = None
        self.gps_valid  = False
        self.synced     = False
        self.pps        = False
        self.gps_state  = None
        self.seq        = None
        self.warn_count = 0

    @property
    def pdr(self):
        total = self.packets + self.lost
        return (self.packets / total * 100.0) if total else 0.0

    @property
    def rssi_mean(self):
        return (self.rssi_sum / self.packets) if self.packets else 0.0

    @property
    def age(self):
        return time.monotonic() - self.last_seen if self.last_seen else None


nodes  = {}
events = deque(maxlen=200)


def log_event(text, node_id=None):
    stamp = datetime.now().strftime("%H:%M:%S")
    tag   = f"[{node_id}]" if node_id is not None else "[-]"
    events.append(f"{stamp} {tag} {text}")


def node_state(node_id):
    if node_id not in nodes:
        nodes[node_id] = NodeState(node_id)
        log_event("node appeared", node_id)
    return nodes[node_id]


# ===============================================================
def validate(r):
    """Return a list of short warning codes for one reading."""
    w = []
    if not (TEMP_MIN <= r["temp_c"] <= TEMP_MAX):
        w.append("TEMP")
    if not (RH_MIN <= r["humidity"] <= RH_MAX):
        w.append("RH")
    if r["gps_valid"]:
        if r["latitude"] == 0.0 and r["longitude"] == 0.0:
            w.append("GPS0")
        elif CHECK_BOUNDS and not (LAT_MIN <= r["latitude"] <= LAT_MAX
                                   and LNG_MIN <= r["longitude"] <= LNG_MAX):
            w.append("POS")
    return w


def parse_payload(raw: bytes):
    """Decode one application payload -> (node_id, seq, flags, [readings], warnings)."""
    # Some library/HAT combinations hand back hex-encoded ASCII.
    try:
        cand = raw.decode("ascii").strip()
        if cand and len(cand) % 2 == 0 and \
           all(c in "0123456789ABCDEFabcdef" for c in cand):
            raw = bytes.fromhex(cand)
    except Exception:
        pass

    pkt_warn = []
    minimum = APP_HEADER_SIZE + READING_SIZE
    if len(raw) < minimum:
        return None, [f"LEN short {len(raw)}B"]

    node_id, seq, flags = struct.unpack(APP_HEADER_FORMAT, raw[:APP_HEADER_SIZE])
    payload = raw[APP_HEADER_SIZE:]

    n = len(payload) // READING_SIZE
    if len(payload) % READING_SIZE:
        pkt_warn.append("LEN")
    if n == 0:
        return None, ["LEN no readings"]

    readings = []
    for i in range(n):
        chunk = payload[i * READING_SIZE:(i + 1) * READING_SIZE]
        try:
            t10, hum, lat6, lng6, gv = struct.unpack(READING_FORMAT, chunk)
        except struct.error:
            pkt_warn.append("UNPACK")
            continue
        r = {
            "node_id":       node_id,
            "seq":           seq,
            "reading_index": i,
            "temp_c":        round(t10 / 10.0, 1),
            "humidity":      float(hum),
            "latitude":      round(lat6 / 1e6, 6),
            "longitude":     round(lng6 / 1e6, 6),
            "gps_valid":     bool(gv),
            "time_synced":   bool(flags & FLAG_TIME_SYNC),
            "gps_state":     GPS_STATE_NAMES[gps_state_of(flags)],
            "pps_lock":      bool(flags & FLAG_PPS_LOCK),
        }
        rw = validate(r)
        r["warnings"] = "|".join(rw)
        if rw:
            pkt_warn.extend(rw)
        readings.append(r)

    if not readings:
        return None, pkt_warn or ["UNPACK"]
    return (node_id, seq, flags, readings), sorted(set(pkt_warn))


def track_sequence(st, seq):
    """Update loss stats. Detects transmitter restarts instead of
    counting them as ~65000 lost packets (the v1 bug)."""
    st.packets += 1
    if st.last_seq is None:
        st.last_seq = seq
        return
    gap = (seq - st.last_seq) & 0xFFFF
    if gap == 0:
        log_event(f"duplicate seq {seq}", st.node_id)
    elif gap > RESTART_THRESHOLD:
        # Backwards jump / implausible forward jump -> node rebooted.
        st.restarts += 1
        st.packets = 1
        st.lost = 0
        st.rssi_sum = st.rssi or 0.0
        log_event(f"RESYNC — seq reset to {seq}, counters cleared", st.node_id)
    elif gap > 1:
        st.lost += gap - 1
    st.last_seq = seq


# ===============================================================
class Logger:
    """Buffered writer. No per-packet fsync — that starves the radio."""

    def __init__(self):
        self.rows = []
        self.last_flush = time.monotonic()
        self.total_rows = 0
        self.last_write = None
        self._rotate_if_old_columns()
        new = not os.path.exists(CSV_FILE) or os.path.getsize(CSV_FILE) == 0
        self.csv_f = open(CSV_FILE, "a", newline="")
        self.writer = csv.DictWriter(self.csv_f, fieldnames=CSV_FIELDS)
        if new:
            self.writer.writeheader()
            self.csv_f.flush()
        self.json_f = open(LOG_FILE, "a")
        if not new:
            try:
                with open(CSV_FILE) as f:
                    self.total_rows = max(0, sum(1 for _ in f) - 1)
            except Exception:
                pass

    @staticmethod
    def _rotate_if_old_columns():
        """A CSV written with an older column list is renamed, not appended to,
        so rows never land under the wrong headings."""
        if not os.path.exists(CSV_FILE) or os.path.getsize(CSV_FILE) == 0:
            return
        with open(CSV_FILE, newline="") as f:
            header = next(csv.reader(f), [])
        if header == CSV_FIELDS:
            return
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        base, ext = os.path.splitext(CSV_FILE)
        old = f"{base}_oldcols_{stamp}{ext}"
        os.rename(CSV_FILE, old)
        print(f"[info] {os.path.basename(CSV_FILE)} had the old column list; "
              f"moved to {os.path.basename(old)} and starting a new file.")

    def add(self, readings, rssi, snr):
        ts = datetime.now(timezone.utc).isoformat()
        for r in readings:
            row = dict(r)
            row["timestamp"] = ts
            row["rssi"] = rssi
            row["snr"] = snr
            self.rows.append(row)

    def maybe_flush(self, force=False):
        now = time.monotonic()
        if not self.rows:
            return
        if force or len(self.rows) >= FLUSH_ROWS or \
           (now - self.last_flush) >= FLUSH_SECONDS:
            for row in self.rows:
                self.writer.writerow({k: row.get(k, "") for k in CSV_FIELDS})
                self.json_f.write(json.dumps(
                    {k: row.get(k) for k in CSV_FIELDS}) + "\n")
            self.csv_f.flush()
            self.json_f.flush()
            self.total_rows += len(self.rows)
            self.last_write = now
            self.rows.clear()
            self.last_flush = now

    def close(self):
        self.maybe_flush(force=True)
        try:
            os.fsync(self.csv_f.fileno())
            os.fsync(self.json_f.fileno())
        except Exception:
            pass
        self.csv_f.close()
        self.json_f.close()

    @property
    def size_mb(self):
        try:
            return os.path.getsize(CSV_FILE) / 1e6
        except Exception:
            return 0.0

    @property
    def write_age(self):
        return time.monotonic() - self.last_write if self.last_write else None


# ===============================================================
def init_radio():
    spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
    rfm9x = adafruit_rfm9x.RFM9x(spi, CS_PIN, RESET_PIN, FREQUENCY)
    rfm9x.signal_bandwidth = BANDWIDTH        # bandwidth first, then SF
    rfm9x.spreading_factor = SPREADING_FACTOR
    rfm9x.coding_rate      = CODING_RATE
    rfm9x.preamble_length  = PREAMBLE_LENGTH
    rfm9x.enable_crc       = ENABLE_CRC
    rfm9x.low_datarate_optimize = 1 if LOW_DATARATE_OPT else 0
    rfm9x.auto_agc         = AUTO_AGC
    rfm9x._write_u8(0x39, SYNC_WORD)          # RegSyncWord — library never sets it; make it explicit
    rfm9x.node             = 0xFF             # hear every node

    # Read back from the chip so a wrong setting fails loudly instead of silently.
    got = {
        "spreading_factor": rfm9x.spreading_factor,
        "signal_bandwidth": rfm9x.signal_bandwidth,
        "coding_rate":      rfm9x.coding_rate,
        "sync_word":        rfm9x._read_u8(0x39),
        "enable_crc":       rfm9x.enable_crc,
    }
    want = {
        "spreading_factor": SPREADING_FACTOR,
        "signal_bandwidth": BANDWIDTH,
        "coding_rate":      CODING_RATE,
        "sync_word":        SYNC_WORD,
        "enable_crc":       ENABLE_CRC,
    }
    bad = {k: (got[k], want[k]) for k in want if got[k] != want[k]}
    if bad:
        raise RuntimeError(f"Radio readback mismatch (got, wanted): {bad}")
    return rfm9x


def settings_text():
    return (f"config={CONFIG_ID}  freq={FREQUENCY}MHz  SF{SPREADING_FACTOR}  "
            f"BW{BANDWIDTH//1000}kHz  CR4/{CODING_RATE}  sync=0x{SYNC_WORD:02X}  "
            f"preamble={PREAMBLE_LENGTH}  CRC={'on' if ENABLE_CRC else 'off'}  "
            f"LDRO={'on' if LOW_DATARATE_OPT else 'off'}  AGC={'on' if AUTO_AGC else 'off'}  "
            f"header=explicit")


def log_settings():
    """Append one line per receiver start, so every CSV row can be traced to its settings."""
    with open(SETTINGS_FILE, "a") as f:
        f.write(f"{datetime.now(timezone.utc).isoformat()}  START  {settings_text()}\n")


RH_HEADER_LEN = 4
REG_HOP_CHANNEL = 0x1C          # SX1276 RegHopChannel; bit 6 = CRC was on in the received header


def check_frame(raw, crc_in_header):
    """Return None if this looks like a genuine Fireflies v2 packet, else a short reason.

    raw is the WHOLE frame including the 4-byte RadioHead header
    (rfm9x.receive(with_header=True)). The transmitter writes node_id and
    seq into BOTH the RadioHead header and the app header, so noise or a
    foreign LoRa packet almost never matches on every check below.
    """
    if not crc_in_header:
        # Same test RadioLib applies ("LoRa header damaged"): our TX always
        # sends a CRC, so a header claiming no CRC is noise or not ours.
        return "no-CRC"
    n_app = len(raw) - RH_HEADER_LEN - APP_HEADER_SIZE
    if n_app <= 0 or n_app % READING_SIZE or n_app // READING_SIZE > 10:
        return f"length {len(raw)}B"
    h_dest, h_src, h_id, h_flags = raw[0], raw[1], raw[2], raw[3]
    node_id, seq, flags = struct.unpack(
        APP_HEADER_FORMAT, raw[RH_HEADER_LEN:RH_HEADER_LEN + APP_HEADER_SIZE])
    if h_dest != 0xFF or h_flags != 0x00:
        return f"RH header {h_dest:02X}/{h_flags:02X}"
    if h_src != node_id or h_id != (seq & 0xFF):
        return "header mismatch"
    if node_id not in ALLOWED_NODES:
        return f"node {node_id} not allowed"
    if flags & ~FLAGS_KNOWN & 0xFF or gps_state_of(flags) not in GPS_STATE_NAMES:
        return f"flags {flags:02X}"
    return None


class RejectLog:
    """Rejected frames go here, not into sensor_log.csv / the dashboard."""

    def __init__(self):
        new = not os.path.exists(REJECT_FILE) or os.path.getsize(REJECT_FILE) == 0
        self.f = open(REJECT_FILE, "a", newline="")
        self.w = csv.writer(self.f)
        if new:
            self.w.writerow(["timestamp", "reason", "length", "rssi", "snr", "hex"])
            self.f.flush()

    def add(self, raw, reason, rssi, snr):
        self.w.writerow([datetime.now(timezone.utc).isoformat(), reason,
                         len(raw), rssi, snr, bytes(raw).hex()])
        self.f.flush()

    def close(self):
        self.f.close()


def handle_frame(raw, crc_in_header, rssi, snr, logger, rejects, stats):
    stats["packets"] += 1
    reason = check_frame(raw, crc_in_header)
    if reason:
        stats["rejected"] += 1
        rejects.add(raw, reason, rssi, snr)
        log_event(f"rejected {len(raw)} B, RSSI {rssi}, SNR {snr} — {reason}")
        return
    handle_packet(bytes(raw[RH_HEADER_LEN:]), rssi, snr, logger, stats)


def receive_frame(rfm9x, timeout):
    """Receive with the RadioHead header kept, plus whether the header said CRC-on."""
    raw = rfm9x.receive(timeout=timeout, with_header=True)
    if raw is None:
        return None, False
    crc_on = bool(rfm9x._read_u8(REG_HOP_CHANNEL) & 0x40)
    return raw, crc_on


def handle_packet(pkt, rssi, snr, logger, stats):
    parsed, warns = parse_payload(pkt)
    if parsed is None:
        stats["rejected"] += 1
        log_event(f"undecodable packet, {len(pkt)} B, RSSI {rssi} — {','.join(warns)}")
        return

    node_id, seq, flags, readings = parsed
    st = node_state(node_id)
    track_sequence(st, seq)

    st.last_seen = time.monotonic()
    st.seq       = seq
    st.rssi      = rssi
    st.snr       = snr
    st.rssi_sum += rssi
    st.rssi_min  = rssi if st.rssi_min is None else min(st.rssi_min, rssi)
    st.readings += len(readings)
    st.synced    = bool(flags & FLAG_TIME_SYNC)
    st.pps       = bool(flags & FLAG_PPS_LOCK)
    gstate       = gps_state_of(flags)
    if gstate and gstate != st.gps_state:
        log_event(f"GPS {GPS_STATE_NAMES.get(st.gps_state, '-')} -> {GPS_STATE_NAMES[gstate]}"
                  f"{' (PPS)' if st.pps else ''}", node_id)
    st.gps_state = gstate

    last = readings[-1]
    st.temp      = last["temp_c"]
    st.hum       = last["humidity"]
    st.lat       = last["latitude"]
    st.lng       = last["longitude"]
    st.gps_valid = last["gps_valid"]

    if warns:
        st.warn_count += 1
        log_event(f"seq {seq} — {', '.join(warns)}", node_id)

    logger.add(readings, rssi, snr)


# ===============================================================
#  Dashboard
# ===============================================================
def draw(stdscr, logger, stats, started):
    stdscr.erase()
    h, w = stdscr.getmaxyx()
    if h < 18 or w < 76:
        stdscr.addstr(0, 0, "Terminal too small — need 76x18 or larger.")
        stdscr.refresh()
        return

    A_HEAD = curses.color_pair(1) | curses.A_BOLD
    A_OK   = curses.color_pair(2)
    A_WARN = curses.color_pair(3)
    A_BAD  = curses.color_pair(4)
    A_DIM  = curses.A_DIM

    up = int(time.monotonic() - started)
    title = " FIREFLIES BASE STATION "
    cfg   = f" {CONFIG_ID} "
    stdscr.addstr(0, 0, title.ljust(w - len(cfg))[:w - len(cfg)], A_HEAD)
    stdscr.addstr(0, w - len(cfg) - 1, cfg, A_HEAD)

    stdscr.addstr(1, 1,
                  f"uptime {up//3600:02d}:{(up//60)%60:02d}:{up%60:02d}"
                  f"    packets {stats['packets']:,}"
                  f"    rejected {stats['rejected']}"
                  f"    nodes {len(nodes)}", A_DIM)

    # ---- node panels, two per row ----
    row = 3
    COLW = max(36, (w - 3) // 2)
    for idx, nid in enumerate(sorted(nodes)):
        st = nodes[nid]
        col = 1 + (idx % 2) * COLW
        if idx % 2 == 0 and idx > 0:
            row += 11
        if row + 10 >= h - 8:
            break

        age = st.age
        if age is None or age > 5 * TX_INTERVAL_S:
            head_attr, status = A_BAD, "OFFLINE"
        elif age > 2.5 * TX_INTERVAL_S:
            head_attr, status = A_WARN, "stale"
        else:
            head_attr, status = A_OK, "live"

        def put(dy, label, value, attr=0):
            if row + dy < h - 1:
                stdscr.addstr(row + dy, col, f"{label:<9}"[:9], A_DIM)
                stdscr.addstr(row + dy, col + 9, str(value)[:COLW - 10], attr)

        stdscr.addstr(row, col, f"NODE {nid}  {status}"[:COLW - 1], head_attr | curses.A_BOLD)
        put(1, "seq",  f"{st.seq if st.seq is not None else '-'}")
        put(2, "temp", f"{st.temp:.1f} C" if st.temp is not None else "-")
        put(3, "RH",   f"{st.hum:.0f} %"  if st.hum  is not None else "-")
        if st.gps_valid and st.lat is not None:
            put(4, "GPS", f"{st.lat:.6f}, {st.lng:.6f}", A_OK)
        elif st.gps_state:
            put(4, "GPS", f"no fix - {GPS_STATE_NAMES[st.gps_state]}",
                A_BAD if st.gps_state == 1 else A_WARN)
        else:
            put(4, "GPS", "no fix", A_WARN)
        put(5, "RSSI", f"{st.rssi} dBm  (min {st.rssi_min}, avg {st.rssi_mean:.0f})"
                       if st.rssi is not None else "-")
        put(6, "SNR",  f"{st.snr} dB" if st.snr is not None else "-")
        pdr_attr = A_OK if st.pdr >= 95 else (A_WARN if st.pdr >= 80 else A_BAD)
        put(7, "PDR",  f"{st.pdr:.1f} %   ({st.packets} rx, {st.lost} lost)", pdr_attr)
        put(8, "sync", ("GPS+PPS" if st.pps else "GPS") if st.synced else "free-run",
            A_OK if st.synced else A_WARN)
        put(9, "last", f"{age:.1f} s ago" if age is not None else "never")

    # ---- CSV status ----
    csv_row = h - 8
    wa = logger.write_age
    stdscr.addstr(csv_row, 1, "─" * (w - 2), A_DIM)
    stdscr.addstr(csv_row + 1, 1,
                  f"CSV  {os.path.basename(CSV_FILE)}   "
                  f"{logger.total_rows:,} rows   "
                  f"{logger.size_mb:.2f} MB   "
                  f"buffered {len(logger.rows)}   "
                  f"last write {f'{wa:.1f} s ago' if wa is not None else 'never'}"[:w - 2],
                  A_OK if (wa is not None and wa < 5) else A_WARN)

    # ---- events ----
    ev_row = h - 6
    stdscr.addstr(ev_row, 1, "EVENTS", A_HEAD)
    for i, line in enumerate(list(events)[-(h - ev_row - 2):]):
        if ev_row + 1 + i >= h - 1:
            break
        attr = A_WARN if ("WARN" in line or "—" in line and "RESYNC" not in line) else A_DIM
        if "RESYNC" in line:
            attr = A_BAD
        stdscr.addstr(ev_row + 1 + i, 2, line[:w - 3], attr)

    stdscr.addstr(h - 1, 1, "q or Ctrl+C to quit"[:w - 2], A_DIM)
    stdscr.refresh()


def run_dashboard(stdscr, rfm9x, logger, rejects, stats, started):
    curses.curs_set(0)
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_CYAN)
    curses.init_pair(2, curses.COLOR_GREEN, -1)
    curses.init_pair(3, curses.COLOR_YELLOW, -1)
    curses.init_pair(4, curses.COLOR_RED, -1)
    stdscr.nodelay(True)

    last_draw = 0.0
    while True:
        ch = stdscr.getch()
        if ch in (ord('q'), ord('Q')):
            break

        raw, crc_on = receive_frame(rfm9x, 0.25)
        if raw is not None:
            handle_frame(raw, crc_on, rfm9x.last_rssi, rfm9x.last_snr,
                         logger, rejects, stats)

        logger.maybe_flush()

        now = time.monotonic()
        if now - last_draw > 0.25:
            draw(stdscr, logger, stats, started)
            last_draw = now


def run_plain(rfm9x, logger, rejects, stats, started):
    print("Fireflies receiver")
    print(f"  {settings_text()}")
    print(f"CSV -> {CSV_FILE}\n")
    hdr = 0
    while True:
        raw, crc_on = receive_frame(rfm9x, 1.0)
        logger.maybe_flush()
        if raw is None:
            continue
        handle_frame(raw, crc_on, rfm9x.last_rssi, rfm9x.last_snr,
                     logger, rejects, stats)
        if hdr % 20 == 0:
            print(f"\n{'TIME':<9}{'NODE':<6}{'SEQ':>7}{'TEMP':>8}{'RH':>6}"
                  f"{'GPS':>26}{'RSSI':>7}{'SNR':>7}{'PDR':>8}")
        hdr += 1
        for nid in sorted(nodes):
            st = nodes[nid]
            if st.age is not None and st.age < 0.5:
                gps = (f"{st.lat:.5f},{st.lng:.5f}" if st.gps_valid else
                       f"no fix ({GPS_STATE_NAMES[st.gps_state]})" if st.gps_state else "no fix")
                print(f"{datetime.now().strftime('%H:%M:%S'):<9}"
                      f"{nid:<6}{st.seq:>7}{st.temp:>7.1f}C{st.hum:>5.0f}%"
                      f"{gps:>26}{st.rssi:>7}{st.snr:>7.1f}{st.pdr:>7.1f}%")


# ===============================================================
def main():
    plain = "--plain" in sys.argv
    stats = {"packets": 0, "rejected": 0}
    started = time.monotonic()

    try:
        rfm9x = init_radio()
    except Exception as e:
        print(f"[FATAL] Radio init failed: {e}")
        print("Check the HAT seating, SPI enabled (raspi-config), "
              "and that CS_PIN/RESET_PIN match your HAT.")
        raise

    log_settings()
    logger = Logger()
    rejects = RejectLog()
    log_event(f"receiver started — SF{SPREADING_FACTOR} BW{BANDWIDTH//1000}k")

    try:
        if plain:
            run_plain(rfm9x, logger, rejects, stats, started)
        else:
            curses.wrapper(run_dashboard, rfm9x, logger, rejects, stats, started)
    except KeyboardInterrupt:
        pass
    finally:
        logger.close()
        rejects.close()
        print("\nLink summary")
        for nid in sorted(nodes):
            st = nodes[nid]
            print(f"  node {nid}: {st.packets} received, {st.lost} lost, "
                  f"{st.readings} readings, PDR {st.pdr:.1f}%, "
                  f"{st.restarts} restarts, {st.warn_count} packets with warnings")
        print(f"  CSV: {logger.total_rows:,} rows, {logger.size_mb:.2f} MB "
              f"-> {CSV_FILE}")
        print(f"  Rejected (noise / foreign / not ours): {stats['rejected']} "
              f"-> {REJECT_FILE}")


if __name__ == "__main__":
    main()
