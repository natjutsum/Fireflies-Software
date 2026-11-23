# Fireflies — latest code (snapshot 3 Oct 2026)

**Matched radio set:** `FF-915.0-SF7-BW125-CR45-SW12-PRE8-CRC` — printed at boot by every
transmitter and by the receiver. If the two tags differ, they will not hear each other.
Full settings table (old SF9 vs new SF7) for the test spreadsheet: `RADIO_SETTINGS.csv`.

The receiver appends one line per start to `radio_settings_log.txt` next to its CSV, so every
logged row can be traced to the settings used.

Source: `Claude outputs/firefly_tx_v2.ino` and `Claude outputs/receiver.py`
(both last modified 17 Sep 2026 — the newest TX/RX code in the repo).

## Transmitter (XIAO RP2040 + RFM95W + SHT41 + u-blox GPS)
Set `NODE_ID` (1 or 2) at the top before flashing. Each folder is a complete Arduino sketch.

| Sketch | TX power | OCP limit | Use |
|---|---|---|---|
| `firefly_tx_v2_5dBm`  | 5 dBm  (~3 mW)   | 60 mA  | Bench / short-range testing |
| `firefly_tx_v2_17dBm` | 17 dBm (~50 mW)  | 120 mA | Fallback if the supply sags / brown-outs at full power |
| `firefly_tx_v2_20dBm` | 20 dBm (~100 mW) | 150 mA | Full power — the SX1276 maximum |

Only the power, OCP and banner lines differ from v2. Everything else (SF7/BW125/CR4-5,
TDMA slots, 128 B packet format) is unchanged, so all three work with the same receiver.

Notes:
- 22 dBm is not possible on the RFM95W/SX1276. RadioLib accepts 2–17 dBm or exactly 20 dBm
  on PA_BOOST; 18, 19 and 21+ return an error (the sketch now halts and prints it).
- These are conducted output powers in dBm. dBi is antenna gain; EIRP = dBm + antenna dBi − cable loss.
- RadioLib's `begin()` sets the over-current limit to 60 mA, which is below the PA_BOOST
  draw at 17/20 dBm. The original v2 never raised it; these variants do.
- Compile-checked 3 Oct 2026: RadioLib 7.8.1, rp2040:rp2040 6.2.0, board Seeed XIAO RP2040.

## Receiver (Raspberry Pi 5 + RFM95W HAT)
`Receiver/receiver.py` — unchanged copy. `python3 receiver.py` (dashboard) or `--plain`.

## Fix log
- **3 Oct 2026 — TDMA "sends once then stops" bug (inherited from firefly_tx_v2.ino).**
  Once GPS supplied time (possible before a position fix), the slot anchor was refreshed every
  second, so `cycleNumber()` was always 0 and the node never fired again. `cycleNumber()` now
  counts absolute GPS UTC seconds, with a half-cycle hold-off against a double fire at the
  free-run -> GPS switch-over. Pre-fix copies: `_backup_pre_tdma_fix/`.
  The original `Claude outputs/firefly_tx_v2.ino` still has the bug — do not flash it.
- **3 Oct 2026 — CYCLE_MS is now the single timing knob.** Slot timing uses absolute UTC
  ms-of-day, and `SAMPLE_INTERVAL_MS = CYCLE_MS / BATCH_SIZE`, so a longer cycle samples more
  slowly instead of dropping readings. The 17 dBm sketch is set to CYCLE_MS 5000 (2 Hz sampling,
  one packet every 5 s); 5 and 20 dBm remain at 1000 (10 Hz, one packet per second).
- **3 Oct 2026 — silent after a restart (frozen GPS time).** Before satellite lock the GPS sends
  an empty time field, which TinyGPSPlus reports as valid and fresh every second (00:00:00, or
  stuck at the last real time). The slot clock re-anchored to that frozen value and stopped
  advancing. The node now anchors only when the GPS date is real (year >= 2024) and the time has
  advanced; otherwise it extrapolates from the last good anchor or free-runs on millis().
  Pre-fix copies: `_backup_pre_tdma_fix/*.pre-frozen-time-fix`.
- **3 Oct 2026 — receiver packet filter (ghost nodes).** Frames are now accepted only if the LoRa
  header says CRC-on (same test RadioLib uses for "header damaged"), the length is 8 + 12n bytes,
  the RadioHead header matches the app header (node_id and seq low byte), and the node is in
  `ALLOWED_NODES` (default {1, 2}). Rejects go to `rejected_packets.csv`, not the data CSV.
  `TX_INTERVAL_S` sets the dashboard stale/offline timing (set 5.0 if the TX uses CYCLE_MS 5000).
  Tested: 0 of 200,000 random frames passed. md5 83d474b0f26903145dd6e9db47343659, 682 lines.
