# Fireflies — latest code (snapshot 3 Oct 2026)

**Matched radio set:** `FF-915.0-SF7-BW125-CR45-SW12-PRE8-CRC` — printed at boot by every
transmitter and by the receiver. If the two tags differ, they will not hear each other.
Full settings table (old SF9 vs new SF7) for the test spreadsheet: `RADIO_SETTINGS.csv`.

The receiver appends one line per start to `radio_settings_log.txt` next to its CSV, so every
logged row can be traced to the settings used.

## Transmitter (XIAO RP2040 + RFM95W + SHT41 + u-blox GPS)
Set `NODE_ID` at the top before flashing. Each folder is a complete Arduino sketch.

| Sketch | TX power | OCP limit | Use |
|---|---|---|---|
| `firefly_tx_v2_5dBm`  | 5 dBm  (~3 mW)   | 60 mA  | Bench / short-range testing |
| `firefly_tx_v2_17dBm` | 17 dBm (~50 mW)  | 120 mA | Fallback if the supply sags / brown-outs at full power |
| `firefly_tx_v2_20dBm` | 20 dBm (~100 mW) | 150 mA | Full power — the SX1276 maximum |

Only the power, OCP and banner lines differ from v2. Everything else (SF7/BW125/CR4-5,
TDMA slots, 128 B packet format) is unchanged, so all three work with the same receiver.

Notes:
- 22 dBm is not possible on the RFM95W/SX1276. RadioLib accepts 2–17 dBm or exactly 20 dBm
  on PA_BOOST; 18, 19 and 21+ return an error.
- These are conducted output powers in dBm. dBi is antenna gain; EIRP = dBm + antenna dBi − cable loss.
- RadioLib's `begin()` sets the over-current limit to 60 mA, which is below the PA_BOOST
  draw at 17/20 dBm. The original v2 never raised it; these variants do.
- Compile-checked 3 Oct 2026: RadioLib 7.8.1, rp2040:rp2040 6.2.0, board Seeed XIAO RP2040.

## Receiver (Raspberry Pi 5 + RFM95W HAT)
`Receiver/receiver.py` — unchanged copy. `python3 receiver.py` (dashboard) or `--plain`.
