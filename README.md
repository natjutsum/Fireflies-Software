# Fireflies — Software

Firmware and receiver software for Fireflies: self-sacrificing, auto-rotating bush-fire monitoring
sensors, deployable from firefighting aircraft.

| Folder | Contents |
|---|---|
| [`perfboard/`](perfboard) | Perfboard nodes (XIAO RP2040 + RFM95W + SHT41 + u-blox GPS) and the Raspberry Pi receiver (`receiver.py`) |
| [`nano/`](nano) | Fireflies Nano PCB firmware (RAK3172 + u-blox MAX-M10S + SHT4x) |

Both node types use the same radio settings (`FF-915.0-SF7-BW125-CR45-SW12-PRE8-CRC`) and packet
format, so they can run together. See each folder's README for details.
