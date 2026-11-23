# Fireflies Nano — RAK3172 transmitter firmware

`firefly_nano_tx/firefly_nano_tx.ino` is the perfboard firmware
(`Code/Fireflies_Latest_2026-10-03/Transmitter/firefly_tx_v2`) ported to the Fireflies Nano PCB:
RAK3172 (STM32WLE5 with built-in LoRa radio) + u-blox MAX-M10S GPS + Sensirion SHT4x.

It sends the **same packet layout on the same radio settings** as the perfboard nodes
(`FF-915.0-SF7-BW125-CR45-SW12-PRE8-CRC`, 128 B, GPS-slotted TDMA), so Nano and perfboard
nodes can run together. The Nano also puts its GPS state in the flags byte (below), so use
`receiver.py` from 21 Oct 2026 or later. Older copies reject Nano packets as `flags XX`.

## GPS state in the flags byte

Both Nano sketches include `gps_status.h` (an identical copy in each sketch folder; Arduino IDE
shows it as a second tab). Bits 0-1 mean the same as on the perfboard nodes:

| Bits | Meaning |
|---|---|
| 0 | Time synced to GPS (slot timing on GPS time) |
| 1 | Position fix valid |
| 2 | GPS timepulse locked (a pulse on PA0 in the last 1.5 s) |
| 3-5 | GPS state: 1 no data, 2 searching, 3 few sats (1-3 heard), 4 acquiring (4+ heard, no fix), 5 fix |
| 6-7 | Reserved, 0 |

"Heard" means the GPS gives a signal strength for that satellite in its GSV sentences. State 0 means
"not reported", which is what the perfboard nodes send. Example flag values: `0x11` = time + searching,
`0x19` = time + few sats, `0x21` = time + acquiring, `0x2F` = time + fix + PPS.

receiver.py shows the state on the dashboard (`GPS  no fix - few sats`, `sync GPS+PPS`), logs each
change as an event (`GPS few sats -> acquiring`), and adds `gps_state` and `pps_lock` columns to
`sensor_log.csv`. If an existing `sensor_log.csv` has the old columns, it is renamed to
`sensor_log_oldcols_<date>.csv` and a new file is started.

Compile-checked 21 Oct 2026: RAKwireless RUI STM32 Boards 4.3.0, TinyGPSPlus 1.0.3,
boards "WisDuo RAK3172 Evaluation Board" and "WisDuo RAK3172-T Board". Not yet tested on hardware.
The GPS-state flags (gps_status.h, 21 Oct 2026) were checked with a host compiler and unit tests,
not yet with the RAK toolchain or on air. Compile in Arduino IDE before relying on them.

## Bench test sketch (no antenna needed)

`firefly_nano_test/firefly_nano_test.ino` **never transmits**, so it is safe with nothing on the
U.FL connector. Transmitting without an antenna can damage the RAK3172's radio, so use this
sketch until an antenna is fitted. It uses the same sensors, GPS, timing and packet code, and prints
everything to the Serial Monitor (115200) instead:

```
R  12.3  23.41  51.2  -27.561234  151.953456  1  | 8  1.1  04:12:09  GPS+PPS
PKT seq 12  slot 2 ms  128 B  flags 0x2F  GPS FIX, 8 heard, PPS  [GPS+PPS]  (not transmitted)
    FF 01 0C 00 01 0C 00 03 ...
[status] up 120 s  GPS FIX  heard 9  used 8  FIX  time GPS+PPS  ...  nmea ok/bad 812/0  sht ok/err 1200/0
```

At boot it also runs an I2C scan and reads back the radio settings (it configures the radio
but never sends). Options at the top: `PRINT_READINGS`, `PRINT_PACKETS`, `ECHO_NMEA`
(raw GPS sentences), `CHECK_RADIO`.

## GPS diagnostic sketch

`firefly_nano_gps_test/firefly_nano_gps_test.ino` (never uses the LoRa radio) prints a report every 2 s:

```
GPS -> RAK (PB7): OK   bytes 5120  NMEA ok/bad 160/0
RAK -> GPS (PB6): OK (GPS answered our commands)
GPS model: MAX-M10S   firmware: ROM SPG 5.10 ...
Fix: no   satellites used 0   HDOP -   timepulses 0
RF: noise 82   AGC gain 64%   jamming none   antenna unknown (...)
Satellites reported 9   heard 4   >=25 dB-Hz 2   strongest 31 dB-Hz
  SYS    PRN  ELEV  AZIM  dB-Hz
  GPS     12    45   120     31  ###############
  GAL      7    12   300      -  (not heard)
Hint: satellites heard but weak/few - go outside with open sky ...
```

Signal: <20 too weak, 20-30 weak, 30-40 good, >40 excellent dB-Hz. A fix needs about 4
satellites at 25+. The LED blinks every report while searching and stays on once there is a fix.

## Flashing with Arduino IDE

1. **File → Preferences → Additional boards manager URLs**, add:
   `https://raw.githubusercontent.com/RAKWireless/RAKwireless-Arduino-BSP-Index/main/package_rakwireless.com_rui_index.json`
2. **Boards Manager**: install **RAKwireless RUI STM32 Boards**.
3. **Library Manager**: install **TinyGPSPlus** (Mikal Hart). No other libraries are needed.
4. **Tools → Board**: **WisDuo RAK3172 Evaluation Board**.
   If the module label says **RAK3172-T** (TCXO version), pick **WisDuo RAK3172-T Board** instead.
   The wrong choice compiles fine but the radio may not transmit.
5. Wire a **3.3 V** USB-serial adapter to the test pads:

   | Adapter | Nano pad |
   |---|---|
   | TX | UART_2_RX (PA3) |
   | RX | UART_2_TX (PA2) |
   | GND | GND |

   Power the board from the battery or the 3V3 pad. Never connect 5 V.
6. Set `NODE_ID` at the top of the sketch (1 or 2), pick the adapter's COM port, and click **Upload**.
7. Open **Serial Monitor at 115200**.

On the very first boot the board switches itself from LoRaWAN to LoRa P2P mode and
reboots once. That's expected.

If the upload never starts: RAK3172 modules ship with the RUI3 bootloader, which Arduino IDE
uses over UART2. A module flashed with other firmware needs RUI3 put back first with
STM32CubeProgrammer (RAK's "RAK3172 bootloader" guide).

## What you should see

```
Fireflies Nano TX [RAK3172]  -  NODE 1
...
Radio read-back: 915000000 Hz  SF7  BW125  CRcode 0  pre 8  14 dBm  sync 0x1424
GPS: Serial1 (PB6/PB7) @ 9600
SHT4x: ready at 0x44
TX seq 0  slot 3 ms  128 B  flags 0x10  GPS searching, 0 heard  [free]
  TxDone 216 ms   ok=1 err=0 missed=0
[status] GPS FIX  heard 9  used 7  fix -27.5,152.9  time OK  pps 41  nmea 900/bad 0  T 23.4 C  RH 51 %  sht_err 0
```

`[free]` means no GPS time yet (slots run on the internal clock). `[GPS+PPS]` means slots are
locked to the GPS timepulse.

**LED (PA9):** on for 2 s at boot, then flashes during each transmission. A fast steady blink
means it stopped on an error. The Serial Monitor says which one.

## Settings at the top of the sketch

| Setting | Default | Notes |
|---|---|---|
| `NODE_ID` | 1 | 1 or 2. For 3+ nodes, also add them to `ALLOWED_NODES` in receiver.py and check slot timing. |
| `LORA_POWER_DBM` | 14 | 5–22 dBm. See the power warning below. |
| `CYCLE_MS` | 1000 | Must be the same on every node. 5000 = one packet per 5 s. |
| `USE_PPS` | 1 | Use the GPS timepulse (PA0) for exact slot timing. |
| `LED_ON_TX` | 1 | Set 0 to save power. |

## Differences from the perfboard firmware

- Radio: RAK's RUI3 `api.lora` calls instead of RadioLib + RFM95W. The SX1276 sync word 0x12
  becomes 0x1424 on this radio. It's the same on air.
- SHT4x read directly over I2C (no Adafruit library). It tries addresses 0x44 and 0x45.
- If the sensor is missing or a read fails, the reading is sent as −3276.8 °C / 255 %.
  receiver.py keeps the row and marks it `TEMP|RH`, instead of logging a fake 0 °C / 0 %.
- GPS timepulse on PA0 gives exact UTC second edges. Perfboard nodes time slots from NMEA
  arrival, which runs slightly late. That eats into the 285 ms guard time, but should still
  fit. If Nano and perfboard packets collide, set `USE_PPS 0` on the Nano.
- A status line every 10 s (satellites, fix, time sync, sensor).

## Hardware notes from the schematic (affect field use)

- **Coin-cell power.** The board runs from a CR2450 through the TLV75533 regulator. A CR2450
  can only supply a few mA continuously. The MAX-M10S takes ~25 mA while searching for
  satellites, and the RAK3172 ~45 mA at 14 dBm (90–120 mA at 20–22 dBm) while transmitting.
  Expect resets or a quickly drained cell. Use a bench supply or LiPo on the BATT+ pad for testing.
- **GPS V_BCKP isn't connected**, so the GPS starts cold after every power-up
  (roughly 30 s+ to first fix outdoors).
