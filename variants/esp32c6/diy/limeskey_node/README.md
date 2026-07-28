# limeskey-node — ESP32-C6 + E22P-915M30S + NEO-M9N

A custom, battery-powered handheld Meshtastic node built around the **Seeed XIAO ESP32-C6**, an **EBYTE E22P-915M30S** (SX1262 + 30 dBm PA) LoRa module, and a **u-blox NEO-M9N** GNSS receiver. Reports to the mesh as `PRIVATE_HW`.

This is a community/DIY variant — there is no dedicated Meshtastic `HardwareModel` for this hardware combination.

---

## Required hardware

| Component | Part                | Notes                                                  |
| --------- | ------------------- | ------------------------------------------------------ |
| MCU       | Seeed XIAO ESP32-C6 | Castellated SMD module, 4 MB flash, native USB         |
| LoRa      | EBYTE E22P-915M30S  | SX1262 + 30 dBm PA, 915 MHz; SPI; single master-enable |
| GNSS      | u-blox NEO-M9N      | SPI mode (D_SEL → GND) — see GNSS note below           |

---

## Pinout (ESP32-C6 GPIO)

### LoRa — E22P-915M30S (SX1262), default SPI bus

| SX1262 fn      | E22P pin        | GPIO | Define                |
| -------------- | --------------- | ---- | --------------------- |
| NSS / CS       | 19              | 21   | `LORA_CS`             |
| SCK            | 18              | 19   | `LORA_SCK`            |
| MOSI           | 17              | 18   | `LORA_MOSI`           |
| MISO           | 16              | 20   | `LORA_MISO`           |
| DIO1 (IRQ)     | 13              | 1    | `SX126X_DIO1`         |
| BUSY           | 14              | 7    | `SX126X_BUSY`         |
| NRST           | 15              | 16   | `SX126X_RESET`        |
| EN (master)    | 6               | 17   | `SX126X_POWER_EN`     |
| T/R control    | 7 (= DIO2, R21) | —    | `SX126X_DIO2_AS_RF_SWITCH` |
| TCXO           | DIO3            | —    | `SX126X_DIO3_TCXO_VOLTAGE 1.8` |

**RF switching:** the E22P routes its T/R control to the SX1262's DIO2 (via R21), so the radio toggles the PA/LNA itself. There are **no** `SX126X_RXEN`/`SX126X_TXEN` MCU pins on this board.

**Module enable:** `SX126X_POWER_EN` (GPIO17 → E22P pin 6) is driven HIGH at radio init and LOW on sleep, so the module is powered only while the radio is in use.

**Power / PA:** `-D EBYTE_E22_900M30S` (in `platformio.ini`) selects the E22(P)-900M30S PA profile: `TX_GAIN_LORA = 7`, `SX126X_MAX_POWER = 22` dBm at the SX1262 (~30 dBm at the antenna after the PA).

### GNSS — NEO-M9N (SPI, GPIO2)

| Signal | GPIO | Note                       |
| ------ | ---- | -------------------------- |
| CS     | 2    | Parked HIGH at boot only   |
| SCK    | 19   | shared with LoRa           |
| MOSI   | 18   | shared with LoRa           |
| MISO   | 20   | shared with LoRa           |

### Battery sense — GPIO0

| Signal        | GPIO | Define                     |
| ------------- | ---- | -------------------------- |
| BAT divider   | 0    | `BATTERY_PIN` / `ADC_CHANNEL_0` |

18650 positive through 100k, 100k to GND, with a 100nF at the ADC node to give the SAR sample-and-hold something low-impedance to charge from (50k Thévenin on its own reads low). `ADC_MULTIPLIER 2.0` for the 1:2 ratio. Default 12 dB attenuation covers the 1.5–2.1 V the divider produces across the cell's usable range.

### I2C — optional / unpopulated

| Signal | GPIO |
| ------ | ---- |
| SDA    | 22   |
| SCL    | 23   |

Free pins, broken out for a future I2C peripheral (OLED, sensor, or an I2C-mode GNSS bodge). Nothing is required here for LoRa to work.

---

## GNSS over SPI (custom driver)

Mainline Meshtastic GPS is **UART-only** — the stack talks to the receiver through `_serial_gps` (`src/gps/GPS.h`), a `HardwareSerial`. There is no SPI or I2C GPS reader in mainline. This board adds one.

`src/gps/UBloxSPIGNSS.{h,cpp}` is a small `Stream` adapter that carries the u-blox SPI byte stream (clock `0xFF` to read; a run of `0xFF` = "no data"; clock command bytes to write). Because GPS.cpp only ever touches the port through `Stream`/`Print` methods plus a few `HardwareSerial`-shaped control calls, the adapter stands in for a serial port with **two** small guarded edits to the core (the `_serial_gps` pointer type in `GPS.h`/`GPS.cpp`). `GPS_SPI_INTERFACE` (set in `variant.h`) selects it.

**Shared bus.** The GNSS and SX1262 share SCK/MOSI/MISO. Every GNSS transfer is wrapped in the global `spiLock` — the same lock RadioLib's `LockingArduinoHal` takes — and is bounded to `GPS_SPI_DRAIN_CHUNK` bytes so the radio is never blocked for long. Each device keeps its own CS parked HIGH when idle, so they never both drive MISO. `earlyInitVariant()` parks the NEO's CS HIGH before the first `SPI.begin()`; the adapter manages CS for its own transactions thereafter.

**SPI port output.** The NEO-M9N is detected as `GNSS_MODEL_UBLOX9`, which configures NMEA with the legacy CFG-MSG packets in `ubx.h`. Those enable RMC/GGA on UART1 only and set the **SPI port rate to 0** (and `SAVE`), which would silence the SPI port. The `UBX_NMEA_SPI_RATE` macro (guarded by `GPS_SPI_INTERFACE`) flips the SPI rate to `1` for RMC + GGA — **only** on this board; all other variants are byte-for-byte unchanged.

### Bring-up checklist

1. **D_SEL must be strapped to GND** (SPI mode) at power-on — the NEO latches its interface at boot.
2. Confirm the boot log shows `u-blox SPI GNSS ready (...)` then a `GNSS_MODEL_UBLOX9` probe success. The probe (UBX-MON-VER) runs over SPI unchanged.
3. If no fix appears, verify NMEA is reaching the SPI port — the firmware now sets RMC/GGA SPI rate = 1, but a `SAVE`d prior config can interfere; re-flashing the module's config in u-center (or letting the firmware re-`SAVE`) resolves it.
4. **Binary UBX caveat:** the drain treats a run of `GPS_SPI_IDLE_RUN` (default 4) `0xFF` bytes as "buffer empty". `0xFF` is valid inside UBX payloads (e.g. `0xFFFFFFFF`), so heavy binary UBX can truncate. NMEA is ASCII (no `0xFF`), so the position path is fine; raise `GPS_SPI_IDLE_RUN` in `variant.h` if you rely on binary UBX.
5. **Tuning:** `GPS_SPI_HZ` (default 1 MHz; NEO-M9N max 5.5 MHz), `GPS_SPI_DRAIN_CHUNK`, `GPS_SPI_RINGBUF` — all overridable in `variant.h`.

> Alternative (no firmware): re-strap `D_SEL` to UART, wire NEO **TXD → a free C6 GPIO**, set `GPS_RX_PIN`. RX-only works (`m5stack_coreink`: "GPS works with just RX"). Keeps zero custom code but abandons SPI mode.

---

## Build

```bash
pio run -e limeskey_node
```

## Flash (USB, native CDC)

The XIAO ESP32-C6 enumerates as a USB CDC serial port:

```bash
pio run -e limeskey_node -t upload
```

If the port is not detected, enter the ROM bootloader: hold **BOOT**, tap **RESET**, release **BOOT**, then upload.

---

## Region / regulatory

Operating frequency and TX power are runtime settings, not compiled in. Set your region in the Meshtastic app (e.g. **US / 915 MHz**). For **ISED RSS-247 (Canada)** compliance, set the LoRa region and cap TX power to stay within the EIRP limit for your antenna gain — the `-D EBYTE_E22_900M30S` PA profile already limits the SX1262 to 22 dBm, but the external PA brings this to ~30 dBm at the antenna port.

---

## Notes

- **Screen + audio excluded** (`MESHTASTIC_EXCLUDE_SCREEN`, `MESHTASTIC_EXCLUDE_AUDIO`) — the 4 MB flash on the ESP32-C6 is tight, matching `tlora_c6`.
- **Bluetooth** works on the ESP32-C6 (NimBLE) — the phone app pairs over BLE. Note that enabling Wi-Fi disables BLE until reboot (the two radios don't run concurrently here), so you configure over BLE *or* Wi-Fi/USB, not both at once. The BLE stack is torn down on that switch; see the teardown-race fix in `src/nimble/NimbleBluetooth.cpp` (a long-polling `FromRadio` read used to write into a freed characteristic and corrupt the heap).
- **Battery:** monitored on GPIO0 through a 100k/100k divider (see the pinout section). `ADC_MULTIPLIER` is the nominal 2.0; resistor tolerance and ADC offset mean the reported voltage will be a little off until trimmed. Compare against a DMM on the cell and correct with the runtime `config.power.adc_multiplier_override` power setting rather than editing `variant.h`, so the calibration is per-board and survives a reflash.
