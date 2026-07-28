// limeskey-node — custom handheld Meshtastic node
// MCU : Seeed XIAO ESP32-C6 (castellated SMD module, 4 MB flash)
// LoRa: EBYTE E22P-915M30S (SX1262 + 30 dBm PA), SPI
// GNSS: u-blox NEO-M9N, SPI (D_SEL strapped to GND) — see GNSS note below
//
// Community/DIY board — no dedicated Meshtastic HardwareModel, reports as PRIVATE_HW.
#define PRIVATE_HW

// Selects src/platform/extra_variants/limeskey_node/variant.cpp (earlyInitVariant).
#define _VARIANT_LIMESKEY_NODE

// ---------------------------------------------------------------------------
// I2C  (optional / unpopulated by default)
// ---------------------------------------------------------------------------
// GPIO22/23 are free on this design and broken out for a future I2C peripheral
// (OLED, sensor, or the NEO-M9N I2C-bodge endgame). Nothing is required here for
// LoRa to work; the boot-time I2C scan simply finds nothing if unpopulated.
#define I2C_SDA 22
#define I2C_SCL 23

// ---------------------------------------------------------------------------
// Status LED
// ---------------------------------------------------------------------------
// Seeed XIAO ESP32-C6 onboard user LED is on GPIO15, active LOW.
// (Verify against your board; toggling an unused pin is harmless if it differs.)
#define LED_POWER 15
#define LED_STATE_ON 0 // LED lit when pin driven LOW

// ---------------------------------------------------------------------------
// Battery sense — 18650 through a 1:2 divider into GPIO0 (XIAO pad A0)
// ---------------------------------------------------------------------------
// BAT+ ──100k──┬── GPIO0 ──┐
//              100k       100nF     (both to GND)
// The ESP32-C6 has a single ADC unit, and GPIO0 is ADC1_CH0, so
// BAT_MEASURE_ADC_UNIT stays undefined (Power.cpp defaults to ADC_UNIT_1).
#define BATTERY_PIN 0
#define ADC_CHANNEL ADC_CHANNEL_0

// 100k series + 100k to GND = 1:2. This matches Power.cpp's fallback, but is
// stated here so the divider is documented where the resistors are described.
// Trim against a DMM via the runtime config.power.adc_multiplier_override
// (no reflash) before changing this number.
#define ADC_MULTIPLIER 2.0457

// ADC_ATTENUATION is deliberately left undefined: Power.cpp defaults to
// ADC_ATTEN_DB_12, usable to ~3.1 V on the C6, and a full 4.2 V cell reads
// 2.1 V here. Do NOT copy the ADC_ATTEN_DB_2_5 other variants use for their
// higher-ratio dividers — it saturates near 1.1 V and would clip most of the
// discharge curve. There is no ADC_CTRL switch: the divider draws ~21 uA
// continuously (~0.5 mAh/day, negligible against an 18650).

// ---------------------------------------------------------------------------
// LoRa — EBYTE E22P-915M30S (SX1262) on the default SPI bus
// ---------------------------------------------------------------------------
//   SX1262 fn   E22P pin   net            ESP32-C6 GPIO
//   NSS         19         /CS_LORA        21
//   SCK         18         /SCK            19
//   MOSI        17         /MOSI           18   (shared with NEO-M9N)
//   MISO        16         /MISO           20   (shared with NEO-M9N)
//   DIO1 (IRQ)  13         /DIO1_LORA       1
//   BUSY        14         /BUSY_LORA       7
//   NRST        15         /NRST_LORA      16
//   EN (master)  6         /EN_LORA        17   held HIGH while radio is active
//   T/R CTRL     7 (=DIO2 via R21)              SX1262 drives the RF switch
//   TCXO        DIO3                            1.8 V reference (per E22 manual)
#define USE_SX1262

#define LORA_SCK 19
#define LORA_MISO 20
#define LORA_MOSI 18
#define LORA_CS 21
#define LORA_DIO1 1
#define LORA_RESET 16

#define SX126X_CS LORA_CS
#define SX126X_DIO1 LORA_DIO1
#define SX126X_BUSY 7
#define SX126X_RESET LORA_RESET

// E22P master enable (module pin 6). Driven HIGH at radio init, LOW on sleep.
#define SX126X_POWER_EN 17

// The E22P routes its T/R control to the SX1262 DIO2 (via R21), so the SX1262
// switches the PA/LNA itself — no separate RXEN/TXEN MCU pins on this board.
#define SX126X_DIO2_AS_RF_SWITCH

// TCXO is fed from DIO3 at 1.8 V.
#define SX126X_DIO3_TCXO_VOLTAGE 1.8

// PA gain / max power for the E22(P)-900M30S family is set in platformio.ini via
// -D EBYTE_E22_900M30S (TX_GAIN_LORA=7, SX126X_MAX_POWER=22 dBm at the SX1262).

// ---------------------------------------------------------------------------
// GNSS — u-blox NEO-M9N on the shared SPI bus (custom SPI driver)
// ---------------------------------------------------------------------------
// Mainline Meshtastic GPS is UART-only, so this board adds a SPI Stream adapter
// (src/gps/UBloxSPIGNSS.{h,cpp}) that routes the standard GPS stack over SPI.
// GPS_SPI_INTERFACE swaps GPS::_serial_gps to the adapter; GPS_SERIAL_PORT names
// its instance. The adapter shares the LoRa SPI bus and arbitrates with the
// SX1262 through the global spiLock.
#define GPS_SPI_INTERFACE
#define GPS_SERIAL_PORT ubloxSPIGNSS

// NEO-M9N chip select (its own line; SCK/MOSI/MISO are shared with the SX1262).
// Also parked HIGH at boot in earlyInitVariant() before SPI.begin().
#define GPS_SPI_CS_PIN 2

// createGps() only probes when both rx/tx "pins" are non-zero. These name the
// shared SPI data lines for documentation; the SPI adapter ignores them (it does
// NOT reconfigure them as a UART), so the SPI bus keeps these pins.
#define HAS_GPS 1
#define GPS_RX_PIN 20 // NEO MISO (shared w/ SX1262) — logical only, unused by SPI driver
#define GPS_TX_PIN 18 // NEO MOSI (shared w/ SX1262) — logical only, unused by SPI driver

// Optional SPI tuning (defaults in UBloxSPIGNSS.h):
// #define GPS_SPI_HZ 1000000UL   // NEO-M9N SPI max is 5.5 MHz
// #define GPS_SPI_IDLE_RUN 4     // raise if relying on binary UBX with 0xFF-heavy payloads

#define SERIAL_PRINT_PORT 1
