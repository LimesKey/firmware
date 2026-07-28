#pragma once

#include "configuration.h"

#ifdef LIMESKEY_DIAG

#include <stdint.h>

/**
 * limeskey_node field diagnostics.
 *
 * Emits self-describing `LKD:` lines through the normal LOG_* macros, so they
 * ride the standard log_record path and land in the stock Meshtastic phone app's
 * debug log (over BLE, gated by config.security.debug_log_api_enabled) as well as
 * the USB console. The intent is that a walk/ride log can be handed to a tool or
 * an LLM afterwards and diagnose PCB, antenna, bus and firmware problems without
 * anyone having to remember what each number meant.
 *
 * Format contract, relied on by anything parsing these:
 *   - Every line starts with the literal "LKD:" followed by a section name.
 *   - The rest of the line is space-separated key=value pairs, no spaces inside
 *     a value, no quoting. Missing/unknown values are omitted, never blanked.
 *   - Sections repeat on a fixed cadence; "LKD:evt" lines are event-driven.
 *
 * Cost control: one line per tick rather than a burst, because the BLE log path
 * shares the NimBLE notify pool with fromNum notifications (see the backoff in
 * NimbleBluetooth::sendLog). All state here is static; nothing on this path
 * allocates.
 */
namespace limeskeydiag
{

/**
 * Feed one byte clocked off the u-blox SPI port to the UBX sniffer.
 *
 * Called from UBloxSPIGNSS on the GPS thread for every byte in both directions.
 * Purely passive: it reassembles UBX frames as they flow past to GPS.cpp's NMEA
 * parser (which discards them), verifies their checksums, and keeps the fields we
 * report. Nothing here drives the bus, so it cannot contend with the GPS thread
 * for the ring buffer or with the SX1262 for spiLock.
 */
void ubxTapFeed(uint8_t b);

/// Create the diagnostics thread. Call once from setupModules().
void setup();

} // namespace limeskeydiag

/**
 * Report the deepest the NimBLE to-phone queue has been since boot.
 *
 * Defined in NimbleBluetooth.cpp, where the queue lives; returns 0 on any build
 * without the NimBLE stack. The queue depth itself is MAX_RX_TOPHONE.
 */
uint32_t limeskeyBleToPhoneHighWater();

#endif // LIMESKEY_DIAG
