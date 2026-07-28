#pragma once

#include "configuration.h"

#include <stdint.h>

// Most bytes ubxTapFeed() can hand back from one call: a UBX header it held on to
// while deciding whether the frame was one of its own (sync1, sync2, class, id,
// length low, length high).
#define LIMESKEY_UBX_TAP_MAX_RELEASE 6

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

#ifdef LIMESKEY_DIAG

/**
 * Feed one byte clocked off the u-blox SPI port to the UBX sniffer, and get back
 * the bytes (if any) that should still reach the NMEA parser.
 *
 * Called from UBloxSPIGNSS on the GPS thread for every byte in both directions.
 * The tap never drives the bus: it reassembles UBX frames as they flow past,
 * verifies their checksums, and keeps the fields we report.
 *
 * It also acts as a filter. The five messages enabled for diagnostics are
 * withheld entirely, because TinyGPS++ would otherwise see their payload bytes
 * and treat any 0x24 in them as the '$' starting an NMEA sentence, producing a
 * steady trickle of bogus "GPS checksum failures". Every other frame is released
 * byte-for-byte unchanged. That distinction matters: ACK/NAK (class 0x05) and
 * MON-VER are parsed by GPS::getACK(), and swallowing those would silently break
 * GNSS configuration and probing.
 *
 * @param b   the byte just clocked off the module
 * @param out buffer of at least LIMESKEY_UBX_TAP_MAX_RELEASE bytes
 * @return    how many bytes of `out` to buffer for the NMEA parser; 0 means the
 *            tap took the byte. A count above 1 replays a header it had held.
 */
uint8_t ubxTapFeed(uint8_t b, uint8_t *out);

/// True while the tap is part-way through a UBX frame. A 0xFF is payload then,
/// not the module's "nothing to send" filler, so the caller must not count it
/// toward an idle run.
bool ubxTapInFrame();

/// Create the diagnostics thread. Call once from setupModules().
void setup();

#else

// No-op stubs, so the GNSS adapter's hot path needs no #ifdefs (same approach as
// memory/MemAudit.h). Every byte is passed straight through.
inline uint8_t ubxTapFeed(uint8_t b, uint8_t *out)
{
    out[0] = b;
    return 1;
}
inline bool ubxTapInFrame()
{
    return false;
}
inline void setup() {}

#endif // LIMESKEY_DIAG

} // namespace limeskeydiag

#ifdef LIMESKEY_DIAG
/**
 * Report the deepest the NimBLE to-phone queue has been since boot.
 *
 * Defined in NimbleBluetooth.cpp, where the queue lives; returns 0 on any build
 * without the NimBLE stack. The queue depth itself is MAX_RX_TOPHONE.
 */
uint32_t limeskeyBleToPhoneHighWater();
#endif
