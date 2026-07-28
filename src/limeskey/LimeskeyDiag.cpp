#include "limeskey/LimeskeyDiag.h"

#ifdef LIMESKEY_DIAG

#include "NodeDB.h"
#include "airtime.h"
#include "concurrency/OSThread.h"
#include "gps/UBloxSPIGNSS.h"
#include "main.h"
#include "memGet.h"
#include "memory/MemAudit.h"
#include "mesh/RadioLibInterface.h"
#include "mesh/mesh-pb-constants.h"
#include <stdio.h>
#include <string.h>

#if !MESHTASTIC_EXCLUDE_BLUETOOTH && defined(ARCH_ESP32)
#include "nimble/NimbleBluetooth.h"
#endif

#ifdef ARCH_ESP32
#include <esp_system.h>
#endif

#if HAS_WIFI && !defined(MESHTASTIC_EXCLUDE_WIFI)
#include <WiFi.h>
#endif

// How often a full summary sweep starts. The sweep itself emits one section per
// tick (kLineSpacingMs apart) so a burst of lines can't starve the NimBLE notify
// pool that fromNum notifications share.
#ifndef LIMESKEY_DIAG_PERIOD_MS
#define LIMESKEY_DIAG_PERIOD_MS (30 * 1000)
#endif
#ifndef LIMESKEY_DIAG_LINE_SPACING_MS
#define LIMESKEY_DIAG_LINE_SPACING_MS 250
#endif

namespace limeskeydiag
{

// ---------------------------------------------------------------------------
// UBX sniffer
// ---------------------------------------------------------------------------
// Frames are reassembled from the byte stream the NEO-M9N pushes out on its SPI
// port. We never poll: GPS.cpp configures the module (once, at setup) to emit
// these messages on a slow cadence, and we read them off the wire as they pass.
// That keeps this side completely passive - no second producer on the adapter's
// ring buffer, no extra spiLock traffic competing with the SX1262.

// Largest payload we keep. NAV-SAT is 8 + 12*numSvs; 512 covers 42 satellites,
// well past what an M9N reports. Longer frames are checksummed and counted but
// not decoded (see stats.ubxOversize).
static constexpr uint16_t kMaxPayload = 512;

enum : uint16_t {
    kNavStatus = 0x0103,
    kNavDop = 0x0104,
    kNavSat = 0x0135,
    kMonHw = 0x0A09,
    kMonRf = 0x0A38,
};

/// Everything the sniffer has extracted, plus when. Scalars only, so a torn read
/// from the diag thread costs at most one stale field on one line.
struct UbxState {
    // NAV-SAT
    uint8_t numSvs, numUsed;
    uint8_t topCno[4];      // four strongest C/N0, descending, 0 = no such sat
    uint8_t seenPerGnss[7]; // indexed by gnssId: GPS,SBAS,GAL,BDS,IMES,QZSS,GLO
    uint8_t usedPerGnss[7];
    uint32_t navSatMs;

    // NAV-DOP, in hundredths (u-blox native scaling)
    uint16_t pDop, hDop, vDop;
    uint32_t navDopMs;

    // NAV-STATUS
    uint8_t gpsFix, fixFlags;
    uint32_t ttffMs, msssMs;
    uint32_t navStatusMs;

    // MON-RF, block 0 (the M9N has one RF block per antenna input)
    uint8_t rfBlocks, rfJamState, rfJamInd, rfAntStatus, rfAntPower;
    uint16_t rfNoise, rfAgc;
    int8_t rfOfsI, rfOfsQ;
    uint32_t monRfMs;

    // MON-HW
    uint16_t hwNoise, hwAgc;
    uint8_t hwAntStatus, hwAntPower, hwJamInd, hwFlags;
    uint32_t monHwMs;
};

struct UbxStats {
    uint32_t framesOk;    // checksum verified
    uint32_t cksumBad;    // checksum mismatch => SPI signal integrity suspect
    uint32_t oversize;    // payload longer than kMaxPayload, skipped
    uint32_t framesSeen;  // sync pairs that reached a length field
};

static UbxState state;
static UbxStats stats;

// Parser state machine. Bytes arrive one at a time from the GPS thread.
static struct {
    uint8_t phase; // 0=idle 1=saw B5 2=class 3=id 4=len_lo 5=len_hi 6=payload 7=ckA 8=ckB
    uint8_t cls, id;
    uint16_t len, got;
    uint8_t ckA, ckB;
    bool keep;  // buffering this payload (interesting and it fits)
    bool strip; // withholding this frame from the NMEA parser
    uint8_t buf[kMaxPayload];
} p;

static inline uint16_t rdU16(const uint8_t *b, uint16_t o)
{
    return (uint16_t)b[o] | ((uint16_t)b[o + 1] << 8);
}
static inline uint32_t rdU32(const uint8_t *b, uint16_t o)
{
    return (uint32_t)b[o] | ((uint32_t)b[o + 1] << 8) | ((uint32_t)b[o + 2] << 16) | ((uint32_t)b[o + 3] << 24);
}

static void decodeNavSat()
{
    const uint8_t n = p.buf[5];
    if ((uint32_t)8 + 12u * n > p.len)
        return; // truncated; don't walk off the buffer

    uint8_t used = 0;
    uint8_t top[4] = {0, 0, 0, 0};
    uint8_t seen[7] = {0}, usedBy[7] = {0};

    for (uint8_t i = 0; i < n; i++) {
        const uint8_t *sv = &p.buf[8 + 12 * i];
        const uint8_t gnssId = sv[0];
        const uint8_t cno = sv[2];
        const uint32_t flags = rdU32(sv, 8);
        const bool svUsed = flags & 0x08;

        if (gnssId < 7) {
            seen[gnssId]++;
            if (svUsed)
                usedBy[gnssId]++;
        }
        if (svUsed)
            used++;

        // Insertion sort into the descending top-4.
        for (uint8_t k = 0; k < 4; k++) {
            if (cno > top[k]) {
                for (uint8_t j = 3; j > k; j--)
                    top[j] = top[j - 1];
                top[k] = cno;
                break;
            }
        }
    }

    state.numSvs = n;
    state.numUsed = used;
    memcpy(state.topCno, top, sizeof(top));
    memcpy(state.seenPerGnss, seen, sizeof(seen));
    memcpy(state.usedPerGnss, usedBy, sizeof(usedBy));
    state.navSatMs = millis();
}

static void decodeFrame()
{
    switch ((uint16_t)((uint16_t)p.cls << 8 | p.id)) {
    case kNavSat:
        if (p.len >= 8)
            decodeNavSat();
        break;

    case kNavDop:
        if (p.len >= 18) {
            state.pDop = rdU16(p.buf, 6);
            state.vDop = rdU16(p.buf, 10);
            state.hDop = rdU16(p.buf, 12);
            state.navDopMs = millis();
        }
        break;

    case kNavStatus:
        if (p.len >= 16) {
            state.gpsFix = p.buf[4];
            state.fixFlags = p.buf[5];
            state.ttffMs = rdU32(p.buf, 8);
            state.msssMs = rdU32(p.buf, 12);
            state.navStatusMs = millis();
        }
        break;

    case kMonRf:
        // version(1) nBlocks(1) reserved(2), then 24 bytes per block.
        if (p.len >= 4 + 24) {
            state.rfBlocks = p.buf[1];
            const uint8_t *b = &p.buf[4];
            state.rfJamState = b[1] & 0x03;
            state.rfAntStatus = b[2];
            state.rfAntPower = b[3];
            state.rfNoise = rdU16(b, 12);
            state.rfAgc = rdU16(b, 14);
            state.rfJamInd = b[16];
            state.rfOfsI = (int8_t)b[17];
            state.rfOfsQ = (int8_t)b[19];
            state.monRfMs = millis();
        }
        break;

    case kMonHw:
        if (p.len >= 60) {
            state.hwNoise = rdU16(p.buf, 16);
            state.hwAgc = rdU16(p.buf, 18);
            state.hwAntStatus = p.buf[20];
            state.hwAntPower = p.buf[21];
            state.hwFlags = p.buf[22];
            state.hwJamInd = p.buf[45];
            state.monHwMs = millis();
        }
        break;

    default:
        break;
    }
}

static inline bool interesting(uint8_t cls, uint8_t id)
{
    const uint16_t k = (uint16_t)cls << 8 | id;
    return k == kNavSat || k == kNavDop || k == kNavStatus || k == kMonRf || k == kMonHw;
}

bool ubxTapInFrame()
{
    return p.phase != 0;
}

// Helper for the phases that pass a frame through: hand the byte back unless
// this is one of our own frames, which is swallowed.
static inline uint8_t passThrough(uint8_t b, uint8_t *out)
{
    if (p.strip)
        return 0;
    out[0] = b;
    return 1;
}

uint8_t ubxTapFeed(uint8_t b, uint8_t *out)
{
    switch (p.phase) {
    case 0:
        if (b == 0xB5) {
            p.phase = 1; // might be a sync pair; hold it until we know
            return 0;
        }
        out[0] = b;
        return 1;

    case 1:
        if (b == 0x62) {
            p.phase = 2;
            return 0;
        }
        if (b == 0xB5)
            return 0; // stay armed on the second 0xB5 rather than miss a real sync pair
        // Not a frame after all. Release the byte we were holding along with this
        // one, so a stray 0xB5 in the stream costs the NMEA parser nothing.
        p.phase = 0;
        out[0] = 0xB5;
        out[1] = b;
        return 2;

    case 2:
        p.cls = b;
        p.ckA = b;
        p.ckB = b;
        p.phase = 3;
        return 0;

    case 3:
        p.id = b;
        p.ckA += b;
        p.ckB += p.ckA;
        p.phase = 4;
        return 0;

    case 4:
        p.len = b;
        p.ckA += b;
        p.ckB += p.ckA;
        p.phase = 5;
        return 0;

    case 5: {
        p.len |= (uint16_t)b << 8;
        p.ckA += b;
        p.ckB += p.ckA;
        p.got = 0;
        stats.framesSeen++;

        // Only the messages we asked the module for are withheld. Anything else,
        // above all ACK/NAK and MON-VER, has to reach GPS::getACK() untouched or
        // GNSS configuration and probing silently stop working.
        const bool mine = interesting(p.cls, p.id);
        p.strip = mine;
        p.keep = mine && p.len <= kMaxPayload;
        if (mine && p.len > kMaxPayload)
            stats.oversize++;
        p.phase = p.len ? 6 : 7;

        if (p.strip)
            return 0;
        // Replay the header we had held back while deciding.
        out[0] = 0xB5;
        out[1] = 0x62;
        out[2] = p.cls;
        out[3] = p.id;
        out[4] = (uint8_t)(p.len & 0xFF);
        out[5] = b;
        return 6;
    }

    case 6:
        p.ckA += b;
        p.ckB += p.ckA;
        if (p.keep)
            p.buf[p.got] = b;
        if (++p.got >= p.len)
            p.phase = 7;
        return passThrough(b, out);

    case 7:
        // Checksum mismatch here is the SPI signal-integrity signal: the frame
        // was well-formed enough to reach this point but the bytes are corrupt.
        if (b != p.ckA) {
            stats.cksumBad++;
            p.phase = 0;
        } else {
            p.phase = 8;
        }
        return passThrough(b, out);

    case 8:
    default:
        if (b == p.ckB) {
            stats.framesOk++;
            if (p.keep)
                decodeFrame();
        } else {
            stats.cksumBad++;
        }
        p.phase = 0;
        return passThrough(b, out);
    }
}

// ---------------------------------------------------------------------------
// Reporting thread
// ---------------------------------------------------------------------------

#ifdef ARCH_ESP32
static const char *resetReasonName()
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON:
        return "poweron";
    case ESP_RST_SW:
        return "sw";
    case ESP_RST_PANIC:
        return "panic";
    case ESP_RST_INT_WDT:
        return "int_wdt";
    case ESP_RST_TASK_WDT:
        return "task_wdt";
    case ESP_RST_WDT:
        return "wdt";
    case ESP_RST_DEEPSLEEP:
        return "deepsleep";
    case ESP_RST_BROWNOUT:
        return "brownout";
    case ESP_RST_EXT:
        return "ext";
    default:
        return "other";
    }
}
#endif

/// millis() since a timestamp, or a sentinel when that thing has never happened.
/// Printed as age=-1 so a parser can tell "never seen" from "seen 0 ms ago".
static int32_t ageMs(uint32_t stampMs)
{
    return stampMs ? (int32_t)(millis() - stampMs) : -1;
}

class LimeskeyDiagThread : public concurrency::OSThread
{
  public:
    LimeskeyDiagThread() : OSThread("LKDiag", LIMESKEY_DIAG_PERIOD_MS) {}

  protected:
    int32_t runOnce() override;

  private:
    void emitSys();
    void emitMem();
    void emitLora();
    void emitGnss();
    void emitGnssCno();
    void emitGnssRf();
    void emitGnssSpi();
    void checkEvents();

    uint8_t step = 0; // which section the current sweep is on

    // Previous-sample state, for the event lines and for rate deltas.
    uint32_t lastCksumBad = 0;
    uint32_t lastRingOverflow = 0;
    uint32_t lastBusBytes = 0;
    uint32_t lastSweepMs = 0;
    uint8_t lastFixType = 0xFF;
    uint8_t lastJamState = 0xFF;
    uint16_t lastDevErr = 0;
    bool warnedHeap = false;
    bool warnedNoiseFloor = false;
    bool bootLineDone = false;
};

void LimeskeyDiagThread::emitSys()
{
    uint32_t bleConn = 0;
    int32_t bleRssi = 0;
    uint32_t queueHighWater = 0;
#if !MESHTASTIC_EXCLUDE_BLUETOOTH && defined(ARCH_ESP32)
    if (nimbleBluetooth && nimbleBluetooth->isActive() && nimbleBluetooth->isConnected()) {
        bleConn = 1;
        bleRssi = nimbleBluetooth->getRssi();
    }
    queueHighWater = limeskeyBleToPhoneHighWater();
#endif

    int32_t wifiRssi = 0;
    uint32_t wifiUp = 0;
#if HAS_WIFI && !defined(MESHTASTIC_EXCLUDE_WIFI)
    if (WiFi.status() == WL_CONNECTED) {
        wifiUp = 1;
        wifiRssi = WiFi.RSSI();
    }
#endif

#ifdef ARCH_ESP32
    const uint32_t heapFree = ESP.getFreeHeap();
    const uint32_t heapMin = ESP.getMinFreeHeap();
    const uint32_t heapMaxBlock = ESP.getMaxAllocHeap();
    const char *rst = resetReasonName();
#else
    const uint32_t heapFree = memGet.getFreeHeap();
    const uint32_t heapMin = 0, heapMaxBlock = 0;
    const char *rst = "n/a";
#endif

    // heapmax is the largest single allocation still possible: heap that is free
    // but badly fragmented shows up as heapfree staying flat while heapmax falls.
    LOG_INFO("LKD:sys up=%u rst=%s heapfree=%u heapmin=%u heapmax=%u ble=%u blerssi=%d wifi=%u wifirssi=%d "
             "q_tophone_hw=%u/%u",
             (unsigned)(millis() / 1000), rst, (unsigned)heapFree, (unsigned)heapMin, (unsigned)heapMaxBlock,
             (unsigned)bleConn, (int)bleRssi, (unsigned)wifiUp, (int)wifiRssi, (unsigned)queueHighWater,
             (unsigned)MAX_RX_TOPHONE);
}

void LimeskeyDiagThread::emitMem()
{
#if MESHTASTIC_MEM_AUDIT
    // Reuse upstream's per-subsystem accounting rather than a parallel scheme:
    // this is the same breakdown the MemAudit[boot] line prints, sampled live so
    // a subsystem that grows over a ride is visible against the boot baseline.
    memaudit::Tag tags[memaudit::kMaxTags];
    const size_t n = memaudit::snapshot(tags, memaudit::kMaxTags);
    if (!n)
        return;

    // Assembled into one line so the sections stay one-log-record each. The
    // buffer is a fixed stack array; nothing here allocates.
    char line[192];
    size_t off = 0;
    int32_t total = 0;
    for (size_t i = 0; i < n && off + 24 < sizeof(line); i++) {
        int w = snprintf(line + off, sizeof(line) - off, "%s%s=%d", i ? " " : "", tags[i].tag, (int)tags[i].bytes);
        if (w <= 0)
            break;
        off += (size_t)w;
        total += tags[i].bytes;
    }
    LOG_INFO("LKD:mem %s total=%d", line, (int)total);
#endif
}

void LimeskeyDiagThread::emitLora()
{
    RadioLibInterface *r = RadioLibInterface::instance;
    if (!r)
        return;

    // getNoiseFloor()/getAverageNoiseFloor() come from upstream's rolling RX-idle
    // RSSI sampler, so this tracks the known USB3 interference without us poking
    // the radio mid-RX. Expect roughly -110..-130 dBm clear; the bench-observed
    // ~-85 dBm is the interference signature.
    LOG_INFO("LKD:lora nf=%d nfavg=%d rxgood=%u rxbad=%u tx=%u txrelay=%u txdrop=%u chutil=%u.%02u txutil=%u.%02u deverr=0x%04x",
             (int)r->getNoiseFloor(), (int)r->getAverageNoiseFloor(), (unsigned)r->rxGood, (unsigned)r->rxBad,
             (unsigned)r->txGood, (unsigned)r->txRelay, (unsigned)r->txDrop,
             airTime ? (unsigned)airTime->channelUtilizationPercent() : 0u,
             airTime ? (unsigned)(airTime->channelUtilizationPercent() * 100) % 100 : 0u,
             airTime ? (unsigned)airTime->utilizationTXPercent() : 0u,
             airTime ? (unsigned)(airTime->utilizationTXPercent() * 100) % 100 : 0u, (unsigned)r->getDeviceErrorFlags());
}

void LimeskeyDiagThread::emitGnss()
{
    // fix: NAV-STATUS gpsFix (0=none 1=DR 2=2D 3=3D 4=GPS+DR 5=time-only).
    // dop values are hundredths, printed decimal. ttff is the module's own
    // time-to-first-fix for the current cold/warm start, in ms.
    LOG_INFO("LKD:gnss fix=%u fixflags=0x%02x sats=%u/%u pdop=%u.%02u hdop=%u.%02u vdop=%u.%02u ttff=%u msss=%u "
             "age_sat=%d age_dop=%d age_status=%d",
             (unsigned)state.gpsFix, (unsigned)state.fixFlags, (unsigned)state.numUsed, (unsigned)state.numSvs,
             (unsigned)(state.pDop / 100), (unsigned)(state.pDop % 100), (unsigned)(state.hDop / 100),
             (unsigned)(state.hDop % 100), (unsigned)(state.vDop / 100), (unsigned)(state.vDop % 100),
             (unsigned)state.ttffMs, (unsigned)state.msssMs, (int)ageMs(state.navSatMs), (int)ageMs(state.navDopMs),
             (int)ageMs(state.navStatusMs));
}

void LimeskeyDiagThread::emitGnssCno()
{
    // Per-constellation counts are used/seen. C/N0 collapsing across *all*
    // constellations at once is the antenna or shadowing signature; one
    // constellation dropping alone is not.
    LOG_INFO("LKD:gnss.cno top=%u,%u,%u,%u gps=%u/%u sbas=%u/%u gal=%u/%u bds=%u/%u qzss=%u/%u glo=%u/%u",
             (unsigned)state.topCno[0], (unsigned)state.topCno[1], (unsigned)state.topCno[2], (unsigned)state.topCno[3],
             (unsigned)state.usedPerGnss[0], (unsigned)state.seenPerGnss[0], (unsigned)state.usedPerGnss[1],
             (unsigned)state.seenPerGnss[1], (unsigned)state.usedPerGnss[2], (unsigned)state.seenPerGnss[2],
             (unsigned)state.usedPerGnss[3], (unsigned)state.seenPerGnss[3], (unsigned)state.usedPerGnss[5],
             (unsigned)state.seenPerGnss[5], (unsigned)state.usedPerGnss[6], (unsigned)state.seenPerGnss[6]);
}

void LimeskeyDiagThread::emitGnssRf()
{
    // ant/antpwr: this board feeds the active antenna from its own bias-T, and the
    // NEO's antenna supervisor pins (ANT_DET/ANT_OFF) are not wired, so open/short
    // detection is NOT valid here - the module reports its default. Logged anyway
    // so the assumption is visible in the data rather than only in a comment.
    // jam 0..255 is the CW jamming indicator; jamst 0=unknown 1=ok 2=warning 3=critical.
    LOG_INFO("LKD:gnss.rf blocks=%u jam=%u jamst=%u agc=%u noise=%u ofsi=%d ofsq=%d ant=%u antpwr=%u ant_valid=0 age=%d",
             (unsigned)state.rfBlocks, (unsigned)state.rfJamInd, (unsigned)state.rfJamState, (unsigned)state.rfAgc,
             (unsigned)state.rfNoise, (int)state.rfOfsI, (int)state.rfOfsQ, (unsigned)state.rfAntStatus,
             (unsigned)state.rfAntPower, (int)ageMs(state.monRfMs));
    LOG_INFO("LKD:gnss.hw noise=%u agc=%u jam=%u ant=%u antpwr=%u flags=0x%02x ant_valid=0 age=%d", (unsigned)state.hwNoise,
             (unsigned)state.hwAgc, (unsigned)state.hwJamInd, (unsigned)state.hwAntStatus, (unsigned)state.hwAntPower,
             (unsigned)state.hwFlags, (int)ageMs(state.monHwMs));
}

void LimeskeyDiagThread::emitGnssSpi()
{
    const auto &b = ubloxSPIGNSS.busStats();

    // idle_pct is the share of clocked bytes that were 0xFF filler. Near 100%
    // means the module is not talking (dead/reset/misconfigured port); a healthy
    // 1 Hz NMEA + periodic UBX stream sits well below that. ubxcrc rising is the
    // signal-integrity indicator - the 22R series resistor and the shared MISO
    // net are the things to suspect.
    const uint32_t read = b.bytesRead;
    const uint32_t idlePct = read ? (uint32_t)((uint64_t)b.bytesIdle * 100 / read) : 0;

    LOG_INFO("LKD:gnss.spi rd=%u wr=%u idle_pct=%u drains=%u ring_hw=%u/%u ovf=%u ubxok=%u ubxcrc=%u ubxseen=%u ubxbig=%u",
             (unsigned)read, (unsigned)b.bytesWritten, (unsigned)idlePct, (unsigned)b.drains, (unsigned)b.ringHighWater,
             (unsigned)GPS_SPI_RINGBUF, (unsigned)b.ringOverflow, (unsigned)stats.framesOk, (unsigned)stats.cksumBad,
             (unsigned)stats.framesSeen, (unsigned)stats.oversize);
}

void LimeskeyDiagThread::checkEvents()
{
    const auto &b = ubloxSPIGNSS.busStats();

    // Rising UBX checksum failures: bytes are being corrupted on the shared bus.
    if (b.bytesRead != lastBusBytes) {
        const uint32_t dBad = stats.cksumBad - lastCksumBad;
        if (dBad)
            LOG_WARN("LKD:evt gnss_spi_crc d_bad=%u total=%u d_bytes=%u", (unsigned)dBad, (unsigned)stats.cksumBad,
                     (unsigned)(b.bytesRead - lastBusBytes));
        lastCksumBad = stats.cksumBad;
        lastBusBytes = b.bytesRead;
    }

    // Ring overflow means GPS.cpp is not draining fast enough and NMEA is being
    // lost, which shows up downstream as missing fixes rather than as an error.
    if (b.ringOverflow != lastRingOverflow) {
        LOG_WARN("LKD:evt gnss_ring_overflow d=%u total=%u", (unsigned)(b.ringOverflow - lastRingOverflow),
                 (unsigned)b.ringOverflow);
        lastRingOverflow = b.ringOverflow;
    }

    // Fix-type transitions, so the log shows exactly when a fix was gained or lost
    // without having to diff two periodic lines. The first sample only latches the
    // baseline: reporting "from=0 to=0" on the first tick is noise, not an event.
    if (state.navStatusMs && state.gpsFix != lastFixType) {
        if (lastFixType != 0xFF)
            LOG_WARN("LKD:evt gnss_fix_change from=%u to=%u sats=%u/%u topcno=%u pdop=%u.%02u", (unsigned)lastFixType,
                     (unsigned)state.gpsFix, (unsigned)state.numUsed, (unsigned)state.numSvs, (unsigned)state.topCno[0],
                     (unsigned)(state.pDop / 100), (unsigned)(state.pDop % 100));
        lastFixType = state.gpsFix;
    }

    // Jamming state escalation from the module's own detector.
    if (state.monRfMs && state.rfJamState != lastJamState) {
        if (state.rfJamState >= 2)
            LOG_WARN("LKD:evt gnss_jam state=%u ind=%u agc=%u noise=%u", (unsigned)state.rfJamState, (unsigned)state.rfJamInd,
                     (unsigned)state.rfAgc, (unsigned)state.rfNoise);
        lastJamState = state.rfJamState;
    }

#ifdef ARCH_ESP32
    // One-shot so a sustained low-heap condition doesn't spam the BLE log.
    const uint32_t heapFree = ESP.getFreeHeap();
    if (!warnedHeap && heapFree < 24 * 1024) {
        LOG_WARN("LKD:evt heap_low free=%u min=%u max=%u", (unsigned)heapFree, (unsigned)ESP.getMinFreeHeap(),
                 (unsigned)ESP.getMaxAllocHeap());
        warnedHeap = true;
    } else if (warnedHeap && heapFree > 40 * 1024) {
        warnedHeap = false;
    }
#endif

    RadioLibInterface *r = RadioLibInterface::instance;
    if (r) {
        // Drive the sampler ourselves; see sampleNoiseFloor() for why relying on
        // DeviceTelemetry's cadence leaves the reading pinned at its default.
        r->sampleNoiseFloor();

        // -100 dBm is already ~20 dB worse than a clean SX1262 RX floor; the
        // bench figure with USB3 nearby was about -85.
        const int32_t nf = r->getAverageNoiseFloor();
        if (!warnedNoiseFloor && nf > -100) {
            LOG_WARN("LKD:evt lora_noisefloor_high nf=%d rxgood=%u rxbad=%u", (int)nf, (unsigned)r->rxGood,
                     (unsigned)r->rxBad);
            warnedNoiseFloor = true;
        } else if (warnedNoiseFloor && nf < -105) {
            LOG_INFO("LKD:evt lora_noisefloor_recovered nf=%d", (int)nf);
            warnedNoiseFloor = false;
        }

        // The SX126x error word is latched until cleared, so re-reporting it every
        // tick would be one line per tick forever. Report each distinct value once.
        const uint16_t devErr = r->getDeviceErrorFlags();
        if (devErr != lastDevErr) {
            if (devErr)
                LOG_WARN("LKD:evt lora_deverr flags=0x%04x", (unsigned)devErr);
            else
                LOG_INFO("LKD:evt lora_deverr_cleared");
            lastDevErr = devErr;
        }
    }
}

int32_t LimeskeyDiagThread::runOnce()
{
    if (!bootLineDone) {
        // Emitted once so a downloaded log is self-describing: a reader that has
        // never seen these lines can tell what produced them and at what cadence.
        LOG_INFO("LKD:hello ver=1 board=limeskey_node period_ms=%u spacing_ms=%u sections=sys,mem,lora,gnss,gnss.cno,"
                 "gnss.rf,gnss.hw,gnss.spi events=LKD:evt",
                 (unsigned)LIMESKEY_DIAG_PERIOD_MS, (unsigned)LIMESKEY_DIAG_LINE_SPACING_MS);
        bootLineDone = true;
    }

    // One section per tick. Events are checked every tick so an anomaly is
    // reported within a line-spacing rather than at the end of the sweep.
    checkEvents();

    switch (step) {
    case 0:
        emitSys();
        break;
    case 1:
        emitMem();
        break;
    case 2:
        emitLora();
        break;
    case 3:
        emitGnss();
        break;
    case 4:
        emitGnssCno();
        break;
    case 5:
        emitGnssRf();
        break;
    default:
        emitGnssSpi();
        break;
    }

    if (++step > 6) {
        step = 0;
        // Sleep out the rest of the period. The sweep itself consumed
        // 7 * spacing, so subtract it rather than adding to the cadence.
        const uint32_t sweep = 7 * LIMESKEY_DIAG_LINE_SPACING_MS;
        return LIMESKEY_DIAG_PERIOD_MS > sweep ? LIMESKEY_DIAG_PERIOD_MS - sweep : LIMESKEY_DIAG_LINE_SPACING_MS;
    }
    return LIMESKEY_DIAG_LINE_SPACING_MS;
}

static LimeskeyDiagThread *diagThread;

void setup()
{
    if (!diagThread)
        diagThread = new LimeskeyDiagThread();
}

} // namespace limeskeydiag

#endif // LIMESKEY_DIAG
