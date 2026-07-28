#include "UBloxSPIGNSS.h"

#if defined(GPS_SPI_INTERFACE)

#include "SPILock.h" // spiLock + concurrency::LockGuard

// Always included: the tap compiles to pass-through stubs without LIMESKEY_DIAG.
#include "limeskey/LimeskeyDiag.h"

// The one global instance. GPS::_serial_gps points at this via GPS_SERIAL_PORT.
UBloxSPIGNSS ubloxSPIGNSS;

void UBloxSPIGNSS::begin(unsigned long baud, uint32_t, int8_t, int8_t)
{
    _baud = baud; // remembered only so baudRate() has something to return
    if (_started)
        return;

    pinMode(GPS_SPI_CS_PIN, OUTPUT);
    digitalWrite(GPS_SPI_CS_PIN, HIGH); // idle / deselected (also done in earlyInitVariant)

    // NOTE: the SPI bus itself is already up — main.cpp called SPI.begin() with the
    // LoRa pins before createGps() runs, and the GNSS shares that bus.
    _started = true;
    LOG_INFO("u-blox SPI GNSS ready (CS=%d, %lu Hz)", GPS_SPI_CS_PIN, (unsigned long)GPS_SPI_HZ);
}

bool UBloxSPIGNSS::ringPush(uint8_t b)
{
    uint16_t next = (_head + 1) % GPS_SPI_RINGBUF;
    if (next == _tail) {
#ifdef LIMESKEY_DIAG
        _stats.ringOverflow++;
#endif
        return false; // full — drop; reader will catch up next poll
    }
    _ring[_head] = b;
    _head = next;
#ifdef LIMESKEY_DIAG
    noteRingLevel();
#endif
    return true;
}

int UBloxSPIGNSS::ringPop()
{
    if (_head == _tail)
        return -1;
    uint8_t b = _ring[_tail];
    _tail = (_tail + 1) % GPS_SPI_RINGBUF;
    return b;
}

// Clock the module and stash any real bytes. Holds the shared SPI lock only for
// the duration of one bounded chunk so the SX1262 is never blocked for long.
void UBloxSPIGNSS::drain(size_t maxBytes)
{
    if (!_started)
        return;

    // If the last poll found nothing, don't re-clock the shared bus until the
    // idle-poll interval elapses. This keeps getACK/readBytes deadline loops from
    // busy-polling spiLock (and the SX1262's bus) while waiting for a response.
    // Called only from the GPS thread, so these fields need no lock.
    if (_lastDrainEmpty && (millis() - _lastDrainMs) < GPS_SPI_IDLE_POLL_MS)
        return;

    concurrency::LockGuard g(spiLock);
    GPS_SPI_BUS.beginTransaction(_settings);
    digitalWrite(GPS_SPI_CS_PIN, LOW);

#ifdef LIMESKEY_DIAG
    _stats.drains++;
#endif

    uint8_t idle = 0;
    for (size_t n = 0; n < maxBytes; n++) {
        uint8_t b = GPS_SPI_BUS.transfer(0xFF);
#ifdef LIMESKEY_DIAG
        _stats.bytesRead++;
        if (b == 0xFF)
            _stats.bytesIdle++;
#endif

        // Every byte goes through the tap before the ring. It decodes and withholds
        // the diagnostic messages, and passes everything else (NMEA, ACK/NAK,
        // MON-VER) straight back. Without the flag this is a pass-through stub.
        uint8_t rel[LIMESKEY_UBX_TAP_MAX_RELEASE];
        const uint8_t nrel = limeskeydiag::ubxTapFeed(b, rel);

        bool ringFull = false;
        for (uint8_t i = 0; i < nrel; i++) {
            if (!ringPush(rel[i])) {
                ringFull = true;
                break;
            }
        }
        if (ringFull)
            break;

        // A run of 0xFF only means "module buffer empty" between frames. Inside a
        // UBX frame it is ordinary payload (an unset field reads as all-ones), and
        // counting it used to truncate the frame and cost the NMEA stream the rest
        // of the poll.
        if (nrel == 0 || limeskeydiag::ubxTapInFrame()) {
            idle = 0;
            continue;
        }

        if (rel[nrel - 1] == 0xFF) {
            if (++idle >= GPS_SPI_IDLE_RUN) {
                // Trailing filler run => buffer empty. Remove the run we just pushed.
                _head = (_head + GPS_SPI_RINGBUF - idle) % GPS_SPI_RINGBUF;
                break;
            }
        } else {
            idle = 0; // a 0xFF that is followed by real data was intra-frame; keep it
        }
    }

    digitalWrite(GPS_SPI_CS_PIN, HIGH);
    GPS_SPI_BUS.endTransaction();

    // drain() is only entered with an empty ring, so ringCount() now reflects
    // exactly what this poll produced.
    _lastDrainMs = millis();
    _lastDrainEmpty = (ringCount() == 0);
}

int UBloxSPIGNSS::available()
{
    if (_head == _tail)
        drain(GPS_SPI_DRAIN_CHUNK);
    return ringCount();
}

int UBloxSPIGNSS::read()
{
    if (_head == _tail)
        drain(GPS_SPI_DRAIN_CHUNK);
    return ringPop();
}

int UBloxSPIGNSS::peek()
{
    if (_head == _tail)
        drain(GPS_SPI_DRAIN_CHUNK);
    return (_head == _tail) ? -1 : _ring[_tail];
}

size_t UBloxSPIGNSS::write(const uint8_t *buffer, size_t size)
{
    if (!_started)
        return 0;

    concurrency::LockGuard g(spiLock);
    GPS_SPI_BUS.beginTransaction(_settings);
    digitalWrite(GPS_SPI_CS_PIN, LOW);
    for (size_t i = 0; i < size; i++) {
        uint8_t r = GPS_SPI_BUS.transfer(buffer[i]);
#ifdef LIMESKEY_DIAG
        _stats.bytesWritten++;
        _stats.bytesRead++;
        if (r == 0xFF)
            _stats.bytesIdle++;
#endif
        // Same filtering as drain(): capture what the module streams back during a
        // write, minus the diagnostic frames. ACKs must survive this path - getACK()
        // reads them right after the command that is being written here.
        uint8_t rel[LIMESKEY_UBX_TAP_MAX_RELEASE];
        const uint8_t nrel = limeskeydiag::ubxTapFeed(r, rel);
        for (uint8_t k = 0; k < nrel; k++)
            if (rel[k] != 0xFF)
                ringPush(rel[k]);
    }
    digitalWrite(GPS_SPI_CS_PIN, HIGH);
    GPS_SPI_BUS.endTransaction();
    return size;
}

size_t UBloxSPIGNSS::write(uint8_t b)
{
    return write(&b, 1);
}

#endif // GPS_SPI_INTERFACE
