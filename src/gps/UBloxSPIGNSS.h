#pragma once
#include "configuration.h"

#if defined(GPS_SPI_INTERFACE)

#include <SPI.h>

// ---------------------------------------------------------------------------
// Tunables (override any of these in your variant.h before this header is seen)
// ---------------------------------------------------------------------------
#ifndef GPS_SPI_CS_PIN
#error "GPS_SPI_INTERFACE is defined but GPS_SPI_CS_PIN (NEO-M9N chip select) is not"
#endif

// NEO-M9N SPI port maximum is 5.5 MHz (Integration Manual UBX-19014286). We default
// low so each shared-bus transaction holds the SPI lock only briefly, keeping the
// SX1262 from being starved. Raise if you need more GNSS throughput.
#ifndef GPS_SPI_HZ
#define GPS_SPI_HZ 1000000UL
#endif

// Software receive ring. One NAV epoch of NMEA is a few hundred bytes.
#ifndef GPS_SPI_RINGBUF
#define GPS_SPI_RINGBUF 1024
#endif

// Max bytes clocked per available()/read() while the spiLock is held. Bounds how
// long the radio can be blocked: 256 B @ 1 MHz ~= 2 ms.
#ifndef GPS_SPI_DRAIN_CHUNK
#define GPS_SPI_DRAIN_CHUNK 256
#endif

// When the module has nothing to send, don't re-clock the shared SPI bus more
// often than this. GPS.cpp polls available()/read() in tight deadline loops
// (getACK, readBytes) that would otherwise busy-poll spiLock while waiting for a
// response; this paces those polls without meaningfully delaying real data.
#ifndef GPS_SPI_IDLE_POLL_MS
#define GPS_SPI_IDLE_POLL_MS 5
#endif

// A run of this many 0xFF bytes at a frame boundary means "module buffer empty".
// 0xFF is also a valid byte inside a UBX payload (e.g. 0xFFFFFFFF = invalid field),
// so a value that is too small can truncate binary UBX. NMEA is ASCII and never
// contains 0xFF, so the position path is unaffected. See README for the trade-off.
#ifndef GPS_SPI_IDLE_RUN
#define GPS_SPI_IDLE_RUN 4
#endif

// Which Arduino SPI bus the LoRa radio and the GNSS share. main.cpp already calls
// SPI.begin() with the LoRa pins, so we reuse that same bus object.
#ifndef GPS_SPI_BUS
#define GPS_SPI_BUS SPI
#endif

/**
 * Stream adapter that lets Meshtastic's (UART-only) GPS stack drive a u-blox
 * receiver over SPI.
 *
 * The u-blox SPI port carries the *identical* UBX/NMEA byte stream as its UART:
 * to read you clock 0xFF and take whatever comes back (a run of 0xFF = no data);
 * to write you clock the command bytes straight out. Because GPS.cpp only ever
 * touches this object through Stream/Print methods (available/read/peek/write/
 * readBytes) plus a handful of HardwareSerial-shaped control calls, we can stand
 * in for a serial port with no changes to the GPS state machine itself.
 *
 * The SPI bus is shared with the SX1262, so every transfer is wrapped in the
 * global spiLock (the same lock RadioLib's LockingArduinoHal takes). RadioLib
 * leaves the LoRa CS parked HIGH between its transactions, and we keep the GNSS
 * CS parked HIGH between ours, so the two devices never both drive MISO.
 */
class UBloxSPIGNSS : public Stream
{
  public:
    // ---- Stream / Print data path ----
    int available() override;
    int read() override;
    int peek() override;
    size_t write(uint8_t b) override;
    size_t write(const uint8_t *buffer, size_t size) override;
    using Print::write; // keep Print's write(const char*) etc. (GPS.cpp writes string literals)
    void flush() {} // writes are synchronous, nothing buffered on the TX side
    // Matches HardwareSerial::flush(bool): flush(false) discards pending RX (what
    // GPS::clearBuffer expects); flush(true) is TX-only and a no-op for us.
    void flush(bool txOnly)
    {
        if (!txOnly)
            _head = _tail = 0; // drop buffered RX (same thread as read(), no lock needed)
    }

    // ---- HardwareSerial-shaped control surface GPS.cpp expects (no-ops on SPI) ----
    void begin(unsigned long baud, uint32_t cfg = SERIAL_8N1, int8_t rxPin = -1, int8_t txPin = -1);
    void end() {}
    unsigned long baudRate() { return _baud; }
    void updateBaudRate(unsigned long baud) { _baud = baud; }
    size_t setRxBufferSize(size_t size) { return size; }
    void setFIFOSize(size_t) {}
    void setPins(int8_t, int8_t) {}
    void setPinout(int8_t, int8_t) {}
    void setRx(int8_t) {}
    void setTx(int8_t) {}

#ifdef LIMESKEY_DIAG
    // Bus-health counters for the LKD: diagnostics. Written only from the GPS thread
    // (every entry point below is called from there); read from the diag thread, which
    // tolerates a torn read of an individual counter. See LimeskeyDiag.cpp.
    struct BusStats {
        uint32_t bytesRead;    // bytes clocked in by drain(), including 0xFF filler
        uint32_t bytesIdle;    // of those, how many were 0xFF (filler / "nothing to send")
        uint32_t bytesWritten; // bytes clocked out by write()
        uint32_t drains;       // drain() calls that actually touched the bus
        uint32_t ringOverflow; // ringPush() rejections (reader fell behind)
        uint16_t ringHighWater;
    };
    const BusStats &busStats() const { return _stats; }
#endif

  private:
    void drain(size_t maxBytes); // clock the module under spiLock; push real bytes to the ring
    bool ringPush(uint8_t b);
    int ringPop();
    inline uint16_t ringCount() const { return (_head + GPS_SPI_RINGBUF - _tail) % GPS_SPI_RINGBUF; }

#ifdef LIMESKEY_DIAG
    BusStats _stats = {};
    inline void noteRingLevel()
    {
        uint16_t n = ringCount();
        if (n > _stats.ringHighWater)
            _stats.ringHighWater = n;
    }
#endif

    SPISettings _settings{GPS_SPI_HZ, MSBFIRST, SPI_MODE0};
    unsigned long _baud = 0;
    bool _started = false;

    uint8_t _ring[GPS_SPI_RINGBUF];
    uint16_t _head = 0; // write index
    uint16_t _tail = 0; // read index

    uint32_t _lastDrainMs = 0;  // for GPS_SPI_IDLE_POLL_MS throttle
    bool _lastDrainEmpty = false; // last drain produced no usable bytes
};

// The single instance, referenced from variant.h via `#define GPS_SERIAL_PORT ubloxSPIGNSS`.
extern UBloxSPIGNSS ubloxSPIGNSS;

#endif // GPS_SPI_INTERFACE
