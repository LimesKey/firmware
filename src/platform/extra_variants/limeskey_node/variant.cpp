#include "configuration.h"

#ifdef _VARIANT_LIMESKEY_NODE

// limeskey-node board-specific early init.
//
// The u-blox NEO-M9N is wired in SPI mode and shares the LoRa SPI bus (it sits
// on the same /MISO and /MOSI nets as the SX1262). It is driven by the SPI GNSS
// adapter (src/gps/UBloxSPIGNSS.cpp), but that adapter doesn't come up until
// createGps(), which runs after the radio's first SPI transactions. If the NEO's
// CS were left floating/low before then, it could drive the shared MISO line and
// corrupt early LoRa SPI traffic.
//
// earlyInitVariant() runs at the very start of setup(), before SPI.begin() and
// the radio init, so this is the right place to park the NEO's CS HIGH (idle /
// deselected). With CS HIGH the NEO tri-states MISO and stays off the bus until
// the GNSS adapter selects it. The adapter manages CS for its own transactions
// from then on.
void earlyInitVariant()
{
    // XIAO ESP32-C6 2.4 GHz (Wi-Fi/BLE) antenna switch. The module boots on its
    // onboard ceramic antenna; this routes the radio to the external U.FL
    // connector instead. GPIO3 LOW powers the RF switch; GPIO14 HIGH selects
    // external (LOW = onboard). These are XIAO-internal control lines, not board
    // nets.
    //
    // OPT-IN by design: driving the switch to an *open* U.FL port hurts 2.4 GHz
    // performance, so the safe default (flag undefined) leaves the onboard antenna
    // selected. Define XIAO_EXTERNAL_24_ANTENNA in platformio.ini only when a U.FL
    // antenna is actually fitted. Verify the pin numbers/polarity against your
    // XIAO ESP32-C6 revision first. (Does not affect the LoRa antenna, which is on
    // the E22P module.)
#ifdef XIAO_EXTERNAL_24_ANTENNA
    constexpr int XIAO_C6_RF_SWITCH_EN = 3;  // LOW = RF switch powered on
    constexpr int XIAO_C6_ANT_SELECT = 14;   // HIGH = external U.FL, LOW = onboard
    pinMode(XIAO_C6_RF_SWITCH_EN, OUTPUT);
    digitalWrite(XIAO_C6_RF_SWITCH_EN, LOW);
    delay(100);
    pinMode(XIAO_C6_ANT_SELECT, OUTPUT);
    digitalWrite(XIAO_C6_ANT_SELECT, HIGH);
#endif

#ifdef GPS_SPI_CS_PIN
    pinMode(GPS_SPI_CS_PIN, OUTPUT);
    digitalWrite(GPS_SPI_CS_PIN, HIGH); // deselect NEO-M9N → release shared SPI MISO
#endif
}

#endif
