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
#ifdef GPS_SPI_CS_PIN
    pinMode(GPS_SPI_CS_PIN, OUTPUT);
    digitalWrite(GPS_SPI_CS_PIN, HIGH); // deselect NEO-M9N → release shared SPI MISO
#endif
}

#endif
