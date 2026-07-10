/**
 * status_icons.hpp - Bit-mask designs for the 8 status-bar icons referenced
 * in GUI_Requirements.txt. Each mask is 16 px wide x 16 rows tall, stored
 * as 16 uint16_t entries (bit 15 = leftmost pixel).
 *
 * Adding / refining icons
 * -----------------------
 * Open status_icons.cpp; each icon has an ASCII art preview right next to
 * its bit pattern, so editing pixels is just toggling characters and
 * recomputing one row's hex value. No asset pipeline required.
 */
#ifndef CPAP_STATUS_ICONS_HPP
#define CPAP_STATUS_ICONS_HPP

#include <stdint.h>

namespace cpap
{

/* All masks below are 16 rows of 16 bits. */
extern const uint16_t kIconHome[16];
extern const uint16_t kIconBluetooth[16];

/* Cellular signal-strength: pick the array matching the bar count. Index 0
 * is "no signal at all" (kIconNoCellular below); 1..4 are progressively
 * fuller bar towers. Selecting the right one is the caller's job. */
extern const uint16_t kIconCellularBars1[16];
extern const uint16_t kIconCellularBars2[16];
extern const uint16_t kIconCellularBars3[16];
extern const uint16_t kIconCellularBars4[16];

/* "No cellular connection" - simple X glyph distinct from the bar towers. */
extern const uint16_t kIconNoCellular[16];

extern const uint16_t kIconAirplane[16];
extern const uint16_t kIconHumFault[16];
extern const uint16_t kIconHumWarming[16];
extern const uint16_t kIconHumCooling[16];

} /* namespace cpap */

#endif /* CPAP_STATUS_ICONS_HPP */
