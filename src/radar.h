/*
 * IWRL6432 link: reset, push the cfg over UART, then parse the TLV stream and
 * accumulate occupancy until someone takes the window.
 */
#ifndef RADAR_H_
#define RADAR_H_

#include <stdbool.h>
#include <stdint.h>

struct radar_window {
	uint8_t  headcount;   /* most targets seen at once during the window */
	uint8_t  occ_s;       /* seconds of the window that were occupied */
	uint16_t dwell_s;     /* unbroken occupancy at the moment of the take */
	bool     alive;       /* any frame arrived at all */
	bool     tracker;     /* headcount came from the tracker, not presence.
			       * False means it is a floor (0 or 1), not a count. */
};

/* Spawns the radar thread. Returns before the sensor is configured - it takes
 * a few seconds of CLI, and the thread retries on its own if it fails. */
int radar_start(void);

/* Snapshot the accumulators and reset them for the next window. */
void radar_window_take(struct radar_window *out);

#endif /* RADAR_H_ */
