/*
 * Flash-backed ring of occupancy + air quality records.
 *
 * Holds 24 h at one record per STORE_PERIOD_S. seq is monotonic and survives
 * reset - the app and server dedupe on (device_id, seq), so restarting at 0
 * would make new data collide with old and get dropped.
 */
#ifndef STORE_H_
#define STORE_H_

#include <stdint.h>

#include "frame.h"

/*
 * TEST BUILD: 30 s, matching the air-quality node's test setting. Put this back
 * to 60 before shipping, or change the app's samplePeriod and the server's
 * SAMPLE_PERIOD_SECONDS to match - they compute "time occupied" by multiplying
 * record counts by this.
 *
 * Must stay <= 255: occ_s is one byte of the window, and dwell_s is a u16.
 */
#define STORE_PERIOD_S   30
#define STORE_CAPACITY   (24 * 3600 / STORE_PERIOD_S)   /* always 24 h */

int store_init(void);

/* Fills in seq and ts, then persists. Safe to call from any thread. */
int store_append(struct sntl_record *rec);

/* seq that will be handed to the next appended record. */
uint32_t store_next_seq(void);

/* Oldest seq still in the ring (0 until it has wrapped). */
uint32_t store_oldest_seq(void);

/* 0 on success, -ENOENT if that seq is no longer held. */
int store_get(uint32_t seq, struct sntl_record *out);

/* Device seconds since first ever boot. Not wall clock - records carry
 * SNTL_FLAG_RTC_UNSET until something sets the time. */
uint32_t store_now(void);

#endif /* STORE_H_ */
