/*
 * Sensirion SEN5x environmental node (PM / RH / T / VOC) on i2c2.
 *
 * Deliberately not a Zephyr sensor driver: there is one consumer, three
 * commands and no other board to share it with. Same shape as radar.h - poll
 * it, then take the window when the record is due.
 */
#ifndef SEN5X_H_
#define SEN5X_H_

#include <stdbool.h>
#include <stdint.h>

/*
 * "Not measured". The sensor returns 0xFFFF for anything it does not have -
 * a SEN50 has no RH/T/gas at all, and every field reads 0xFFFF before the
 * first conversion. Signed fields use INT16_MAX instead, because 0xFFFF is
 * -1 there and -0.01 C is a perfectly plausible reading.
 */
#define SEN5X_UNKNOWN_U   UINT16_MAX
#define SEN5X_UNKNOWN_S   INT16_MAX

struct sen5x_window {
	uint16_t pm25;   /* 0.1 ug/m3, highest sample in the window */
	uint16_t pm10;   /* 0.1 ug/m3, highest sample in the window */
	int16_t  temp;   /* 0.01 C, last good sample */
	int16_t  rh;     /* 0.01 %RH, last good sample */
	int16_t  voc;    /* 0.1 index, last good sample (SEN54/SEN55 only) */
	bool     alive;  /* a good read this window, and no fan/laser fault */
};

/* Probes the sensor, prints which model it is, and starts Measurement-Mode.
 * Non-zero means no sensor: poll/take then keep returning "unknown". */
int sen5x_start(void);

/* One sample, folded into the current window. Call about once a second -
 * the module produces a new reading every 1 s. Blocks ~20 ms on the bus. */
void sen5x_poll(void);

/* Hands over the window and starts the next one. */
void sen5x_window_take(struct sen5x_window *out);

#endif /* SEN5X_H_ */
