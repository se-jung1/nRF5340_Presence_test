/*
 * SEN5x over I2C. Datasheet "Environmental Sensor Node SEN5x", version 2 - D1,
 * section 6: address 0x69, standard mode only (100 kbit/s), no clock
 * stretching, every 16-bit word followed by a CRC-8.
 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <errno.h>
#include <string.h>

#include "sen5x.h"

#define CMD_START_MEASUREMENT  0x0021
#define CMD_READ_MEASURED      0x03C4
#define CMD_READ_PRODUCT_NAME  0xD014
#define CMD_READ_STATUS        0xD206

/*
 * Device Status Register bits (5.4) that make the PM number a lie: FAN (4) is
 * "fan switched on and measured 0 RPM", LASER (5) is "laser current out of
 * range". Both latch, both mean no air is being sampled - which reads as clean
 * air, the one failure this must never report silently.
 *
 * SPEED (21) is deliberately not here: it sets during fan spin-up and at
 * temperature extremes and clears itself once the fan reaches speed.
 * RHT (6) and GAS (7) are not here either - they only break temp/rh/voc, and
 * those already come back as 0xFFFF when they do.
 */
#define STATUS_FATAL   (BIT(4) | BIT(5))

/* Longest read we make is the 32-char product name: 16 words, 48 bytes. */
#define MAX_WORDS  16

static const struct i2c_dt_spec bus = I2C_DT_SPEC_GET(DT_NODELABEL(sen5x));
static bool running;
static struct sen5x_window acc;

/* CRC-8, poly 0x31, init 0xFF, no reflection, no final XOR. CRC(0xBEEF)=0x92. */
static uint8_t crc8(const uint8_t *d)
{
	uint8_t crc = 0xFF;

	for (int i = 0; i < 2; i++) {
		crc ^= d[i];
		for (int b = 0; b < 8; b++) {
			crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x31)
					   : (uint8_t)(crc << 1);
		}
	}
	return crc;
}

static int send_cmd(uint16_t c)
{
	uint8_t b[2] = { (uint8_t)(c >> 8), (uint8_t)c };

	/* The command code carries its own 3-bit CRC, so no checksum byte. */
	return i2c_write_dt(&bus, b, sizeof(b));
}

/*
 * Command, wait, read back `words` CRC-checked 16-bit words.
 *
 * Two transactions with a delay between them, not i2c_write_read_dt(): 6.1.5
 * asks for 20 ms between the command and the read header so the module can
 * fill its buffer, and the part does not clock-stretch to enforce it. A
 * repeated start returns whatever was in the buffer last time.
 */
static int read_words(uint16_t c, uint16_t *out, int words)
{
	uint8_t raw[3 * MAX_WORDS];
	int err = send_cmd(c);

	if (err) {
		return err;
	}
	k_msleep(20);

	err = i2c_read_dt(&bus, raw, words * 3);
	if (err) {
		return err;
	}
	for (int i = 0; i < words; i++) {
		if (crc8(&raw[i * 3]) != raw[i * 3 + 2]) {
			return -EIO;
		}
		out[i] = ((uint16_t)raw[i * 3] << 8) | raw[i * 3 + 1];
	}
	return 0;
}

static void acc_reset(void)
{
	acc.pm25 = SEN5X_UNKNOWN_U;
	acc.pm10 = SEN5X_UNKNOWN_U;
	acc.temp = SEN5X_UNKNOWN_S;
	acc.rh   = SEN5X_UNKNOWN_S;
	acc.voc  = SEN5X_UNKNOWN_S;
	acc.alive = false;
}

/* Which model is on the bus - handy on a bench, and it is the one read that
 * proves the wiring before the fan even spins up. */
static void print_product_name(void)
{
	uint16_t w[MAX_WORDS];
	char name[2 * MAX_WORDS + 1] = { 0 };

	if (read_words(CMD_READ_PRODUCT_NAME, w, MAX_WORDS) != 0) {
		return;
	}
	for (int i = 0; i < MAX_WORDS; i++) {
		name[2 * i]     = (char)(w[i] >> 8);
		name[2 * i + 1] = (char)w[i];
	}
	/* SEN50 has no RH/T/gas, SEN54 has no NOx - both just report 0xFFFF
	 * for what they do not have, so nothing below branches on this. */
	printk("[SEN5x] %s\n", name);
}

int sen5x_start(void)
{
	int err;

	acc_reset();

	if (!device_is_ready(bus.bus)) {
		printk("[SEN5x] i2c2 not ready\n");
		return -ENODEV;
	}

	print_product_name();

	err = send_cmd(CMD_START_MEASUREMENT);
	if (err) {
		printk("[SEN5x] no answer at 0x%02x (%d) - check 5 V, SEL to GND, "
		       "and the pull-ups\n", bus.addr, err);
		return err;
	}
	k_msleep(50);   /* command execution time, 6.1 */

	running = true;
	printk("[READY] SEN5x measuring (PM stabilises after ~30 s of fan)\n");
	return 0;
}

void sen5x_poll(void)
{
	uint16_t w[8];

	if (!running) {
		return;
	}
	/*
	 * Read Measured Values without checking the Data-Ready flag: it always
	 * returns the latest sample (6.1.5), and polling the flag first would
	 * double the bus traffic to avoid re-reading a value we fold in
	 * idempotently anyway - a peak stays the same peak.
	 */
	if (read_words(CMD_READ_MEASURED, w, 8) != 0) {
		return;
	}

	/*
	 * PM is peak-held and RH/T/VOC are last-value on purpose. A dust event
	 * lasting ten seconds inside a 30 s window is the whole point of the
	 * record - averaging it away would hide exactly what this is for -
	 * while ambient temperature moves slowly enough that the last sample
	 * describes the window as well as any average would.
	 */
	if (w[1] != 0xFFFF && (acc.pm25 == SEN5X_UNKNOWN_U || w[1] > acc.pm25)) {
		acc.pm25 = w[1];
	}
	if (w[3] != 0xFFFF && (acc.pm10 == SEN5X_UNKNOWN_U || w[3] > acc.pm10)) {
		acc.pm10 = w[3];
	}
	if (w[4] != 0xFFFF) {
		acc.rh = (int16_t)w[4];             /* already 0.01 %RH */
	}
	if (w[5] != 0xFFFF) {
		acc.temp = (int16_t)w[5] / 2;       /* 1/200 C -> 0.01 C */
	}
	if (w[6] != 0xFFFF) {
		acc.voc = (int16_t)w[6];            /* already 0.1 index */
	}
	acc.alive = true;

	/* w[0] PM1.0, w[2] PM4.0 and w[7] NOx are read and dropped: they cost
	 * nothing extra on the bus (one command returns all eight) and 6 more
	 * bytes per record for numbers nothing downstream shows. */
}

void sen5x_window_take(struct sen5x_window *out)
{
	uint16_t st[2];

	if (acc.alive && read_words(CMD_READ_STATUS, st, 2) == 0) {
		uint32_t status = ((uint32_t)st[0] << 16) | st[1];

		if (status & STATUS_FATAL) {
			printk("[SEN5x] device status 0x%08x - fan or laser fault\n",
			       status);
			acc.alive = false;
		}
	}

	*out = acc;
	acc_reset();
}
