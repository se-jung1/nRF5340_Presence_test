/*
 * Presence node: IWRL6432BOOST mmWave radar over uart1.
 *
 * - one record per STORE_PERIOD_S into flash (24 h ring), handed out over BLE
 * - the radar does all the signal processing on its own Cortex-M4F and sends
 *   only results, so this core does framing, counting and storage - nothing
 *   here is real-time critical
 *
 * There is no calendar clock on this chip: RTC0/RTC1 are plain 32.768 kHz
 * counters and the DK has no battery-backed RTC, so every record carries
 * SNTL_FLAG_RTC_UNSET and ts is device seconds since first boot. The receiver
 * back-computes: real time = download time - (now_ts - record_ts).
 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#include "ble.h"
#include "frame.h"
#include "radar.h"
#include "store.h"

/* DK "Button 1" (P0.23) - opens the BLE advertising window on press. */
static const struct gpio_dt_spec adv_btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback adv_btn_cb;

/* No debounce: a bounce just re-arms the same 20 s window, which is what a
 * second press would do anyway. */
static void adv_btn_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev); ARG_UNUSED(cb); ARG_UNUSED(pins);
	ble_advertise_window();
}

static int adv_btn_init(void)
{
	int err;

	if (!gpio_is_ready_dt(&adv_btn)) {
		return -ENODEV;
	}
	err = gpio_pin_configure_dt(&adv_btn, GPIO_INPUT);
	if (err) {
		return err;
	}
	err = gpio_pin_interrupt_configure_dt(&adv_btn, GPIO_INT_EDGE_TO_ACTIVE);
	if (err) {
		return err;
	}
	gpio_init_callback(&adv_btn_cb, adv_btn_pressed, BIT(adv_btn.pin));
	return gpio_add_callback(adv_btn.port, &adv_btn_cb);
}

/* Turn the last window into one record and write it. */
static void window_flush(void)
{
	struct radar_window w;
	struct sntl_record rec = { 0 };
	int err;

	radar_window_take(&w);

	rec.headcount = w.headcount;
	rec.occ_s = w.occ_s;
	rec.dwell_s = w.dwell_s;

	if (!w.alive) {
		/* Tag it, never drop it - this is safety-management data. */
		rec.flags |= SNTL_FLAG_SENSOR_FAULT;
	}
	if (!w.tracker) {
		/* headcount is a floor (0 or 1), not a count. */
		rec.flags |= SNTL_FLAG_NO_TRACKER;
	}
	rec.flags |= SNTL_FLAG_RTC_UNSET;   /* no time source on this board */
	/* DK runs off USB and has no battery gauge. Report full rather than a
	 * made up number; wire an ADC divider here for a real product. */
	rec.batt = 100;

	err = store_append(&rec);
	if (err) {
		printk("[STORE] append failed (%d)\n", err);
		return;
	}
	printk("[STORE] %us  headcount %u%s  occupied %us/%us  dwell %us\n",
	       rec.ts, rec.headcount, w.tracker ? "" : " (no tracker)",
	       rec.occ_s, STORE_PERIOD_S, rec.dwell_s);
}

int main(void)
{
	int64_t next_tick;
	uint32_t sec = 0;
	int err;

	printk("\n");
	printk("========================================\n");
	printk("   Presence node: IWRL6432 mmWave\n");
	printk("========================================\n");

	err = store_init();
	if (err) {
		printk("[ERROR] store init failed (%d) - nothing will be recorded\n", err);
	} else {
		printk("[READY] store: next seq %u, oldest %u, t=%u s\n",
		       store_next_seq(), store_oldest_seq(), store_now());
	}

	err = ble_start();
	if (err) {
		printk("[ERROR] BLE start failed (%d)\n", err);
	}

	err = adv_btn_init();
	if (err) {
		printk("[WARN ] Button 1 not available (%d) - no way to advertise\n", err);
	}

	err = radar_start();
	if (err) {
		printk("[ERROR] radar start failed (%d) - check uart1 and NRST wiring\n", err);
	}

	/* Absolute cadence rather than k_msleep(1000) at the end, so a slow NVS
	 * write does not stretch the record period. */
	next_tick = k_uptime_get() + 1000;

	while (1) {
		if (++sec >= STORE_PERIOD_S) {
			sec = 0;
			window_flush();
		}

		int64_t delay = next_tick - k_uptime_get();

		if (delay > 0) {
			k_msleep((int32_t)delay);
		}
		next_tick += 1000;
	}

	return 0;
}
