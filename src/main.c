/*
 * Presence + air quality node: IWRL6432BOOST mmWave radar over uart1, Sensirion
 * SEN5x over i2c2.
 *
 * - one record per STORE_PERIOD_S into flash (24 h ring), handed out over BLE
 * - the radar does all the signal processing on its own Cortex-M4F and sends
 *   only results, so this core does framing, counting and storage - nothing
 *   here is real-time critical
 * - the two sensors fail independently and each has its own flag; neither
 *   failure suppresses the other's numbers
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
#include "sen5x.h"
#include "store.h"

/* DK "Button 1" (P0.23) - opens the BLE advertising window on press. */
static const struct gpio_dt_spec adv_btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static struct gpio_callback adv_btn_cb;

/* DK "Button 2" - held at boot, runs probe_mode() instead of the app. */
static const struct gpio_dt_spec probe_btn = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);

/*
 * Bring-up aid: find the right hole on the EVM's J8 header without a
 * multimeter. Reconfigures the uart1 RX pin as a plain input with a pull-down
 * and reports its level, so a hole that reads HIGH is one something is
 * actively driving. On a running EVM that is only the radar's TX, which idles
 * high - ground pins and the DNP pins all read low against the pull-down.
 * Touch the holes with the RX wire and watch.
 *
 * The uart1 driver has already claimed this pin through pinctrl by now; the
 * GPIO config below rewrites the same PIN_CNF register, and radar_start() is
 * never reached in this mode, so nothing fights over it.
 *
 * ponytail: pin hardcoded to match the uart1_radar RX psel in the overlay.
 * One pin used in one place - move both together if the RX pin moves.
 */
#define PROBE_PIN 5   /* P1.05, the uart1 RX pin */

static void probe_mode(void)
{
	const struct device *p1 = DEVICE_DT_GET(DT_NODELABEL(gpio1));
	int last = -1;

	printk("[PROBE] P1.05 as input with pull-down.\n");
	printk("[PROBE] touch J8 holes with the RX wire - HIGH means driven.\n");

	if (!device_is_ready(p1) ||
	    gpio_pin_configure(p1, PROBE_PIN, GPIO_INPUT | GPIO_PULL_DOWN)) {
		printk("[PROBE] cannot claim the pin\n");
		return;
	}
	while (1) {
		int v = gpio_pin_get_raw(p1, PROBE_PIN);

		if (v != last) {
			printk("[PROBE] %s\n",
			       v ? "HIGH  <-- driven, this is the hole" : "low");
			last = v;
		}
		k_msleep(200);
	}
}

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
	struct sen5x_window aq;
	struct sntl_record rec = { 0 };
	int err;

	radar_window_take(&w);
	sen5x_window_take(&aq);

	rec.headcount = w.headcount;
	rec.occ_s = w.occ_s;
	rec.dwell_s = w.dwell_s;

	rec.pm25 = aq.pm25;
	rec.pm10 = aq.pm10;
	rec.temp = aq.temp;
	rec.rh = aq.rh;
	rec.voc = aq.voc;

	if (!w.alive) {
		/* Tag it, never drop it - this is safety-management data. */
		rec.flags |= SNTL_FLAG_SENSOR_FAULT;
	}
	if (!aq.alive) {
		rec.flags |= SNTL_FLAG_AQ_FAULT;
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
	printk("[STORE] %us  headcount %u%s  occupied %us/%us  dwell %us",
	       rec.ts, rec.headcount, w.tracker ? "" : " (no tracker)",
	       rec.occ_s, STORE_PERIOD_S, rec.dwell_s);

	if (rec.pm25 == SNTL_AQ_UNKNOWN_U) {
		printk("  air: no data\n");
	} else {
		/* Split the sign off before dividing: -0.43 C is temp = -43, and
		 * -43/100 is 0, so "%d.%02d" would print it as 0.43. */
		int t = (rec.temp == SNTL_AQ_UNKNOWN_S) ? 0 : rec.temp;
		int ta = (t < 0) ? -t : t;

		printk("  pm2.5 %u.%u  pm10 %u.%u  %s%d.%02d C  %d.%d%%RH",
		       rec.pm25 / 10, rec.pm25 % 10, rec.pm10 / 10, rec.pm10 % 10,
		       (t < 0) ? "-" : "", ta / 100, ta % 100,
		       (rec.rh == SNTL_AQ_UNKNOWN_S) ? 0 : rec.rh / 100,
		       (rec.rh == SNTL_AQ_UNKNOWN_S) ? 0 : (rec.rh / 10) % 10);

		/* A SEN50 has no gas sensor, so this stays "--" there for good.
		 * On a SEN54/55 it climbs from 0 to about 100 over the first few
		 * minutes - that is the index algorithm learning its baseline,
		 * not a fault. */
		if (rec.voc == SNTL_AQ_UNKNOWN_S) {
			printk("  voc --");
		} else {
			printk("  voc %d.%d", rec.voc / 10, rec.voc % 10);
		}
		printk("%s\n", (rec.flags & SNTL_FLAG_AQ_FAULT) ? "  AQ_FAULT" : "");
	}
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

	/* gpio_pin_get_dt() is logical, and the DK's buttons are ACTIVE_LOW, so
	 * held reads 1. Not held: fall through and run the app as usual. */
	if (gpio_is_ready_dt(&probe_btn) &&
	    gpio_pin_configure_dt(&probe_btn, GPIO_INPUT) == 0 &&
	    gpio_pin_get_dt(&probe_btn) == 1) {
		probe_mode();   /* never returns */
	}

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

	/* Not fatal either: a node with a dead SEN5x still records occupancy,
	 * and every record it writes says so through SNTL_FLAG_AQ_FAULT. */
	err = sen5x_start();
	if (err) {
		printk("[ERROR] SEN5x start failed (%d) - air quality will read unknown\n", err);
	}

	/* Absolute cadence rather than k_msleep(1000) at the end, so a slow NVS
	 * write does not stretch the record period. */
	next_tick = k_uptime_get() + 1000;

	while (1) {
		/* ~20 ms on the bus; the absolute cadence below absorbs it. */
		sen5x_poll();

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
