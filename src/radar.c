/*
 * IWRL6432BOOST host link.
 *
 * Everything awkward here comes from one fact: the Presence demo multiplexes
 * the ASCII CLI and the binary TLV stream onto a SINGLE uart. So this file
 * talks text until sensorStart returns, and binary forever after.
 *
 * The four traps from the PC bring-up (see the IWRL6432 repo's tools/README.md)
 * all apply unchanged:
 *   1. one uart for CLI and data
 *   2. lowPowerCfg must be 0, or uart RX dies after sensorStart and only NRST
 *      brings it back
 *   3. the demo cannot be reconfigured in place - every line answers "Done" and
 *      then it silently never streams. Reset before every cfg.
 *   4. a whole line written at once loses characters; trickle it
 *   5. "no Error" is not success. Only an explicit "Done" counts.
 *
 * What is different from the PC version: no baudRate line. Zephyr's nrfx uart
 * driver has no 1250000 entry, and it is not needed - with the range profile
 * turned off a frame is a few hundred bytes at 4 fps, well under the ~11 kB/s
 * that 115200 carries.
 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/printk.h>
#include <errno.h>
#include <string.h>

#include "radar.h"
#include "store.h"   /* STORE_PERIOD_S - the window occ_s is expressed in */
#include "tlv.h"

#define RADAR_PROMPT       "mmwDemo:/>"
#define RADAR_MAX_TARGETS  20

/*
 * 4 KB of ring: an NVS append in the main loop can erase a page (~85 ms) and
 * the BLE dump thread can hold the CPU for a while, and at 115200 that is only
 * ~1 KB of arrivals. Overruns are counted and printed rather than hidden.
 */
#define RX_RING_SIZE  4096

static const struct device *const uart = DEVICE_DT_GET(DT_NODELABEL(uart1));
/* Active low, open drain: the EVM has its own reset circuit on this net and we
 * only ever pull it down. */
static const struct gpio_dt_spec nrst =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), radar_nrst_gpios);

RING_BUF_DECLARE(rx_rb, RX_RING_SIZE);
static uint32_t rx_dropped;

/*
 * The presence profile from the IWRL6432 repo, verified 25/25 on this firmware
 * (SDK 05.05.03.00), with three changes:
 *   - guiMonitor: range profile off (512 B/frame nobody reads), tracker on
 *   - trackingCfg added, 6-arg form because that is what this board's `help`
 *     prints. If it is rejected the sensor still streams presence and records
 *     carry SNTL_FLAG_NO_TRACKER - see radar_configure().
 *   - baudRate removed, see the file header
 *
 * Do not drop sigProcChainCfg: its motDetMode 2 is what enables minor-motion
 * (presence) detection at all. Its 8-argument form is this firmware's; the
 * published 05.05.04 profile has 9.
 */
static const char *const cfg_lines[] = {
	"sensorStop 0",
	"channelCfg 7 3 0",
	"chirpComnCfg 8 0 0 256 4 23 2",
	"chirpTimingCfg 6 24 0 75 60.5",
	"frameCfg 2 4 600 4 250 0",
	/* <pointCloud> <rangeProfile> <noiseProfile> <azHeatMap> <dopHeatMap>
	 * <stats> <presence> <adcSamples> <tracker> <microDoppler> <classifier> */
	"guiMonitor 2 0 0 0 0 1 1 0 1 0 0",
	"sigProcChainCfg 64 4 2 2 4 4 0 0.5",
	"cfarCfg 2 8 4 3 0 15 0 0.5 0 1 1 1",
	"aoaFovCfg -70 70 -60 60",
	"rangeSelCfg 0.25 7.5",
	"clutterRemoval 1",
	"compRangeBiasAndRxChanPhase 0.0 1.00000 0.00000 -1.00000 0.00000 1.00000 0.00000 -1.00000 0.00000 1.00000 0.00000 -1.00000 0.00000",
	"adcDataSource 0 adc_data_0001_CtestAdc6Ant.bin",
	"adcLogging 0",
	"lowPowerCfg 0",
	"factoryCalibCfg 1 0 36 3 0x1ff000",
	"sensorPosition 0 0 1.5 0 0",
	"mpdBoundaryBox 1 -3.5 3.5 0 7 0 3",
	"minorStateCfg 4 3 12 8 5 20 4 20",
	"clusterCfg 1 1 2",
	/*
	 * <enable> <paramSet> <numPoints> <numTracks> <maxDoppler> <framePeriod>
	 * plus a seventh argument the board's `help` does not list. Six args is
	 * what help prints and what every published profile uses, and this
	 * firmware rejects all of them with "Invalid usage"; seven is accepted.
	 * (cfarCfg has the same problem - help says 8, it takes 12.) The trailing
	 * 0 is unnamed, so it is left at 0.
	 */
	"trackingCfg 1 2 250 20 0 250 0",
	"sensorStart 0 0 0 0",
};

/* ---------------- accumulators ---------------- */
static struct {
	uint8_t  max_count;
	uint16_t frames;
	uint16_t occ_frames;
	bool     any_tracker;
	int64_t  occ_since;   /* uptime ms the current occupancy began, 0 = empty */
} acc;

K_MUTEX_DEFINE(acc_lock);

void radar_window_take(struct radar_window *out)
{
	int64_t held;

	k_mutex_lock(&acc_lock, K_FOREVER);

	out->alive     = acc.frames > 0;
	out->tracker   = acc.any_tracker;
	out->headcount = acc.max_count;
	/* Fraction of frames occupied, scaled to the window. Frame counts rather
	 * than a clock so a slow or stuttering sensor still reports honestly. */
	out->occ_s = acc.frames ? (uint8_t)((uint32_t)acc.occ_frames * STORE_PERIOD_S /
					    acc.frames)
				: 0;

	held = acc.occ_since ? (k_uptime_get() - acc.occ_since) / 1000 : 0;
	out->dwell_s = (held > UINT16_MAX) ? UINT16_MAX : (uint16_t)held;

	acc.max_count = 0;
	acc.frames = 0;
	acc.occ_frames = 0;
	acc.any_tracker = false;
	/* occ_since deliberately survives: dwell is the unbroken occupancy clock
	 * and must run across window boundaries. */

	k_mutex_unlock(&acc_lock);
}

/* ---------------- uart ---------------- */
static void uart_isr(const struct device *dev, void *user)
{
	uint8_t b[64];

	ARG_UNUSED(user);

	if (!uart_irq_update(dev)) {
		return;
	}
	while (uart_irq_rx_ready(dev)) {
		int n = uart_fifo_read(dev, b, sizeof(b));

		if (n <= 0) {
			break;
		}
		if (ring_buf_put(&rx_rb, b, n) < (uint32_t)n) {
			rx_dropped++;
		}
	}
}

static void cli_drain(void)
{
	uint8_t b;

	while (ring_buf_get(&rx_rb, &b, 1) == 1) {
	}
}

/* One character per millisecond. The demo echoes from a polling loop and drops
 * input if a whole line lands at once - silently, since the mangled command
 * just never answers. */
static void cli_write(const char *s)
{
	for (; *s != '\0'; s++) {
		uart_poll_out(uart, (unsigned char)*s);
		k_msleep(1);
	}
}

/*
 * Read until the prompt comes back. Returns 1 only if "Done" was seen: at the
 * wrong speed, or while the sensor is streaming, a reply contains neither
 * "Done" nor "Error", so absence of an error means nothing.
 */
static int cli_wait(int timeout_ms)
{
	char buf[96];
	size_t fill = 0;
	int64_t deadline = k_uptime_get() + timeout_ms;
	int done = 0;

	buf[0] = '\0';
	while (k_uptime_get() < deadline) {
		uint8_t b;

		if (ring_buf_get(&rx_rb, &b, 1) != 1) {
			k_msleep(2);
			continue;
		}
		buf[fill++] = (char)b;
		buf[fill] = '\0';

		if (!done && strstr(buf, "Done") != NULL) {
			done = 1;
		}
		if (strstr(buf, RADAR_PROMPT) != NULL) {
			return done;
		}
		if (fill >= sizeof(buf) - 1) {
			/* Slide, keeping a tail longer than either token so one
			 * straddling the boundary is not lost. */
			memmove(buf, buf + fill - 16, 16);
			fill = 16;
			buf[fill] = '\0';
		}
	}
	return done;
}

static int cli_line(const char *line, int timeout_ms)
{
	cli_drain();
	cli_write(line);
	cli_write("\n");
	return cli_wait(timeout_ms);
}

/* ---------------- bring-up ---------------- */
/*
 * Both resets, in that order, because each covers what the other cannot.
 *
 * sensorWarmRst over the CLI is enough for the normal case and works with only
 * the two UART wires connected - so the NRST wire is optional for bring-up.
 * It cannot help once the demo has hung, though: a failed sensorStart leaves it
 * deaf to the CLI and only NRST (or a USB replug) brings it back.
 *
 * Pulsing an unconnected NRST pin is harmless, so there is no build flag for
 * "is the wire there" - do both every time.
 */
static void radar_reset(void)
{
	cli_line("sensorStop 0", 2000);
	cli_line("sensorWarmRst 0", 2000);   /* reboots; no reply comes back */

	gpio_pin_set_dt(&nrst, 1);   /* asserted = low */
	k_msleep(20);
	gpio_pin_set_dt(&nrst, 0);

	k_msleep(2000);              /* demo boots and prints its banner */
	cli_drain();
	cli_line("", 500);           /* terminate any half line in its parser */
}

/*
 * Reset, then push the whole cfg. Returns 0 once sensorStart is accepted.
 *
 * A rejected trackingCfg is not fatal: the likely cause is that the flashed
 * image is the plain Presence_Demo with no tracker DPU, and the sensor still
 * streams device-side presence zones. Records then carry SNTL_FLAG_NO_TRACKER,
 * which on_frame() decides per frame from whether TARGET_LIST actually shows
 * up - so nothing here has to remember the outcome.
 */
static int radar_configure(void)
{
	radar_reset();

	for (size_t i = 0; i < ARRAY_SIZE(cfg_lines); i++) {
		const char *ln = cfg_lines[i];
		bool is_start = strncmp(ln, "sensorStart", 11) == 0;

		if (cli_line(ln, is_start ? 15000 : 3000)) {
			printk("[RADAR] ok    %s\n", ln);
			continue;
		}

		printk("[RADAR] FAIL  %s\n", ln);
		if (is_start) {
			return -EIO;
		}
	}
	return 0;
}

/* ---------------- frame handling ---------------- */
static void on_frame(const uint8_t *frame)
{
	struct tlv_target tgt[RADAR_MAX_TARGETS];
	uint8_t zones[8];
	const uint8_t *p;
	uint32_t len;
	int n = 0;
	bool tracker = false;
	static uint32_t seen;

	p = tlv_find(frame, TLV_TARGET_LIST, &len);
	if (p != NULL) {
		tracker = true;
		n = tlv_targets(p, len, tgt, RADAR_MAX_TARGETS);
	} else {
		/* No tracker in this build. Device-side presence zones only say
		 * "something is moving", so headcount becomes a floor of 0 or 1. */
		p = tlv_find(frame, TLV_ENHANCED_PRESENCE, &len);
		if (p != NULL) {
			int nz = tlv_presence(p, len, zones, (int)ARRAY_SIZE(zones));

			for (int z = 0; z < nz; z++) {
				if (zones[z] != TLV_ZONE_NONE) {
					n = 1;
					break;
				}
			}
		}
	}

	k_mutex_lock(&acc_lock, K_FOREVER);
	acc.frames++;
	if (tracker) {
		acc.any_tracker = true;
	}
	if (n > 0) {
		acc.occ_frames++;
		if (n > acc.max_count) {
			acc.max_count = (uint8_t)n;
		}
		if (acc.occ_since == 0) {
			acc.occ_since = k_uptime_get();
		}
	} else {
		/* ponytail: one empty frame breaks the dwell clock. If it turns
		 * out to flicker in the real room, require N empty frames here. */
		acc.occ_since = 0;
	}
	k_mutex_unlock(&acc_lock);

	/* ~1 Hz at the profile's 250 ms frame period. */
	if (++seen % 4 == 0) {
		struct tlv_header h;

		tlv_header_get(frame, &h);
		printk("[%6us] frame %u  %s %d", store_now(), h.frame_no,
		       tracker ? "targets" : "presence", n);
		for (int i = 0; i < n && tracker && i < 4; i++) {
			printk("  #%u(%d,%dcm)", tgt[i].tid,
			       (int)(tgt[i].x * 100.0f), (int)(tgt[i].y * 100.0f));
		}
		if (rx_dropped) {
			printk("   [rx dropped %u]", rx_dropped);
		}
		printk("\n");
	}
}

/* ---------------- thread ---------------- */
#define RADAR_STACK    2560
#define RADAR_PRIORITY 6

static K_THREAD_STACK_DEFINE(radar_stack, RADAR_STACK);
static struct k_thread radar_thread_data;

static void radar_thread(void *a, void *b, void *c)
{
	static struct tlv_reasm re;   /* 2 KB - too big for the thread stack */
	uint8_t chunk[64];

	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	while (radar_configure() != 0) {
		printk("[RADAR] cfg failed - resetting and retrying in 5 s\n");
		k_msleep(5000);
	}
	printk("[START] radar streaming\n");

	tlv_reset(&re);
	while (1) {
		uint32_t n = ring_buf_get(&rx_rb, chunk, sizeof(chunk));

		if (n == 0) {
			k_msleep(5);
			continue;
		}
		for (uint32_t i = 0; i < n; i++) {
			if (tlv_push(&re, chunk[i])) {
				on_frame(re.buf);
				tlv_reset(&re);
			}
		}
	}
}

int radar_start(void)
{
	int err;

	if (!device_is_ready(uart)) {
		return -ENODEV;
	}
	if (!gpio_is_ready_dt(&nrst)) {
		return -ENODEV;
	}
	err = gpio_pin_configure_dt(&nrst, GPIO_OUTPUT_INACTIVE | GPIO_OPEN_DRAIN);
	if (err) {
		return err;
	}

	uart_irq_callback_user_data_set(uart, uart_isr, NULL);
	uart_irq_rx_enable(uart);

	k_thread_create(&radar_thread_data, radar_stack, K_THREAD_STACK_SIZEOF(radar_stack),
			radar_thread, NULL, NULL, NULL, RADAR_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&radar_thread_data, "radar");
	return 0;
}
