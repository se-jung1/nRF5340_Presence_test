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
/* Every byte the uart has ever handed us. The frame counter going quiet says
 * nothing about which end stopped - this says whether the radar is still
 * talking while the reassembler sits stuck. */
static uint32_t rx_total;

/*
 * The presence profile from the IWRL6432 repo, 22/22 accepted on this board
 * (xWRL6432 MMW Demo 05.05.03.00), with four changes:
 *   - guiMonitor: range profile off (512 B/frame nobody reads), tracker on,
 *     and twelve arguments rather than the documented eleven
 *   - sigProcChainCfg: motDetMode 3 rather than 2, so the major-motion chain
 *     runs as well as minor-motion presence
 *   - trackingCfg added, seven-argument form
 *   - baudRate removed, see the file header
 *
 * Argument counts here come from what the parser accepts on this image, not
 * from the published 05.05.04 guide and not from the board's own `help`. Both
 * are wrong about at least one line, in opposite directions, and the CLI
 * answers "Done" to a short guiMonitor without complaint - so a wrong count
 * shows up as behaviour that quietly differs, never as an error. Each line
 * below that was got wrong once carries the details.
 *
 * Do not drop sigProcChainCfg: it is what enables minor-motion (presence)
 * detection at all. Its 8-argument form is this firmware's; the published
 * 05.05.04 profile has 9.
 */
static const char *const cfg_lines[] = {
	"sensorStop 0",
	"channelCfg 7 3 0",
	"chirpComnCfg 8 0 0 256 4 23 2",
	"chirpTimingCfg 6 24 0 75 60.5",
	"frameCfg 2 4 600 4 250 0",
	/*
	 * <pointCloud> <rangeProfile> <noiseProfile> <rangeAzimuthHeatMap>
	 * <rangeDopplerHeatMap> <statsInfo> <presenceInfo> <adcSamples>
	 * <trackerInfo> <microDopplerInfo> <classifierInfo> <quickEvalInfo>
	 *
	 * Twelve, from this board's own `help`. The published 05.05.04 docs list
	 * eleven - no quickEvalInfo - and the CLI answers "Done" to either, so
	 * the short form looked fine and simply left the last field at whatever
	 * the parser had. Take the count from the firmware, not the docs.
	 */
	"guiMonitor 2 0 0 0 0 1 1 0 1 0 0 0",
	/*
	 * <azimuthFftSize> <elevationFftSize> <motDetMode> <coherentDoppler>
	 * <numFrmPerMinorMotProc> <numMinorMotionChirpsPerFrame>
	 * <forceMinorMotionVelocityToZero> <minorMotionVelocityInclusionThr>
	 *
	 * motDetMode: 1 = major motion only, 2 = minor motion only,
	 * 3 = auto (both). It used to be 2, copied from the IWRL6432 repo's
	 * presence profile, and that is what kept the tracker silent: the group
	 * tracker runs on the MAJOR motion point cloud, so in minor-only mode it
	 * has no input and TARGET_LIST is never emitted - the frame carries only
	 * 301/306/315. trackingCfg and guiMonitor's tracker flag were both being
	 * accepted the whole time; the tracker simply had nothing to track.
	 *
	 * 3 keeps minor-motion presence (a person sitting still still registers)
	 * and adds the major-motion chain that feeds the tracker.
	 */
	"sigProcChainCfg 64 4 3 2 4 4 0 0.5",
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
	 * <enable> <initialConfigParams> <maxNumPoints> <maxNumTracks>
	 * <maxRadialVelocity> <radialVelocityResolution> <deltaT>
	 * [<boresightFilteringEnable>]   - MMWAVE-L-SDK 05.05 demo guide
	 *
	 * Seven required, the eighth optional. Believe the guide here and not the
	 * board's own `help`, which prints a six-field summary
	 * (<maxDoppler> <framePeriod> in place of the last three) that the parser
	 * does not accept: sending exactly those six gets "Error: I...". The
	 * seven-field form is accepted. `help` is an abbreviation, not a spec.
	 *
	 * maxRadialVelocity is 10x m/s, radialVelocityResolution is mm/s, and
	 * deltaT is the frame period in ms, which has to match frameCfg (250).
	 *
	 * An earlier "1 2 250 20 0 250 0" - reverse engineered from that same
	 * `help` line - put 0 in maxRadialVelocity and asked for 20 tracks. The
	 * CLI answers "Done" and sensorStart then dies with "Tracker DPU config
	 * return error:-30907" (tracker DPU base -30900, -7 =
	 * EMAX_NUM_TRACKS_EXCEEDED).
	 *
	 * ponytail: the two velocity numbers are TI's, not derived from the chirp
	 * config above. They feed the tracker's motion model, so being off
	 * degrades tracking rather than breaking it - retune with TI's Sensing
	 * Estimator if fast walkers drop out.
	 */
	"trackingCfg 1 2 100 3 61.4 191.8 250",
	/*
	 * NOT sent, deliberately:
	 *
	 *   boundaryBox -3.5 3.5 0 7 0 3
	 *   staticBoundaryBox -3 3 0.5 6 0 3
	 *
	 * GTRACK only allocates a track inside boundaryBox, so on paper these
	 * are what the tracker needs. Both are accepted, sensorStart is accepted
	 * - and then the demo stops transmitting the instant the major motion
	 * chain first produces points. Reproduced twice on 05.05.03.00, both at
	 * "pts 2": the last frame prints, the uart byte counter freezes to the
	 * byte, and nothing comes back short of NRST. Without them the same
	 * build streams through point clouds of 15 without trouble.
	 *
	 * So the tracker allocation path takes this image down. Records carry
	 * SNTL_FLAG_NO_TRACKER and headcount stays a 0/1 floor until the EVM is
	 * reflashed with an image whose tracker survives - a much better trade
	 * than a sensor that dies the first time somebody walks past it.
	 */
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

	/*
	 * No frames at all this window means the sensor is not reporting, and
	 * the occupancy clock then has no basis: occ_since is only ever cleared
	 * from on_frame(), so with the stream dead it keeps running and dwell
	 * climbs forever. Observed on the bench - the demo stopped streaming and
	 * the node went on recording headcount 0, occupied 0 s, dwell 201 s, 231,
	 * 261 ... 711 and rising. A reader cannot tell that from a genuine long
	 * occupancy. Stop the clock; SNTL_FLAG_SENSOR_FAULT already marks why.
	 */
	if (!out->alive) {
		acc.occ_since = 0;
	}

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
		rx_total += n;
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
/* Bring-up diagnostics: what the last cli_wait() actually heard. 0 bytes means
 * nothing reaches uart1 RX at all (wiring, or EVM switch S1.4 still off);
 * bytes that are not the echoed command mean the baud rate is wrong. */
static uint32_t cli_rx_n;
/* Long enough to hold the echoed command plus the demo's error text after it;
 * at 41 the interesting half ("Error: Invalid ...") was always cut off. */
static char cli_rx_head[129];

/*
 * rx=0 says a byte never arrived; it cannot say whether that is because our TX
 * never reached the radar or because the radar's TX never reached us. So sample
 * the RX line itself while waiting. UARTE leaves the pin's GPIO input buffer
 * connected, so IN still reads the wire while the driver owns it.
 *
 *   HI only  - line idling high: the radar's TX is on this hole. Fault is on
 *              our TX side (wrong hole, or the radar is not hearing us).
 *   HI+LO    - the line is moving: bytes are being sent but not decoded, so
 *              suspect baud or framing rather than wiring.
 *   LO only  - nothing drives it. The overlay's bias-pull-up is off in this
 *              build, so a dead wire reads low against the radar's absence.
 *
 * ponytail: pin number duplicated from the overlay's uart1_radar RX psel.
 * Two places, one pin - move them together.
 */
#define RX_PIN 5   /* P1.05 */
static const struct device *const gpio_p1 = DEVICE_DT_GET(DT_NODELABEL(gpio1));
static uint8_t cli_rx_lvl;   /* bit0 = saw low, bit1 = saw high */

static int cli_wait(int timeout_ms)
{
	char buf[96];
	size_t fill = 0;
	int64_t deadline = k_uptime_get() + timeout_ms;
	int done = 0;

	buf[0] = '\0';
	cli_rx_n = 0;
	cli_rx_head[0] = '\0';
	cli_rx_lvl = 0;
	while (k_uptime_get() < deadline) {
		uint8_t b;

		if (ring_buf_get(&rx_rb, &b, 1) != 1) {
			cli_rx_lvl |= gpio_pin_get_raw(gpio_p1, RX_PIN) ? 2 : 1;
			k_msleep(2);
			continue;
		}
		if (cli_rx_n < sizeof(cli_rx_head) - 1) {
			cli_rx_head[cli_rx_n] = (b >= 0x20 && b < 0x7f) ? (char)b : '.';
			cli_rx_head[cli_rx_n + 1] = '\0';
		}
		cli_rx_n++;
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
	int ok;

	cli_drain();
	cli_write(line);
	cli_write("\n");
	ok = cli_wait(timeout_ms);
	if (!ok) {
		static const char *const lvl[] = { "?", "LO", "HI", "HI+LO" };

		printk("[RADAR] rx=%u rxpin=%s \"%s\"\n",
		       cli_rx_n, lvl[cli_rx_lvl & 3], cli_rx_head);
	}
	return ok;
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

	/*
	 * Print the TLV type list the first time a type we have not seen before
	 * turns up, so this is one line at startup and one more the moment the
	 * tracker finally emits TARGET_LIST. Printing only the first frame is
	 * not enough: the tracker stays quiet until it has a confirmed target,
	 * so its TLV appears seconds later, long after frame 0.
	 *
	 * ponytail: types are 301..315, so type-300 fits a u32 mask with room
	 * to spare. Anything outside that range folds into bit 0 and is
	 * reported once - fine for a bring-up aid, not a parser.
	 */
	{
		static uint32_t types_seen;
		struct tlv_header h;
		uint32_t off = TLV_HDR_LEN;
		uint32_t mask = 0;

		tlv_header_get(frame, &h);
		for (uint32_t i = 0; i < h.num_tlvs && off + 8 <= h.total_len; i++) {
			uint32_t t, l;

			memcpy(&t, frame + off, 4);
			memcpy(&l, frame + off + 4, 4);
			mask |= BIT((t - 300) & 31);
			off += 8 + l;
		}
		if (mask & ~types_seen) {
			types_seen |= mask;
			off = TLV_HDR_LEN;
			printk("[RADAR] %u tlvs, %u bytes, %u points:",
			       h.num_tlvs, h.total_len, h.num_detected);
			for (uint32_t i = 0; i < h.num_tlvs && off + 8 <= h.total_len; i++) {
				uint32_t t, l;

				memcpy(&t, frame + off, 4);
				memcpy(&l, frame + off + 4, 4);
				printk(" %u(%u B)", t, l);
				off += 8 + l;
			}
			printk("\n");
		}
	}

	/* ~1 Hz at the profile's 250 ms frame period. */
	if (++seen % 4 == 0) {
		struct tlv_header h;

		tlv_header_get(frame, &h);
		/* num_detected is the major-motion point count, and it is the
		 * tracker's only input - a steady 0 while someone walks means
		 * the major chain is producing nothing, which is a different
		 * problem from the tracker dropping the points it gets. */
		printk("[%6us] frame %u  %s %d  pts %u", store_now(), h.frame_no,
		       tracker ? "targets" : "presence", n, h.num_detected);
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

/*
 * Silence longer than this means the sensor is gone, not slow: the profile
 * runs at 250 ms per frame, so 10 s is forty missed frames. Long enough that a
 * busy NVS erase or a BLE dump cannot trip it, short enough to lose at most
 * one 30 s record to the restart.
 */
#define RADAR_STALL_MS 10000

static K_THREAD_STACK_DEFINE(radar_stack, RADAR_STACK);
static struct k_thread radar_thread_data;

static void radar_thread(void *a, void *b, void *c)
{
	static struct tlv_reasm re;   /* 2 KB - too big for the thread stack */
	uint8_t chunk[64];
	int64_t last_frame;

	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);

	while (1) {
		while (radar_configure() != 0) {
			printk("[RADAR] cfg failed - resetting and retrying in 5 s\n");
			k_msleep(5000);
		}
		printk("[START] radar streaming\n");

		tlv_reset(&re);
		last_frame = k_uptime_get();

		while (k_uptime_get() - last_frame < RADAR_STALL_MS) {
			uint32_t n = ring_buf_get(&rx_rb, chunk, sizeof(chunk));

			if (n == 0) {
				k_msleep(5);
				continue;
			}
			for (uint32_t i = 0; i < n; i++) {
				if (tlv_push(&re, chunk[i])) {
					on_frame(re.buf);
					tlv_reset(&re);
					last_frame = k_uptime_get();
				}
			}
		}

		/*
		 * The demo can stop dead - it did so reproducibly the moment the
		 * tracker allocation path was enabled - and nothing short of
		 * NRST brings it back, which is what radar_configure() starts
		 * with. Left alone the node goes on writing empty records
		 * forever; the bench run filled 14 minutes that way.
		 *
		 * The counters say which end failed: bytes frozen means the
		 * radar stopped, bytes still climbing with a non-zero reasm len
		 * means it is talking and we lost framing.
		 */
		printk("[RADAR] stalled %d s: %u bytes in, %u dropped, "
		       "reasm len=%u want=%u match=%u - reconfiguring\n",
		       RADAR_STALL_MS / 1000, rx_total, rx_dropped,
		       re.len, re.want, re.match);
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
