/*
 * IWRL6432 UART output: frame reassembly + TLV decode.
 *
 * Deliberately plain C (stdint/string only, no Zephyr) so tests/test_tlv.c can
 * build and run on the host. Do not add Zephyr includes here.
 *
 * Layouts transcribed from MMWAVE_L_SDK 05.05.04.02, same sources the Python
 * decoder in the IWRL6432 repo used:
 *   MmwDemo_output_message_header      motion_detect.h:1350   magic[8] + 8x u32
 *   trackerProc_Target (GTRACK_3D)     trackerproc.h:261      u32 + 27x f32
 *   enhanced presence payload          motion_detect.c:1216   byte 0 = zone
 *       count, then 2 bits of state per zone from the LSB up
 */
#ifndef TLV_H_
#define TLV_H_

#include <stdint.h>
#include <stddef.h>

#define TLV_MAGIC_LEN   8
#define TLV_HDR_LEN     40

/* Biggest frame we will hold. The tracking profile with the range profile
 * turned off runs a few hundred bytes; 2 KB is headroom, and anything claiming
 * more than this is treated as a desync and resynced past. */
#define TLV_MAX_FRAME   2048

/* TLV types (SDK: MMWDEMO_OUTPUT_MSG_EXT_* = 300 + n) */
#define TLV_POINTS              301
#define TLV_STATS               306
#define TLV_PRESENCE_INFO       307
#define TLV_TARGET_LIST         308
#define TLV_TARGET_INDEX        309
#define TLV_ENHANCED_PRESENCE   315

/* Zone states in the enhanced presence payload. */
#define TLV_ZONE_NONE   0
#define TLV_ZONE_MINOR  1
#define TLV_ZONE_MAJOR  2

extern const uint8_t tlv_magic[TLV_MAGIC_LEN];

/*
 * Streaming reassembler. Feed it one byte at a time; it returns 1 when buf
 * holds one complete frame of `len` bytes.
 *
 * Framing trusts the header's own totalPacketLen rather than scanning for the
 * next magic word, so a magic-looking byte pair inside a payload cannot
 * desync us. A bogus length falls back to hunting for the next magic.
 */
struct tlv_reasm {
	uint8_t  buf[TLV_MAX_FRAME];
	uint32_t len;    /* bytes in buf */
	uint32_t want;   /* full frame length once the header is in, else 0 */
	uint8_t  match;  /* magic bytes matched so far while hunting */
};

void tlv_reset(struct tlv_reasm *r);
int  tlv_push(struct tlv_reasm *r, uint8_t byte);

struct tlv_header {
	uint32_t version;
	uint32_t total_len;
	uint32_t platform;
	uint32_t frame_no;
	uint32_t cpu_cycles;
	uint32_t num_detected;
	uint32_t num_tlvs;
	uint32_t sub_frame;
};

void tlv_header_get(const uint8_t *frame, struct tlv_header *out);

/* Returns the payload of the first TLV of `type`, or NULL. Stops cleanly on a
 * truncated or lying length. */
const uint8_t *tlv_find(const uint8_t *frame, uint32_t type, uint32_t *len_out);

struct tlv_target {
	uint32_t tid;
	float x, y, z;   /* metres; +y out of the antenna face, +x right, +z up */
};

/*
 * Tracker output. Picks the struct the payload length divides by, so a
 * GTRACK_2D firmware build decodes too (z reads back 0 there).
 * Returns the number of targets written, never more than max.
 */
int tlv_targets(const uint8_t *payload, uint32_t len, struct tlv_target *out, int max);

/* Enhanced presence -> one state byte per zone, zone 1 first. Returns count. */
int tlv_presence(const uint8_t *payload, uint32_t len, uint8_t *out, int max);

#endif /* TLV_H_ */
