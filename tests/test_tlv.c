/*
 * Host self-check for the IWRL6432 frame reassembler and TLV decoder.
 * No board, no radar.
 *
 *   gcc -I../src -o test_tlv test_tlv.c ../src/tlv.c && ./test_tlv
 *
 * The synthetic frame mirrors the one the Python decoder in the IWRL6432 repo
 * self-checks against, so both sides agree on the layouts.
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "tlv.h"

static uint8_t buf[4096];
static size_t fill;

static void put_u32(uint32_t v)
{
	buf[fill++] = (uint8_t)v;
	buf[fill++] = (uint8_t)(v >> 8);
	buf[fill++] = (uint8_t)(v >> 16);
	buf[fill++] = (uint8_t)(v >> 24);
}

static void put_f32(float f)
{
	uint32_t v;

	memcpy(&v, &f, sizeof(v));
	put_u32(v);
}

/* Two 3D targets, ids 9 and 10. */
static size_t target_list_off, target_list_len;

static size_t build_frame(uint32_t frame_no, int with_targets)
{
	size_t start = fill;
	size_t len_at;

	memcpy(buf + fill, tlv_magic, TLV_MAGIC_LEN);
	fill += TLV_MAGIC_LEN;
	put_u32(0x05050300);          /* version */
	len_at = fill;
	put_u32(0);                   /* totalPacketLen, patched below */
	put_u32(0x000a6432);          /* platform */
	put_u32(frame_no);
	put_u32(0);                   /* timeCpuCycles */
	put_u32(1);                   /* numDetectedObj */
	put_u32(with_targets ? 3 : 2);
	put_u32(0xFFFFFFFF);          /* subFrameNumber */

	/* stats: 24 zero bytes */
	put_u32(TLV_STATS);
	put_u32(24);
	memset(buf + fill, 0, 24);
	fill += 24;

	if (with_targets) {
		target_list_len = 2 * 112;
		put_u32(TLV_TARGET_LIST);
		put_u32((uint32_t)target_list_len);
		target_list_off = fill;
		for (int t = 0; t < 2; t++) {
			put_u32(t == 0 ? 9 : 10);
			put_f32(t == 0 ? 1.5f : -2.0f);   /* x */
			put_f32(t == 0 ? 3.0f : 5.0f);    /* y */
			put_f32(1.2f);                    /* z */
			for (int i = 0; i < 24; i++) {    /* rest of the 27 floats */
				put_f32(0.0f);
			}
		}
	}

	/* enhanced presence: 4 zones, states minor/none/major/none */
	put_u32(TLV_ENHANCED_PRESENCE);
	put_u32(2);
	buf[fill++] = 4;
	buf[fill++] = 0x21;   /* 00 10 00 01 from the LSB up */

	buf[len_at + 0] = (uint8_t)(fill - start);
	buf[len_at + 1] = (uint8_t)((fill - start) >> 8);
	buf[len_at + 2] = 0;
	buf[len_at + 3] = 0;
	return fill - start;
}

/* Feed bytes one at a time, collecting every frame the reassembler completes. */
static int feed(struct tlv_reasm *r, const uint8_t *data, size_t len,
		uint8_t frames[][TLV_MAX_FRAME], uint32_t *lens, int max)
{
	int n = 0;

	for (size_t i = 0; i < len; i++) {
		if (tlv_push(r, data[i]) && n < max) {
			memcpy(frames[n], r->buf, r->len);
			lens[n] = r->len;
			n++;
			tlv_reset(r);
		}
	}
	return n;
}

static struct tlv_reasm re;
static uint8_t got[4][TLV_MAX_FRAME];
static uint32_t got_len[4];

static void check_decode(const uint8_t *f, uint32_t frame_no, int with_targets)
{
	struct tlv_header h;
	struct tlv_target tgt[8];
	uint8_t zones[8];
	const uint8_t *p;
	uint32_t len;
	int n;

	tlv_header_get(f, &h);
	assert(h.version == 0x05050300);
	assert(h.platform == 0x000a6432);
	assert(h.frame_no == frame_no);
	assert(h.num_tlvs == (with_targets ? 3u : 2u));

	p = tlv_find(f, TLV_ENHANCED_PRESENCE, &len);
	assert(p != NULL && len == 2);
	n = tlv_presence(p, len, zones, 8);
	assert(n == 4);
	assert(zones[0] == TLV_ZONE_MINOR && zones[1] == TLV_ZONE_NONE);
	assert(zones[2] == TLV_ZONE_MAJOR && zones[3] == TLV_ZONE_NONE);

	p = tlv_find(f, TLV_TARGET_LIST, &len);
	if (!with_targets) {
		assert(p == NULL && len == 0);
		return;
	}
	assert(p != NULL && len == 2 * 112);
	n = tlv_targets(p, len, tgt, 8);
	assert(n == 2);
	assert(tgt[0].tid == 9 && tgt[1].tid == 10);
	assert(tgt[0].x > 1.49f && tgt[0].x < 1.51f);
	assert(tgt[1].y > 4.99f && tgt[1].y < 5.01f);
	assert(tgt[0].z > 1.19f && tgt[0].z < 1.21f);

	/* A type that is not in the frame must not be invented. */
	p = tlv_find(f, TLV_POINTS, &len);
	assert(p == NULL && len == 0);
}

static void check_stream(void)
{
	int n;

	fill = 0;
	build_frame(7, 1);
	build_frame(8, 0);
	/* a partial magic at the tail must not produce a frame */
	memcpy(buf + fill, tlv_magic, 3);
	fill += 3;

	tlv_reset(&re);
	n = feed(&re, buf, fill, got, got_len, 4);
	assert(n == 2);
	check_decode(got[0], 7, 1);
	check_decode(got[1], 8, 0);
}

static void check_leading_garbage(void)
{
	uint8_t noise[] = { 0xAA, 0x02, 0x01, 0x04, 0x00, 0x02, 0x02, 0x01 };
	int n;

	/* Garbage that contains a partial magic, then the real thing. */
	fill = 0;
	memcpy(buf, noise, sizeof(noise));
	fill = sizeof(noise);
	build_frame(42, 1);

	tlv_reset(&re);
	n = feed(&re, buf, fill, got, got_len, 4);
	assert(n == 1);
	check_decode(got[0], 42, 1);
}

static void check_bogus_length(void)
{
	size_t first;
	int n;

	fill = 0;
	first = build_frame(1, 0);
	/* Claim a totalPacketLen far past the buffer: the frame must be dropped
	 * and the next real one still found. */
	buf[8 + 4 + 0] = 0x9F;
	buf[8 + 4 + 1] = 0x86;
	buf[8 + 4 + 2] = 0x01;
	(void)first;
	build_frame(99, 1);

	tlv_reset(&re);
	n = feed(&re, buf, fill, got, got_len, 4);
	assert(n == 1);
	check_decode(got[0], 99, 1);
}

static void check_truncated_tlv(void)
{
	const uint8_t *p;
	uint32_t len;

	fill = 0;
	build_frame(5, 1);
	/* Make the stats TLV claim more than the frame holds. tlv_find must stop
	 * rather than walk off the end. */
	buf[TLV_HDR_LEN + 4] = 0xFF;
	buf[TLV_HDR_LEN + 5] = 0xFF;
	p = tlv_find(buf, TLV_ENHANCED_PRESENCE, &len);
	assert(p == NULL && len == 0);
}

static void check_2d_targets(void)
{
	struct tlv_target tgt[4];
	int n;

	/* GTRACK_2D build: u32 tid + 18 floats = 76 bytes, no z. */
	fill = 0;
	put_u32(4);
	put_f32(-1.25f);
	put_f32(2.5f);
	for (int i = 0; i < 16; i++) {
		put_f32(0.0f);
	}
	assert(fill == 76);

	n = tlv_targets(buf, (uint32_t)fill, tgt, 4);
	assert(n == 1);
	assert(tgt[0].tid == 4);
	assert(tgt[0].x < -1.24f && tgt[0].x > -1.26f);
	assert(tgt[0].y > 2.49f && tgt[0].y < 2.51f);
	assert(tgt[0].z == 0.0f);

	/* A length matching neither struct decodes to nothing, not garbage. */
	assert(tlv_targets(buf, 33, tgt, 4) == 0);
	assert(tlv_targets(NULL, 0, tgt, 4) == 0);
}

int main(void)
{
	check_stream();
	check_leading_garbage();
	check_bogus_length();
	check_truncated_tlv();
	check_2d_targets();
	printf("all tlv checks passed\n");
	return 0;
}
