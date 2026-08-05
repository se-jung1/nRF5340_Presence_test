#include <string.h>

#include "tlv.h"

/* {0x0102, 0x0304, 0x0506, 0x0708} little-endian on the wire */
const uint8_t tlv_magic[TLV_MAGIC_LEN] = { 2, 1, 4, 3, 6, 5, 8, 7 };

static uint32_t rd_u32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* The SDK writes IEEE-754 floats little-endian, which is also how both the
 * Cortex-M33 and any host we test on store them - but memcpy rather than a
 * pointer cast, because the payload is not 4-byte aligned. */
static float rd_f32(const uint8_t *p)
{
	uint32_t v = rd_u32(p);
	float f;

	memcpy(&f, &v, sizeof(f));
	return f;
}

void tlv_reset(struct tlv_reasm *r)
{
	r->len = 0;
	r->want = 0;
	r->match = 0;
}

int tlv_push(struct tlv_reasm *r, uint8_t byte)
{
	/* hunting for the magic word */
	if (r->len == 0) {
		if (byte == tlv_magic[r->match]) {
			r->match++;
			if (r->match == TLV_MAGIC_LEN) {
				memcpy(r->buf, tlv_magic, TLV_MAGIC_LEN);
				r->len = TLV_MAGIC_LEN;
				r->want = 0;
				r->match = 0;
			}
		} else {
			/* Restart, but not blindly at 0: the byte that broke the
			 * match may itself begin a new one (magic has repeats). */
			r->match = (byte == tlv_magic[0]) ? 1 : 0;
		}
		return 0;
	}

	r->buf[r->len++] = byte;

	if (r->want == 0) {
		if (r->len < TLV_HDR_LEN) {
			return 0;
		}
		r->want = rd_u32(r->buf + 12);   /* totalPacketLen */
		if (r->want < TLV_HDR_LEN || r->want > TLV_MAX_FRAME) {
			tlv_reset(r);   /* bogus length - resync on the next magic */
			return 0;
		}
	}

	if (r->len < r->want) {
		return 0;
	}

	r->len = r->want;
	r->want = 0;
	return 1;
}

void tlv_header_get(const uint8_t *frame, struct tlv_header *out)
{
	out->version      = rd_u32(frame + 8);
	out->total_len    = rd_u32(frame + 12);
	out->platform     = rd_u32(frame + 16);
	out->frame_no     = rd_u32(frame + 20);
	out->cpu_cycles   = rd_u32(frame + 24);
	out->num_detected = rd_u32(frame + 28);
	out->num_tlvs     = rd_u32(frame + 32);
	out->sub_frame    = rd_u32(frame + 36);
}

const uint8_t *tlv_find(const uint8_t *frame, uint32_t type, uint32_t *len_out)
{
	uint32_t total = rd_u32(frame + 12);
	uint32_t off = TLV_HDR_LEN;

	while (off + 8 <= total) {
		uint32_t t = rd_u32(frame + off);
		uint32_t ln = rd_u32(frame + off + 4);

		if (ln > total || off + 8 + ln > total) {
			break;   /* truncated or lying length */
		}
		if (t == type) {
			*len_out = ln;
			return frame + off + 8;
		}
		off += 8 + ln;
	}
	*len_out = 0;
	return NULL;
}

/* GTRACK_3D: u32 tid + 27 floats. GTRACK_2D: u32 tid + 18 floats. */
#define TARGET_3D_SIZE  112
#define TARGET_2D_SIZE  76

int tlv_targets(const uint8_t *payload, uint32_t len, struct tlv_target *out, int max)
{
	uint32_t size;
	int n = 0;

	if (payload == NULL || len == 0) {
		return 0;
	}
	if (len % TARGET_3D_SIZE == 0) {
		size = TARGET_3D_SIZE;
	} else if (len % TARGET_2D_SIZE == 0) {
		size = TARGET_2D_SIZE;
	} else {
		return 0;
	}

	for (uint32_t off = 0; off + size <= len && n < max; off += size) {
		const uint8_t *t = payload + off;

		out[n].tid = rd_u32(t);
		out[n].x = rd_f32(t + 4);
		out[n].y = rd_f32(t + 8);
		out[n].z = (size == TARGET_3D_SIZE) ? rd_f32(t + 12) : 0.0f;
		n++;
	}
	return n;
}

int tlv_presence(const uint8_t *payload, uint32_t len, uint8_t *out, int max)
{
	int n;

	if (payload == NULL || len < 2) {
		return 0;
	}
	n = payload[0];
	if (n > max) {
		n = max;
	}
	for (int z = 0; z < n; z++) {
		uint32_t byte_idx = 1 + (uint32_t)(z / 4);

		out[z] = (byte_idx < len) ? ((payload[byte_idx] >> (2 * (z % 4))) & 0x3) : 0;
	}
	return n;
}
