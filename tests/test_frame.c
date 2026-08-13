/*
 * Host self-check for the SNTL frame encoder. No board, no Zephyr.
 *
 *   gcc -I../src -o test_frame test_frame.c ../src/frame.c && ./test_frame
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "frame.h"

static void check_crc_vector(void)
{
	const char *v = "123456789";
	uint16_t crc = sntl_crc16(0xFFFF, (const uint8_t *)v, strlen(v));

	printf("CRC-16/CCITT-FALSE(\"123456789\") = 0x%04X\n", crc);
	assert(crc == 0x29B1);
}

static void check_frame_sizes(void)
{
	assert(sntl_frame_size(0) == 18);
	assert(sntl_frame_size(2) == 66);          /* 16 + 48 + 2 */
	assert(sntl_frame_size(2880) == 69138);    /* a full 24 h at 30 s */
}

static void check_header(void)
{
	uint8_t h[SNTL_HEADER_SIZE];

	sntl_encode_header(h, 2, "PRA3F92C");
	assert(h[0] == 0x53 && h[1] == 0x4E && h[2] == 0x54 && h[3] == 0x4C);
	assert(h[4] == 3);                          /* version: presence + air */
	assert(h[5] == 24);                         /* recordSize */
	assert(h[6] == 2 && h[7] == 0);             /* recordCount LE */
	assert(memcmp(h + 8, "PRA3F92C", 8) == 0);

	/* short id is zero padded, not left as garbage */
	sntl_encode_header(h, 0, "PR-A3F9");
	assert(memcmp(h + 8, "PR-A3F9\0", 8) == 0);
}

static void check_empty_frame(void)
{
	uint8_t f[18];
	uint16_t crc;

	sntl_encode_header(f, 0, "PRA3F92C");
	crc = sntl_crc16(0xFFFF, f, SNTL_HEADER_SIZE);
	f[16] = (uint8_t)(crc & 0xFF);
	f[17] = (uint8_t)(crc >> 8);

	assert(sntl_frame_size(0) == sizeof(f));
	assert(sntl_crc16(0xFFFF, f, SNTL_HEADER_SIZE) == crc);
}

static void check_record_encoding(void)
{
	struct sntl_record r = {
		.seq = 0x11223344, .ts = 0x55667788,
		.headcount = 3, .occ_s = 27, .dwell_s = 0x0403,
		.flags = SNTL_FLAG_RTC_UNSET | SNTL_FLAG_NO_TRACKER, .batt = 100,
		.pm25 = 123, .pm10 = 456, .temp = 2350, .rh = 4512, .voc = 1005,
	};
	uint8_t b[SNTL_RECORD_SIZE];

	sntl_encode_record(b, &r);
	assert(b[0] == 0x44 && b[1] == 0x33 && b[2] == 0x22 && b[3] == 0x11);
	assert(b[4] == 0x88 && b[5] == 0x77 && b[6] == 0x66 && b[7] == 0x55);
	assert(b[8] == 3 && b[9] == 27);
	assert(b[10] == 0x03 && b[11] == 0x04);     /* dwell_s LE */
	assert(b[12] == 0x24 && b[13] == 100);      /* RTC_UNSET | NO_TRACKER */
	assert(b[14] == 123 && b[15] == 0);         /* pm25 = 12.3 ug/m3 */
	assert(b[16] == 0xC8 && b[17] == 0x01);     /* pm10 = 45.6 ug/m3 */
	assert(b[18] == 0x2E && b[19] == 0x09);     /* temp = 23.50 C */
	assert(b[20] == 0xA0 && b[21] == 0x11);     /* rh = 45.12 %RH */
	assert(b[22] == 0xED && b[23] == 0x03);     /* voc index 100.5 */

	/* A sub-zero temperature has to survive the trip as two's complement,
	 * and it must not collide with the "not measured" sentinel. */
	r.temp = -1250;                             /* -12.50 C */
	r.rh = SNTL_AQ_UNKNOWN_S;
	sntl_encode_record(b, &r);
	assert(b[18] == 0x1E && b[19] == 0xFB);
	assert(b[20] == 0xFF && b[21] == 0x7F);
}

static void check_cmd_parsing(void)
{
	uint8_t good[] = { 0x01, 0x2A, 0x00, 0x00, 0x00 };
	uint8_t bad_op[] = { 0x02, 0x00, 0x00, 0x00, 0x00 };
	uint8_t bad_len[] = { 0x01, 0x00 };
	uint32_t seq = 0xDEADBEEF;

	assert(sntl_parse_cmd(good, sizeof(good), &seq) == 1 && seq == 42);
	assert(sntl_parse_cmd(bad_op, sizeof(bad_op), &seq) == 0);
	assert(sntl_parse_cmd(bad_len, sizeof(bad_len), &seq) == 0);
}

static void check_dump_range(void)
{
	uint32_t start;

	/* stored seq 0,1,2 -> next_seq = 3, oldest = 0 */
	assert(sntl_dump_range(0, 0, 3, &start) == 3 && start == 0);
	assert(sntl_dump_range(2, 0, 3, &start) == 1 && start == 2);
	assert(sntl_dump_range(3, 0, 3, &start) == 0);
	/* asking below the ring's oldest clamps up, it does not resend nothing */
	assert(sntl_dump_range(0, 100, 103, &start) == 3 && start == 100);
	/* empty store */
	assert(sntl_dump_range(0, 0, 0, &start) == 0);
}

int main(void)
{
	check_crc_vector();
	check_frame_sizes();
	check_header();
	check_empty_frame();
	check_record_encoding();
	check_cmd_parsing();
	check_dump_range();
	printf("all frame checks passed\n");
	return 0;
}
