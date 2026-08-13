/*
 * SNTL wire frame - node to app. Version 3: presence + air quality.
 *
 * Same envelope throughout the family - magic, header, fixed records, trailing
 * CRC. The version byte says which record shape follows, and it has to: v1 (the
 * air-quality node, nRF5340_Wearable) and v2 (presence only) are both 14 bytes,
 * so length alone will not tell them apart.
 *
 * v3 = v2's 14 bytes unchanged, with the SEN5x fields appended. A reader that
 * already decodes v2 keeps every offset it has and only gains a tail; one that
 * does not care about air quality can read a v3 record as a v2 record by
 * ignoring bytes 14..23.
 *
 * Deliberately plain C (stdint/string only, no Zephyr) so tests/test_frame.c
 * can build and run on the host. Do not add Zephyr includes here.
 *
 * Frame layout, all integers little-endian:
 *   header 16 B | record 24 B x recordCount | crc16 2 B
 * The CRC covers everything from the first header byte up to (not including)
 * the CRC itself.
 */
#ifndef FRAME_H_
#define FRAME_H_

#include <stdint.h>
#include <stddef.h>

#define SNTL_MAGIC          "SNTL"
#define SNTL_VERSION        3
#define SNTL_RECORD_SIZE    24
#define SNTL_HEADER_SIZE    16
#define SNTL_CRC_SIZE       2
#define SNTL_DEVICE_ID_LEN  8

/*
 * "Not measured", passed straight through from the SEN5x. A SEN50 has no
 * RH/T/gas and a SEN54 has no NOx; both answer 0xFFFF for what they lack, as
 * does any of them before its first conversion. Signed fields use INT16_MAX
 * because 0xFFFF reads as -1 there, and -0.01 C is a plausible temperature.
 * Mirrors SEN5X_UNKNOWN_U/_S - kept spelled out here so a reader of the wire
 * format does not have to go find the driver.
 */
#define SNTL_AQ_UNKNOWN_U   0xFFFF
#define SNTL_AQ_UNKNOWN_S   0x7FFF

/* Control characteristic command: 0x01 then uint32 LE firstSeq. */
#define SNTL_CMD_DUMP       0x01
#define SNTL_CMD_LEN        5

/* record flags */
#define SNTL_FLAG_SENSOR_FAULT        0x01   /* no radar frames in the window */
#define SNTL_FLAG_LOW_BATTERY         0x02
#define SNTL_FLAG_RTC_UNSET           0x04
#define SNTL_FLAG_CALIBRATING         0x08
/*
 * The tracker was not running, so headcount is a floor, not a count: it is 1
 * whenever the device-side presence zones report any motion and 0 otherwise.
 * The server must not add these up as people.
 */
#define SNTL_FLAG_NO_TRACKER          0x20
/*
 * The SEN5x did not answer this window, or it did and its status register
 * reported a fan or laser fault. Separate from SENSOR_FAULT, which is the
 * radar: one node now carries two sensors that fail independently, and a dead
 * fan reads as clean air, so this cannot share a bit with anything.
 * The PM fields are still sent when this is set - tag, never drop.
 */
#define SNTL_FLAG_AQ_FAULT            0x40

struct sntl_record {
	uint32_t seq;
	uint32_t ts;         /* UTC unix seconds; RTC_UNSET flag if not trustworthy */
	uint8_t  headcount;  /* most people seen at once during the window */
	uint8_t  occ_s;      /* seconds of the window that were occupied, 0..255 */
	uint16_t dwell_s;    /* unbroken occupancy at the end of the window */
	uint8_t  flags;
	uint8_t  batt;       /* percent, 0..100 */
	/* --- v3, SEN5x. Peak over the window for PM, last sample for the rest. --- */
	uint16_t pm25;       /* 0.1 ug/m3 */
	uint16_t pm10;       /* 0.1 ug/m3 */
	int16_t  temp;       /* 0.01 C */
	int16_t  rh;         /* 0.01 %RH */
	int16_t  voc;        /* 0.1 VOC index */
};

/*
 * CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no final XOR.
 * Call with seed 0xFFFF and chain across chunks to CRC a streamed frame.
 * Check vector: "123456789" -> 0x29B1.
 */
uint16_t sntl_crc16(uint16_t seed, const uint8_t *data, size_t len);

/* out must be SNTL_HEADER_SIZE bytes. device_id is ASCII, zero padded to 8. */
void sntl_encode_header(uint8_t *out, uint16_t record_count, const char *device_id);

/* out must be SNTL_RECORD_SIZE bytes. */
void sntl_encode_record(uint8_t *out, const struct sntl_record *rec);

/* Total frame length for a given record count. */
size_t sntl_frame_size(uint16_t record_count);

/* Parse a control write. Returns 1 and sets *first_seq on a valid dump
 * command, 0 otherwise. */
int sntl_parse_cmd(const uint8_t *buf, size_t len, uint32_t *first_seq);

/*
 * How many records a dump answers with, and where it starts.
 * The selection is seq >= first_seq (inclusive - an app with nothing stored
 * sends first_seq = 0 and must still get record 0), clamped to what the ring
 * still holds. Returns the count; *start_seq gets the first seq to send.
 * Kept here rather than in the BLE code so the boundaries are host-testable.
 */
uint16_t sntl_dump_range(uint32_t first_seq, uint32_t oldest_seq, uint32_t next_seq,
			 uint32_t *start_seq);

#endif /* FRAME_H_ */
