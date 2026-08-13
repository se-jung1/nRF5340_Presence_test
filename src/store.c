#include <zephyr/kernel.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/kvss/nvs.h>
#include <errno.h>
#include <string.h>

#include "store.h"

/*
 * One NVS entry per record, id = NVS_ID_REC0 + (seq % STORE_CAPACITY), so the
 * ring drops the oldest for free. Deliberately NOT batching writes: the record
 * struct is 24 B in flash plus an 8 B NVS ATE, and at 30 s a full day is
 * 2880 * 32 = ~90 KB against the 160 KB partition - a few hundred sector
 * erases a year against a 10k endurance rating. Batching would only buy wear we
 * do not need, and would cost up to a batch of records on power loss.
 */
#define NVS_ID_META   1
#define NVS_ID_FORMAT 2
#define NVS_ID_FLOOR  3
#define NVS_ID_REC0   0x100

/*
 * What shape the stored records are. nvs_read() asks for exactly
 * sizeof(struct sntl_record) and returns -ENOENT on a short entry, so records
 * written by an older firmware are unreadable after the struct grows - and
 * they still sit in flash taking up the ring, never rewritten, so NVS never
 * reclaims them either. Erase on a format change instead. seq survives it:
 * the server dedupes on (device_id, seq), so restarting the counter at 0 would
 * make every new record collide with an old one and get dropped.
 */
#define NVS_FORMAT    ((uint16_t)sizeof(struct sntl_record))

struct meta {
	uint32_t next_seq;
	uint32_t base_sec;   /* device seconds at the last append */
};

static struct nvs_fs fs;
static struct meta meta;
static uint32_t boot_offset;   /* device seconds at this boot */
static uint32_t floor_seq;     /* nothing below this is held any more */
K_MUTEX_DEFINE(lock);   /* main loop appends, BLE thread reads */

uint32_t store_now(void)
{
	return boot_offset + (uint32_t)(k_uptime_get() / 1000);
}

int store_init(void)
{
	struct flash_pages_info info;
	int err;

	fs.flash_device = PARTITION_DEVICE(storage_partition);
	if (!device_is_ready(fs.flash_device)) {
		return -ENODEV;
	}
	fs.offset = PARTITION_OFFSET(storage_partition);
	err = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	if (err) {
		return err;
	}
	fs.sector_size = info.size;
	fs.sector_count = PARTITION_SIZE(storage_partition) / info.size;

	err = nvs_mount(&fs);
	if (err) {
		return err;
	}

	if (nvs_read(&fs, NVS_ID_META, &meta, sizeof(meta)) != sizeof(meta)) {
		meta.next_seq = 0;
		meta.base_sec = 0;
	}

	uint16_t format = 0;

	nvs_read(&fs, NVS_ID_FORMAT, &format, sizeof(format));
	if (format != NVS_FORMAT) {
		printk("[STORE] record format %u -> %u: dropping stored records, "
		       "keeping seq at %u\n", format, NVS_FORMAT, meta.next_seq);
		err = nvs_clear(&fs);
		if (err) {
			return err;
		}
		err = nvs_mount(&fs);   /* clear leaves the fs unmounted */
		if (err) {
			return err;
		}
		format = NVS_FORMAT;
		/*
		 * Without this the ring would still claim every seq back to
		 * next_seq - STORE_CAPACITY, and a dump would answer with a
		 * few hundred fault records for data that was just erased.
		 */
		floor_seq = meta.next_seq;
		if (nvs_write(&fs, NVS_ID_FORMAT, &format, sizeof(format)) < 0 ||
		    nvs_write(&fs, NVS_ID_FLOOR, &floor_seq, sizeof(floor_seq)) < 0 ||
		    nvs_write(&fs, NVS_ID_META, &meta, sizeof(meta)) < 0) {
			return -EIO;
		}
	} else {
		nvs_read(&fs, NVS_ID_FLOOR, &floor_seq, sizeof(floor_seq));
	}
	/* Resume the clock where the last append left it. The powered-off gap
	 * is unknowable without an RTC - that is what RTC_UNSET is for. */
	boot_offset = meta.base_sec;
	return 0;
}

uint32_t store_next_seq(void)
{
	return meta.next_seq;
}

uint32_t store_oldest_seq(void)
{
	uint32_t ring = (meta.next_seq > STORE_CAPACITY)
			? meta.next_seq - STORE_CAPACITY : 0;

	return (floor_seq > ring) ? floor_seq : ring;
}

int store_append(struct sntl_record *rec)
{
	int err;

	k_mutex_lock(&lock, K_FOREVER);

	rec->seq = meta.next_seq;
	rec->ts  = store_now();

	err = nvs_write(&fs, NVS_ID_REC0 + (rec->seq % STORE_CAPACITY), rec, sizeof(*rec));
	if (err < 0) {
		k_mutex_unlock(&lock);
		return err;
	}

	meta.next_seq++;
	meta.base_sec = rec->ts;
	err = nvs_write(&fs, NVS_ID_META, &meta, sizeof(meta));

	k_mutex_unlock(&lock);
	return (err < 0) ? err : 0;
}

int store_get(uint32_t seq, struct sntl_record *out)
{
	ssize_t n;

	if (seq >= meta.next_seq || seq < store_oldest_seq()) {
		return -ENOENT;
	}

	k_mutex_lock(&lock, K_FOREVER);
	n = nvs_read(&fs, NVS_ID_REC0 + (seq % STORE_CAPACITY), out, sizeof(*out));
	k_mutex_unlock(&lock);

	if (n != sizeof(*out)) {
		return -ENOENT;
	}
	/* Slot may already have been recycled by a newer seq mid-read. */
	return (out->seq == seq) ? 0 : -ENOENT;
}
