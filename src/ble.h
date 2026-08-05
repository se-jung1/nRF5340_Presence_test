/*
 * SNTL BLE GATT service: advertises, takes dump commands on the control
 * characteristic, streams frames out on the data characteristic.
 *
 * Same service and UUIDs as the air-quality node, so one app talks to both.
 * What differs is the device id prefix (below) and the frame version byte.
 */
#ifndef BLE_H_
#define BLE_H_

#include "frame.h"

/*
 * Device id = this prefix + 6 hex of the factory id, exactly 8 bytes so it also
 * serves as the advertising name with no truncation. "PR" (presence) rather
 * than the air node's "SL" so the two are told apart in the scan list; flip it
 * back if the app or server filters on "SL".
 */
#define BLE_ID_PREFIX "PR"

/* 8 ASCII chars, NUL terminated. Same string is used as the advertising name -
 * the app matches on it to know how far it already got. */
const char *ble_device_id(void);

/* Brings up the stack. Does NOT advertise - call ble_advertise_window(). */
int ble_start(void);

/* Advertise for BLE_ADV_WINDOW_S, restarting after a disconnect until the
 * window expires. Pressing again re-arms it. Safe to call from an ISR. */
#define BLE_ADV_WINDOW_S 20
void ble_advertise_window(void);

#endif /* BLE_H_ */
