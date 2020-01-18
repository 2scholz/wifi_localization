#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// some constants - mac address length, mac adress string length, max length of wireless network id with null character
enum wifi_constants {BSSID_LENGTH=6, BSSID_STRING_LENGTH=18, SSID_MAX_LENGTH_WITH_NULL=33};
// anything >=0 should mean that your are associated with the station
enum bss_status{BSS_NONE=-1, BSS_AUTHENTHICATED=0, BSS_ASSOCIATED=1, BSS_IBSS_JOINED=2};

// everything needed for sending/receiving with netlink
struct netlink_channel
{
	struct nl_sock *nl; //netlink socket
	int nl80211_id; //generic netlink nl80211 id
	uint32_t ifindex; //the wireless interface number (e.g. interface number for wlan0)
};

// a single wireless network can have multiple BSSes working as network under one SSID
struct bss_info
{
	char bssid[BSSID_STRING_LENGTH]; //this is hardware mac address of your AP
	char ssid[SSID_MAX_LENGTH_WITH_NULL]; //this is the name of your AP as you see it when connecting
	int32_t frequency;
	int32_t signal_mbm;  //signal strength in mBm, divide it by 100 to get signal in dBm
	int32_t seen_ms_ago; //when the above information was collected
};

struct bss_infos
{
	struct bss_info* data;
	int32_t sequence;
	int32_t scan_data_length;
};

/* Initializes the library
 *
 * If this functions fails the library will die with error message explaining why
 *
 * parameters:
 * interface - wireless interface, e.g. wlan0, wlan1
 *
 * returns:
 * struct wifi_scan * - pass it to all the functions in the library
 *
 */
void wifi_scan_init(const char *interface, struct netlink_channel *channel);

/* Frees the resources used by library
 *
 * parameters:
 * wifi - library data initialized with wifi_scan_init
 *
 * preconditions:
 * wifi initialized with wifi_scan_init
 */

void wifi_scan_close(struct netlink_channel *channel);

/* Make a passive scan of all networks around.
 *
 * This function triggers passive scan if necessery, waits for completion and returns the data.
 * If some other scan was triggered in the meanwhile the library will collect it's results.
 * Triggering a scan requires permissions, for testing you may use sudo.
 *
 * Scanning may take some time (it can be order of second).
 * While scanning the link may be unusable for other programs!
 *
 * parameters:
 * wifi - library data initialized with wifi_scan_init
 * bss_infos - array of bss_info of size bss_infos_length
 * bss_infos_length - the length of passed array
 *
 * returns:
 * -1 on error (errno is set) or the number of found BSSes, the number may be greater then bss_infos_length
 *
 * Some devices may fail with -1 and errno=EBUSY if triggering scan when another scan is in progress. You may wait and retry in that case
 *
 * preconditions:
 * wifi initialized with wifi_scan_init
 *
 */

int wifi_scan_channels(struct netlink_channel *channel, uint32_t *channels, struct bss_infos* scan_data);

#ifdef __cplusplus
}
#endif
