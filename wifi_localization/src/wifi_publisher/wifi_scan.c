#include <wifi_publisher/wifi_scan.h>
#include <netlink/cache.h>
#include <net/if.h>
#include <unistd.h>
#include <netlink/netlink.h>
#include <netlink/genl/genl.h>
#include <netlink/genl/family.h>
#include <netlink/genl/ctrl.h>
#include <linux/nl80211.h>
#include <stdio.h>
#include <errno.h>
#include <stdio.h>
#include <ctype.h>
#include <time.h>

struct trigger_results {
    int done;
    int aborted;
};


struct handler_args {  // For family_handler() and nl_get_multicast_id().
    const char *group;
    int id;
};


static int error_handler(struct sockaddr_nl *nla, struct nlmsgerr *err, void *arg) {
    // Callback for errors.
    // printf("error_handler() called.\n");
    int *ret = arg;
    *ret = err->error;
    return NL_STOP;
}


static int finish_handler(struct nl_msg *msg, void *arg) {
    // Callback for NL_CB_FINISH.
    int *ret = arg;
    *ret = 0;
    return NL_SKIP;
}


static int ack_handler(struct nl_msg *msg, void *arg) {
    // Callback for NL_CB_ACK.
    int *ret = arg;
    *ret = 0;
    return NL_STOP;
}


static int no_seq_check(struct nl_msg *msg, void *arg) {
    // Callback for NL_CB_SEQ_CHECK.
    return NL_OK;
}


static int family_handler(struct nl_msg *msg, void *arg) {
    // Callback for NL_CB_VALID within nl_get_multicast_id(). From http://sourcecodebrowser.com/iw/0.9.14/genl_8c.html.
    struct handler_args *grp = arg;
    struct nlattr *tb[CTRL_ATTR_MAX + 1];
    struct genlmsghdr *gnlh = nlmsg_data(nlmsg_hdr(msg));
    struct nlattr *mcgrp;
    int rem_mcgrp;

    nla_parse(tb, CTRL_ATTR_MAX, genlmsg_attrdata(gnlh, 0), genlmsg_attrlen(gnlh, 0), NULL);

    if (!tb[CTRL_ATTR_MCAST_GROUPS]) return NL_SKIP;

    nla_for_each_nested(mcgrp, tb[CTRL_ATTR_MCAST_GROUPS], rem_mcgrp) {  // This is a loop.
        struct nlattr *tb_mcgrp[CTRL_ATTR_MCAST_GRP_MAX + 1];

        nla_parse(tb_mcgrp, CTRL_ATTR_MCAST_GRP_MAX, nla_data(mcgrp), nla_len(mcgrp), NULL);

        if (!tb_mcgrp[CTRL_ATTR_MCAST_GRP_NAME] || !tb_mcgrp[CTRL_ATTR_MCAST_GRP_ID]) continue;
        if (strncmp(nla_data(tb_mcgrp[CTRL_ATTR_MCAST_GRP_NAME]), grp->group,
                nla_len(tb_mcgrp[CTRL_ATTR_MCAST_GRP_NAME]))) {
            continue;
                }

        grp->id = nla_get_u32(tb_mcgrp[CTRL_ATTR_MCAST_GRP_ID]);
        break;
    }

    return NL_SKIP;
}


int nl_get_multicast_id(struct nl_sock *sock, const char *family, const char *group) {
    // From http://sourcecodebrowser.com/iw/0.9.14/genl_8c.html.
    struct nl_msg *msg;
    struct nl_cb *cb;
    int ret, ctrlid;
    struct handler_args grp = { .group = group, .id = -ENOENT, };

    msg = nlmsg_alloc();
    if (!msg) return -ENOMEM;

    cb = nl_cb_alloc(NL_CB_DEFAULT);
    if (!cb) {
        ret = -ENOMEM;
        goto out_fail_cb;
    }
    ctrlid = genl_ctrl_resolve(sock, "nlctrl");

    genlmsg_put(msg, 0, 0, ctrlid, 0, 0, CTRL_CMD_GETFAMILY, 0);

    ret = -ENOBUFS;
    NLA_PUT_STRING(msg, CTRL_ATTR_FAMILY_NAME, family);

    ret = nl_send_auto_complete(sock, msg);
    if (ret < 0) goto out;

    ret = 1;

    nl_cb_err(cb, NL_CB_CUSTOM, error_handler, &ret);
    nl_cb_set(cb, NL_CB_ACK, NL_CB_CUSTOM, ack_handler, &ret);
    nl_cb_set(cb, NL_CB_VALID, NL_CB_CUSTOM, family_handler, &grp);
    while (ret > 0) nl_recvmsgs(sock, cb);

    if (ret == 0) ret = grp.id;

    nla_put_failure:
        out:
            nl_cb_put(cb);
        out_fail_cb:
            nlmsg_free(msg);
            return ret;
}


void mac_addr_n2a(char *mac_addr, unsigned char *arg) {
    // From http://git.kernel.org/cgit/linux/kernel/git/jberg/iw.git/tree/util.c.
    int i, l;

    l = 0;
    for (i = 0; i < 6; i++) {
        if (i == 0) {
            sprintf(mac_addr+l, "%02x", arg[i]);
            l += 2;
        } else {
            sprintf(mac_addr+l, ":%02x", arg[i]);
            l += 3;
        }
    }
}

void get_ssid(unsigned char *ie, int ielen, char* ssid) {
    uint8_t len;
    uint8_t *data;
    int i;

    while (ielen >= 2 && ielen >= ie[1]) {
        if (ie[0] == 0 && ie[1] >= 0 && ie[1] <= 32) {
            len = ie[1];
            data = ie + 2;
            for (i = 0; i < len; i++) {
                if (isprint(data[i]) && data[i] != ' ' && data[i] != '\\')
                {
                  sprintf(ssid+i, "%c", data[i]);
                }
                else if (data[i] == ' ' && (i != 0 && i != len -1))
                {
                  sprintf(ssid+i, " ");
                }
                else
                {
                  sprintf(ssid+i, "\\x%.2x", data[i]);
                }
            }
            break;
        }
        ielen -= ie[1] + 2;
        ie += ie[1] + 2;
    }
}



static int callback_trigger(struct nl_msg *msg, void *arg) {
    // Called by the kernel when the scan is done or has been aborted.
    struct genlmsghdr *gnlh = nlmsg_data(nlmsg_hdr(msg));
    struct trigger_results *results = arg;

    if (gnlh->cmd == NL80211_CMD_SCAN_ABORTED) {
        results->done = 1;
        results->aborted = 1;
    } else if (gnlh->cmd == NL80211_CMD_NEW_SCAN_RESULTS) {
        results->done = 1;
        results->aborted = 0;
    }  // else probably an uninteresting multicast message.

    return NL_SKIP;
}


static int callback_dump(struct nl_msg *msg, void *arg) {
    // Called by the kernel with a dump of the successful scan's data. Called for each SSID.
    struct bss_infos* scan_data = arg;
    struct genlmsghdr *gnlh = nlmsg_data(nlmsg_hdr(msg));
    char mac_addr[20];
    struct nlattr *tb[NL80211_ATTR_MAX + 1];
    struct nlattr *bss[NL80211_BSS_MAX + 1];
    static struct nla_policy bss_policy[NL80211_BSS_MAX + 1] = {
        [NL80211_BSS_TSF] = { .type = NLA_U64 },
        [NL80211_BSS_FREQUENCY] = { .type = NLA_U32 },
        [NL80211_BSS_BSSID] = { },
        [NL80211_BSS_BEACON_INTERVAL] = { .type = NLA_U16 },
        [NL80211_BSS_CAPABILITY] = { .type = NLA_U16 },
        [NL80211_BSS_INFORMATION_ELEMENTS] = { },
        [NL80211_BSS_SIGNAL_MBM] = { .type = NLA_U32 },
        [NL80211_BSS_SIGNAL_UNSPEC] = { .type = NLA_U8 },
        [NL80211_BSS_STATUS] = { .type = NLA_U32 },
        [NL80211_BSS_SEEN_MS_AGO] = { .type = NLA_U32 },
        [NL80211_BSS_BEACON_IES] = { },
    };

    // Parse and error check.
    nla_parse(tb, NL80211_ATTR_MAX, genlmsg_attrdata(gnlh, 0), genlmsg_attrlen(gnlh, 0), NULL);
    if (!tb[NL80211_ATTR_BSS]) {
        printf("bss info missing!\n");
        return NL_SKIP;
    }
    if (nla_parse_nested(bss, NL80211_BSS_MAX, tb[NL80211_ATTR_BSS], bss_policy)) {
        printf("failed to parse nested attributes!\n");
        return NL_SKIP;
    }
    if (!bss[NL80211_BSS_BSSID]) return NL_SKIP;
    if (!bss[NL80211_BSS_INFORMATION_ELEMENTS]) return NL_SKIP;

    // Start printing.
    mac_addr_n2a(mac_addr, nla_data(bss[NL80211_BSS_BSSID]));

    if(scan_data->scan_data_length >= scan_data->sequence)
    {
      scan_data->sequence++;
      struct bss_info* data = &scan_data->data[scan_data->sequence-1];
      strcpy(data->bssid, mac_addr);
      data->frequency = nla_get_u32(bss[NL80211_BSS_FREQUENCY]);
      data->signal_mbm = nla_get_u32(bss[NL80211_BSS_SIGNAL_MBM]);
      data->seen_ms_ago = nla_get_u32(bss[NL80211_BSS_SEEN_MS_AGO]);
      get_ssid(nla_data(bss[NL80211_BSS_INFORMATION_ELEMENTS]), nla_len(bss[NL80211_BSS_INFORMATION_ELEMENTS]), data->ssid);
    }
    else
    {
      printf("There were more APs scanned, than the bss_info array can hold.");
      return NL_STOP;
    }

    return NL_SKIP;
}

void wifi_scan_init(const char *interface, struct netlink_channel *channel)
{


	channel->ifindex = if_nametoindex(interface); // Use this wireless interface for scanning.

  // Open socket to kernel.
  channel->nl = nl_socket_alloc();  // Allocate new netlink socket in memory.
  genl_connect(channel->nl);  // Create file descriptor and bind socket.
  channel->nl80211_id = genl_ctrl_resolve(channel->nl, "nl80211");  // Find the nl80211 driver ID.
}

void wifi_scan_close(struct netlink_channel *channel)
{
  nl_socket_free(channel->nl);
}

int wifi_scan_channels(struct netlink_channel *channel, uint32_t *channels, struct bss_infos* scan_data)
{
  // Starts the scan and waits for it to finish. Does not return until the scan is done or has been aborted.
  struct trigger_results results = { .done = 0, .aborted = 0 };
  struct nl_msg *msg;
  struct nl_cb *cb;
  struct nl_msg *ssids_to_scan;
  struct nl_msg *freqs_to_scan;
  int err;
  int ret;
  int mcid = nl_get_multicast_id(channel->nl, "nl80211", "scan");
  nl_socket_add_membership(channel->nl, mcid);  // Without this, callback_trigger() won't be called.

  // Allocate the messages and callback handler.
  msg = nlmsg_alloc();
  if (!msg) {
      printf("ERROR: Failed to allocate netlink message for msg.\n");
      return -1;
  }
  ssids_to_scan = nlmsg_alloc();
  if (!ssids_to_scan) {
      printf("ERROR: Failed to allocate netlink message for ssids_to_scan.\n");
      nlmsg_free(msg);
      return -1;
  }
  freqs_to_scan = nlmsg_alloc();
  if (!freqs_to_scan) {
      printf("ERROR: Failed to allocate netlink message for ssids_to_scan.\n");
      nlmsg_free(ssids_to_scan);
      nlmsg_free(msg);
      return -1;
  }
  cb = nl_cb_alloc(NL_CB_DEFAULT);
  if (!cb) {
      printf("ERROR: Failed to allocate netlink callbacks.\n");
      nlmsg_free(msg);
      nlmsg_free(ssids_to_scan);
      nlmsg_free(freqs_to_scan);
      return -1;
  }

  // Setup the messages and callback handler.
  genlmsg_put(msg, 0, 0, channel->nl80211_id, 0, 0, NL80211_CMD_TRIGGER_SCAN, 0);  // Setup which command to run.
  nla_put_u32(msg, NL80211_ATTR_IFINDEX, channel->ifindex);  // Add message attribute, which interface to use.
  nla_put(ssids_to_scan, 1, 0, "");  // Scan all SSIDs.
  nla_put_nested(msg, NL80211_ATTR_SCAN_SSIDS, ssids_to_scan);  // Add message attribute, which SSIDs to scan for.
  nlmsg_free(ssids_to_scan);  // Copied to `msg` above, no longer need this.

  int i = 0;
  while(channels[i]!=0)
  {
    nla_put_u32(freqs_to_scan, NL80211_FREQUENCY_ATTR_FREQ, channels[i]);
    i++;
  }
  nla_put_nested(msg, NL80211_ATTR_SCAN_FREQUENCIES, freqs_to_scan);
  nl_cb_set(cb, NL_CB_VALID, NL_CB_CUSTOM, callback_trigger, &results);  // Add the callback.
  nl_cb_err(cb, NL_CB_CUSTOM, error_handler, &err);
  nl_cb_set(cb, NL_CB_FINISH, NL_CB_CUSTOM, finish_handler, &err);
  nl_cb_set(cb, NL_CB_ACK, NL_CB_CUSTOM, ack_handler, &err);
  nl_cb_set(cb, NL_CB_SEQ_CHECK, NL_CB_CUSTOM, no_seq_check, NULL);  // No sequence checking for multicast messages.

  // Send NL80211_CMD_TRIGGER_SCAN to start the scan. The kernel may reply with NL80211_CMD_NEW_SCAN_RESULTS on
  // success or NL80211_CMD_SCAN_ABORTED if another scan was started by another process.
  err = 1;
  ret = nl_send_auto(channel->nl, msg);  // Send the message.
  clock_t start = clock();

  clock_t end = clock();
  float seconds = 0;
  while (err > 0 || seconds > 10)  // First wait for ack_handler(). This helps with basic errors.
  {
    ret = nl_recvmsgs(channel->nl, cb);
    end = clock();
    seconds = (float)(end - start) / CLOCKS_PER_SEC; // Making sure the loop does not go on for ever
  }
  if(err > 0)
  {
    return -1;
  }
  if (err < 0) {
      // printf("WARNING: err has a value of %d.\n", err);
  }
  if (ret < 0) {
      // printf("ERROR: nl_recvmsgs() returned %d (%s).\n", ret, nl_geterror(-ret));
      return -1;
  }
  while (!results.done) nl_recvmsgs(channel->nl, cb);  // Now wait until the scan is done or aborted.
  if (results.aborted) {
      // printf("ERROR: Kernel aborted scan.\n");
      return -1;
  }

  // Cleanup.
  nlmsg_free(msg);
  nl_cb_put(cb);
  nl_socket_drop_membership(channel->nl, mcid);  // No longer need this.

  // Now get info for all SSIDs detected.
  msg = nlmsg_alloc();  // Allocate a message.
  genlmsg_put(msg, 0, 0, channel->nl80211_id, 0, NLM_F_DUMP, NL80211_CMD_GET_SCAN, 0);  // Setup which command to run.
  nla_put_u32(msg, NL80211_ATTR_IFINDEX, channel->ifindex);  // Add message attribute, which interface to use.
  nl_socket_modify_cb(channel->nl, NL_CB_VALID, NL_CB_CUSTOM, callback_dump, scan_data);  // Add the callback.
  ret = nl_send_auto(channel->nl, msg);  // Send the message.

  ret = nl_recvmsgs_default(channel->nl);  // Retrieve the kernel's answer. callback_dump() prints SSIDs to stdout.
  nlmsg_free(msg);
  if (ret < 0) {
    // printf("ERROR: nl_recvmsgs_default() returned %d (%s).\n", ret, nl_geterror(-ret));
    return -1;
  }

    return 0;
}
