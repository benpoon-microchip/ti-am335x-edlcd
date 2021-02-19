#include <sys/slogcodes.h>
#include "wilc_main.h"
#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <pthread.h>
#include <net/route.h>

#include "wilc_wlan.h"
#include "wilc_wfi_netdevice.h"
#include "wilc_wifi_cfgoperations.h"
#include "etherdevice.h"
#include "wilc_netdev.h"
#include "workqueue.h"
#include "type_defs.h"
#include "cfg80211.h"

#define ACTION_CAT_ID			24
#define ACTION_SUBTYPE_ID		25
#define P2P_PUB_ACTION_SUBTYPE		30

#define ACTION_FRAME			0xd0
#define GO_INTENT_ATTR_ID		0x04
#define CHANLIST_ATTR_ID		0x0b
#define OPERCHAN_ATTR_ID		0x11
#define PUB_ACTION_ATTR_ID		0x04
#define P2PELEM_ATTR_ID			0xdd

#define GO_NEG_REQ			0x00
#define GO_NEG_RSP			0x01
#define GO_NEG_CONF			0x02
#define P2P_INV_REQ			0x03
#define P2P_INV_RSP			0x04
#define PUBLIC_ACT_VENDORSPEC		0x09
#define GAS_INITIAL_REQ			0x0a
#define GAS_INITIAL_RSP			0x0b

#define WILC_INVALID_CHANNEL		0

struct wilc_p2p_mgmt_data {
	int size;
	u8 *buff;
};

struct scan_results scan_result[MAX_SCAN_AP];
int scan_idx = 0;
int scan_num = 0;
int scan_finish = 0;

static const u8 p2p_oui[] = {0x50, 0x6f, 0x9A, 0x09};
static const u8 p2p_vendor_spec[] = {0xdd, 0x05, 0x00, 0x08, 0x40, 0x03};

//FIXME
#undef malloc
#undef free
struct callback_cb {
    struct stk_callback scb;
    struct ifnet *ifp;
    uint8_t bssid[IEEE80211_ADDR_LEN];
};

static void ieee80211_scan_fn(void *arg)
{
    struct callback_cb *cb = (struct callback_cb *)arg;
    fprintf(stderr, "ieee80211_scan_fn scan finish\n");
    slogf(_SLOGC_NETWORK, _SLOG_ERROR, "%s: stk scan cb %p", __FUNCTION__, cb);
    rt_ieee80211msg(cb->ifp, RTM_IEEE80211_SCAN, NULL, 0);
    free(cb);
}

static void ieee80211_associate_fn(void *arg)
{
    struct callback_cb *cb = (struct callback_cb *)arg;
    struct ieee80211_join_event iev;

    memset(&iev, 0 ,sizeof(iev));
    IEEE80211_ADDR_COPY(iev.iev_addr,cb->bssid);

    slogf(_SLOGC_NETWORK, _SLOG_ERROR, "%s: bssid 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", __FUNCTION__, iev.iev_addr[0], iev.iev_addr[1], iev.iev_addr[2], iev.iev_addr[3], iev.iev_addr[4], iev.iev_addr[5]);
    slogf(_SLOGC_NETWORK, _SLOG_ERROR, "%s: stk cb %p", __FUNCTION__, cb);
    fprintf(stderr, "ieee80211_associate_fn connect finish\n");
    rt_ieee80211msg(cb->ifp, RTM_IEEE80211_ASSOC, &iev, sizeof(iev));
    if_link_state_change(cb->ifp, LINK_STATE_UP);
    //if_up(cb->ifp);
    free(cb);
}

static void ieee80211_disassociate_fn(void *arg)
{
    struct callback_cb *cb = (struct callback_cb *)arg;

    slogf(_SLOGC_NETWORK, _SLOG_ERROR, "%s: stk cb %p", __FUNCTION__, cb);
    rt_ieee80211msg(cb->ifp, RTM_IEEE80211_DISASSOC, NULL, 0);
    if_link_state_change(cb->ifp, LINK_STATE_DOWN);
    free(cb);
}

static void ieee80211_scan_msg(struct ifnet *ifp)
{
    struct callback_cb *cb = (struct callback_cb *)malloc(sizeof(struct callback_cb));

    if (!cb) {
        slogf(_SLOGC_NETWORK, _SLOG_ERROR, "%s: oom", __FUNCTION__);
        return;
    }

    cb->ifp = ifp;
    cb->scb.func = ieee80211_scan_fn;
    cb->scb.arg = cb;
    slogf(_SLOGC_NETWORK, _SLOG_ERROR, "%s: stk cb %p", __FUNCTION__, cb);
    if (stk_context_callback(&cb->scb) == -1) {
        slogf(_SLOGC_NETWORK, _SLOG_ERROR, "%s: err: %s", __FUNCTION__, strerror(errno));
        free(cb);
    }
}

static void ieee80211_associate_msg(struct ifnet *ifp, uint8_t bssid[])
{
    struct callback_cb *cb = (struct callback_cb *)malloc(sizeof(struct callback_cb));

    if (!cb) {
        slogf(_SLOGC_NETWORK, _SLOG_ERROR, "%s: oom", __FUNCTION__);
        return;
    }

    cb->ifp = ifp;
    cb->scb.func = ieee80211_associate_fn;
    cb->scb.arg = cb;
    memcpy(cb->bssid, bssid, ETH_ALEN);
    slogf(_SLOGC_NETWORK, _SLOG_ERROR, "%s: bssod 0x%x 0x%x", __FUNCTION__, cb->bssid[0], cb->bssid[1]);
    slogf(_SLOGC_NETWORK, _SLOG_ERROR, "%s: stk cb %p", __FUNCTION__, cb);
    if (stk_context_callback(&cb->scb) == -1) {
        slogf(_SLOGC_NETWORK, _SLOG_ERROR, "%s: err: %s", __FUNCTION__, strerror(errno));
        free(cb);
    }
}

static void ieee80211_disassociate_msg(struct ifnet *ifp, uint8_t bssid[])
{
    struct callback_cb *cb = (struct callback_cb *)malloc(sizeof(struct callback_cb));

    if (!cb) {
        slogf(_SLOGC_NETWORK, _SLOG_ERROR, "%s: oom", __FUNCTION__);
        return;
    }

    cb->ifp = ifp;
    cb->scb.func = ieee80211_disassociate_fn;
    cb->scb.arg = cb;
    slogf(_SLOGC_NETWORK, _SLOG_ERROR, "%s: stk cb %p", __FUNCTION__, cb);
    if (stk_context_callback(&cb->scb) == -1) {
        slogf(_SLOGC_NETWORK, _SLOG_ERROR, "%s: err: %s", __FUNCTION__, strerror(errno));
        free(cb);
    }
}

static void cfg_scan_result(enum scan_event scan_event,
			    struct wilc_rcvd_net_info *info, void *user_void, struct ifnet *ifp)
{
	struct wilc_priv *priv = user_void;

	if (!priv || !priv->cfg_scanning) {
		//pr_err("%s is NULL\n", __func__);
		return;
	}

	if (scan_event == SCAN_EVENT_NETWORK_FOUND) {
		s32 freq = 0; //TODO
		//struct ieee80211_channel *channel;
		//struct cfg80211_bss *bss;
		//struct wiphy *wiphy = priv->dev->ieee80211_ptr->wiphy;

		//if (!wiphy || !info)
		//	return;

		//freq = ieee80211_channel_to_frequency((s32)info->ch,
		//				      NL80211_BAND_2GHZ);
		//channel = ieee80211_get_channel(wiphy, freq);
		//if (!channel)
		//	return;

		const u8 *ssid_elm;
		u8 *ies;
		int ies_len;
		size_t offset;

		if (ieee80211_is_probe_resp(info->mgmt->frame_control))
			offset = offsetof(struct ieee80211_mgmt, u.probe_resp.variable);
		else if (ieee80211_is_beacon(info->mgmt->frame_control))
			offset = offsetof(struct ieee80211_mgmt, u.beacon.variable);

		ies = info->mgmt->u.beacon.variable;
		ies_len = info->frame_len - offset;

		ssid_elm = cfg80211_find_ie(WLAN_EID_SSID, ies, ies_len);
		PRINT_INFO(HOSTINF_DBG, "ssid = 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n", ssid_elm[0], ssid_elm[1], ssid_elm[2], ssid_elm[3], ssid_elm[4], ssid_elm[5], ssid_elm[6], ssid_elm[7]);
		PRINT_INFO(HOSTINF_DBG, "scan_idx = %d\n", scan_idx);

		if (scan_idx < MAX_SCAN_AP)
		{
			memcpy(scan_result[scan_idx]. ssid, ssid_elm + 2, ssid_elm[1]);
			scan_result[scan_idx].ssid_len = ssid_elm[1];
			memcpy(scan_result[scan_idx].bssid, info->mgmt->bssid, 6);
			scan_result[scan_idx].channel = info->ch;
			scan_result[scan_idx].rssi = info->rssi;
			scan_result[scan_idx].mgmt = (struct ieee80211_mgmt *) create_ptr(info->frame_len);
			memcpy(scan_result[scan_idx].mgmt, info->mgmt, info->frame_len);
			scan_result[scan_idx].frame_len = info->frame_len;

			scan_idx++;
			scan_num++;
		}
		else
		{
			PRINT_INFO(HOSTINF_DBG, "AP scanned reach maximum\n");
		}

		PRINT_INFO(CFG80211_DBG, "ifp= %p\n", ifp);
		PRINT_D(CFG80211_DBG,
			"Network Info:: CHANNEL: %d, RSSI: %d,\n",
			info->ch, ((s32)info->rssi * 100));

		//bss = cfg80211_inform_bss_frame(wiphy, channel, info->mgmt,
		//				info->frame_len,
		//				(s32)info->rssi * 100,
		//				GFP_KERNEL);
		//if (!bss)
			//cfg80211_put_bss(wiphy, bss);
	} else if (scan_event == SCAN_EVENT_DONE) {
		//PRINT_INFO(priv->dev, CFG80211_DBG, "Scan Done[%p]\n",
		//	   priv->dev);
		pthread_mutex_lock(&priv->scan_req_lock);

		if (priv->scan_req) {

		//	cfg80211_scan_done(priv->scan_req, false);

			priv->cfg_scanning = false;
			priv->scan_req = NULL;
		}
		//scan_idx = 0;
		scan_finish = 1;
		PRINT_INFO(CFG80211_DBG, "ifp= %p\n", ifp);
		PRINT_INFO(CFG80211_DBG, "Scan Done 2\n")
		ieee80211_scan_msg(ifp);

		pthread_mutex_unlock(&priv->scan_req_lock);
	} else if (scan_event == SCAN_EVENT_ABORTED) {
		pthread_mutex_lock(&priv->scan_req_lock);

		PRINT_INFO(CFG80211_DBG, "Scan Aborted\n");
		if (priv->scan_req) {

			//cfg80211_scan_done(priv->scan_req, false);

			priv->cfg_scanning = false;
			priv->scan_req = NULL;
		}
		pthread_mutex_unlock(&priv->scan_req_lock);
	}

	PRINT_INFO(CFG80211_DBG, "Scan Done 3\n")
}

void cfg_connect_result(enum conn_event conn_disconn_evt,
			       u8 mac_status, void *priv_data)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR, "%s: In", __FUNCTION__);
	//struct wilc_priv *priv = priv_data;
	//struct net_device *dev = priv->dev;
	//struct wilc_vif *vif = netdev_priv(dev);
	struct wilc_vif *vif = priv_data;
	//struct host_if_drv *wfi_drv = priv->hif_drv;
	struct host_if_drv *wfi_drv = vif->hif_drv;
	struct wilc_conn_info *conn_info = &wfi_drv->conn_info;
	struct wilc_priv *priv = &vif->priv;
	PRINT_INFO(CFG80211_DBG,
				   "cfg_connect_result\n");

	vif->connecting = false;

	if (conn_disconn_evt == EVENT_CONN_RESP) {
		u16 connect_status = conn_info->status;

		PRINT_INFO(CFG80211_DBG,
			   "Connection response received=%d connect_stat[%d]\n",
			   mac_status, connect_status);
		if (mac_status == WILC_MAC_STATUS_DISCONNECTED &&
		    connect_status == WLAN_STATUS_SUCCESS) {
			connect_status = WLAN_STATUS_UNSPECIFIED_FAILURE;

			wilc_wlan_set_bssid(vif, NULL, WILC_STATION_MODE);

			if (vif->iftype != WILC_CLIENT_MODE)
				vif->wilc->sta_ch = WILC_INVALID_CHANNEL;

			PRINT_ER(dev, "Unspecified failure\n");
			ieee80211_disassociate_msg(vif->wilc->sc_ic.ic_ifp, conn_info->bssid);
		}

		if (connect_status == WLAN_STATUS_SUCCESS) {
			PRINT_INFO(CFG80211_DBG,
				"Connection Successful: BSSID: %x%x%x%x%x%x\n",
				conn_info->bssid[0], conn_info->bssid[1],
				conn_info->bssid[2], conn_info->bssid[3],
				conn_info->bssid[4], conn_info->bssid[5]);
			memcpy(priv->associated_bss, conn_info->bssid,
			       ETH_ALEN);

			ieee80211_associate_msg(vif->wilc->sc_ic.ic_ifp, conn_info->bssid);
		}

		PRINT_INFO(CFG80211_DBG,
			   "Association request info elements length = %d\n",
			   conn_info->req_ies_len);
		PRINT_INFO(CFG80211_DBG,
			   "Association response info elements length = %d\n",
			   conn_info->resp_ies_len);


		//cfg80211_connect_result(dev, conn_info->bssid,
		//			conn_info->req_ies,
		//			conn_info->req_ies_len,
		//			conn_info->resp_ies,
		//			conn_info->resp_ies_len, connect_status,
		//			GFP_KERNEL);

		//vif->bss = NULL;
	} else if (conn_disconn_evt == EVENT_DISCONN_NOTIF) {
		u16 reason = 0;
		(void)reason; //TODO: reason

		PRINT_INFO(CFG80211_DBG,
			 "Received WILC_MAC_STATUS_DISCONNECTED dev [%p]\n",
			 priv->dev);
		priv->p2p.local_random = 0x01;
		priv->p2p.recv_random = 0x00;
		priv->p2p.is_wilc_ie = false;
		eth_zero_addr(priv->associated_bss);
		wilc_wlan_set_bssid(vif, NULL, WILC_STATION_MODE);

		if (vif->iftype != WILC_CLIENT_MODE) {
			vif->wilc->sta_ch = WILC_INVALID_CHANNEL;
		} else {
			if (wfi_drv->ifc_up)
				reason = 3;
			else
				reason = 1;
		}

		ieee80211_disassociate_msg(vif->wilc->sc_ic.ic_ifp, conn_info->bssid);
		//cfg80211_disconnected(dev, reason, NULL, 0, false, GFP_KERNEL);

	}
}

struct wilc_vif *wilc_get_wl_to_vif(struct wilc_dev *wl)
{
	struct wilc_vif *vif;

	//vif = list_first_or_null_rcu(&wl->vif_list, typeof(*vif), list);
	vif = list_first_entry_or_null(&wl->vif_list, typeof(*vif), list);

	return vif;
}

static inline void wilc_wfi_cfg_parse_ch_attr(struct wilc_vif *vif, u8 *buf,
					      u8 ch_list_attr_idx,
					      u8 op_ch_attr_idx, u8 sta_ch)
{
	int i = 0;
	int j = 0;

	if (ch_list_attr_idx) {
		u8 limit = ch_list_attr_idx + 3 + buf[ch_list_attr_idx + 1];

		for (i = ch_list_attr_idx + 3; i < limit; i++) {
			if (buf[i] == 0x51) {
				for (j = i + 2; j < ((i + 2) + buf[i + 1]); j++)
					buf[j] = sta_ch;
				break;
			}
		}
	}

	if (op_ch_attr_idx) {
		buf[op_ch_attr_idx + 6] = 0x51;
		buf[op_ch_attr_idx + 7] = sta_ch;
	}
}

static void wilc_wfi_cfg_parse_rx_action(struct wilc_vif *vif, u8 *buf,
					 u32 len, u8 sta_ch, bool p2p_mode)
{
	u32 index = 0;
	u8 op_channel_attr_index = 0;
	u8 channel_list_attr_index = 0;

	while (index < len) {
		if (buf[index] == GO_INTENT_ATTR_ID) {
			if (!p2p_mode)
				buf[index + 3] = (buf[index + 3]  & 0x01) |
						 (0x0f << 1);
			else
				buf[index + 3] = (buf[index + 3]  & 0x01) |
						 (0x00 << 1);
		}
		if (buf[index] ==  CHANLIST_ATTR_ID)
			channel_list_attr_index = index;
		else if (buf[index] ==  OPERCHAN_ATTR_ID)
			op_channel_attr_index = index;
		index += buf[index + 1] + 3;
	}
	if (sta_ch != WILC_INVALID_CHANNEL)
		wilc_wfi_cfg_parse_ch_attr(vif, buf, channel_list_attr_index,
					   op_channel_attr_index, sta_ch);
}

static void wilc_wfi_cfg_parse_rx_vendor_spec(struct wilc_vif *vif, u8 *buff,
					      u32 size)
{
	int i;
	u8 subtype;
	//struct wilc_vif *vif = netdev_priv(priv->dev);
	struct wilc_priv *priv = &vif->priv;

	subtype = buff[P2P_PUB_ACTION_SUBTYPE];
	if ((subtype == GO_NEG_REQ || subtype == GO_NEG_RSP) &&
	    !priv->p2p.is_wilc_ie) {
		for (i = P2P_PUB_ACTION_SUBTYPE; i < size; i++) {
			if (!memcmp(p2p_vendor_spec, &buff[i], 6)) {
				priv->p2p.recv_random = buff[i + 6];
				priv->p2p.is_wilc_ie = true;
				PRINT_INFO(GENERIC_DBG,
					   "WILC Vendor specific IE:%02x\n",
					   priv->p2p.recv_random);
				break;
			}
		}
	}

	if (priv->p2p.local_random <= priv->p2p.recv_random) {
		PRINT_INFO(GENERIC_DBG,
			   "PEER WILL BE GO LocaRand=%02x RecvRand %02x\n",
			   priv->p2p.local_random, priv->p2p.recv_random);
		return;
	}

	if (subtype == GO_NEG_REQ || subtype == GO_NEG_RSP ||
	    subtype == P2P_INV_REQ || subtype == P2P_INV_RSP) {
		for (i = P2P_PUB_ACTION_SUBTYPE + 2; i < size; i++) {
			if (buff[i] == P2PELEM_ATTR_ID &&
			    !(memcmp(p2p_oui, &buff[i + 2], 4))) {
				bool p2p_mode = vif->wilc->attr_sysfs.p2p_mode;

				wilc_wfi_cfg_parse_rx_action(vif, &buff[i + 6],
							     size - (i + 6),
							     vif->wilc->sta_ch,
							     p2p_mode);
				break;
			}
		}
	}
}


int wilc_wfi_p2p_rx(struct wilc_vif *vif, u8 *buff, u32 size)
{
	//struct wilc_dev *wilc = vif->wilc;
	struct wilc_priv *priv = &vif->priv;
	//struct host_if_drv *wfi_drv = priv->hif_drv;
	u32 header, pkt_offset;
	//s32 freq;
	__le16 fc;
	int ret = 0;

	//header = get_unaligned_le32(buff - HOST_HDR_OFFSET);
	memcpy(&header, buff - HOST_HDR_OFFSET, 4);

	pkt_offset = GET_PKT_OFFSET(header);

	fc = ((struct ieee80211_hdr *)buff)->frame_control;
	if (pkt_offset & IS_MANAGMEMENT_CALLBACK) {
		bool ack = false;
		(void)ack; //TODO: ack

		if (ieee80211_is_probe_resp(fc) ||	(pkt_offset & IS_MGMT_STATUS_SUCCES))
			ack = true;

		//cfg80211_mgmt_tx_status(&vif->priv.wdev, priv->tx_cookie, buff,
			//		size, ack, GFP_KERNEL);
		return true;
	}

	PRINT_D(GENERIC_DBG, "Rx Frame Type:%x\n", fc);


	//freq = ieee80211_channel_to_frequency(wl->op_ch, IEEE80211_BAND_2GHZ);

	//if (!ieee80211_is_action(fc)) {
	//	ret = cfg80211_rx_mgmt(&vif->priv.wdev, freq, 0, buff, size, 0);
	//	return ret;
	//}

	PRINT_D(GENERIC_DBG,
		   "Rx Action Frame Type: %x %x\n",
		   buff[ACTION_SUBTYPE_ID],
		   buff[P2P_PUB_ACTION_SUBTYPE]);
	//if (priv->cfg_scanning &&
	 //   time_after_eq(jiffies, (unsigned long)wfi_drv->p2p_timeout)) {
	//	PRINT_WRN(vif->ndev, GENERIC_DBG,
	//		  "Receiving action wrong ch\n");
	//	return false;
	//}
	if (buff[ACTION_CAT_ID] == PUB_ACTION_ATTR_ID) {
		u8 subtype = buff[P2P_PUB_ACTION_SUBTYPE];

		switch (buff[ACTION_SUBTYPE_ID]) {
		case GAS_INITIAL_REQ:
			PRINT_D(GENERIC_DBG,
				   "GAS INITIAL REQ %x\n",
				   buff[ACTION_SUBTYPE_ID]);
			break;

		case GAS_INITIAL_RSP:
			PRINT_D(GENERIC_DBG,
				   "GAS INITIAL RSP %x\n",
				   buff[ACTION_SUBTYPE_ID]);
			break;

		case PUBLIC_ACT_VENDORSPEC:
			if (!memcmp(p2p_oui, &buff[ACTION_SUBTYPE_ID + 1], 4))
				wilc_wfi_cfg_parse_rx_vendor_spec(vif, buff,
								  size);

			if ((subtype == GO_NEG_REQ || subtype == GO_NEG_RSP) &&
			    priv->p2p.is_wilc_ie)
				size -= 7;

			break;

		default:
			slogf(_SLOGC_NETWORK, _SLOG_ERROR,"Not handled action frame type:%x\n", buff[ACTION_SUBTYPE_ID]);

			break;
		}
	}
	//ret = cfg80211_rx_mgmt(&vif->priv.wdev, freq, 0, buff, size, 0);
	return ret;
}


int scan(struct wilc_vif *vif, struct cfg80211_scan_request *request)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] In\n", __func__);
			PRINT_D(CFG80211_DBG,
				"[%s] In",
				__func__);

	//struct wilc_vif *vif = netdev_priv(request->wdev->netdev);
	struct wilc_priv *priv = &vif->priv;
	u32 i;
	int ret = 0;
	u8 scan_ch_list[WILC_MAX_NUM_SCANNED_CH];
	u8 scan_type;

	if (request->n_channels > WILC_MAX_NUM_SCANNED_CH) {
		PRINT_ER(priv->dev, "Requested scanned channels over\n");

		return -EINVAL;
	}

	priv->scan_req = request;
	priv->cfg_scanning = true;
	for (i = 0; i < request->n_channels; i++) {
		u16 freq = request->channels[i].ic_freq;

		scan_ch_list[i] = (u8)ieee80211_mhz2ieee(freq, IEEE80211_CHAN_2GHZ);

		PRINT_D(CFG80211_DBG,
			"ScanChannel List[%d] = %d",
			i, scan_ch_list[i]);
	}

	PRINT_INFO(CFG80211_DBG, "Requested num of channel %d\n",
		   request->n_channels);
	PRINT_INFO(CFG80211_DBG, "Scan Request IE len =  %d\n",
		   request->ie_len);
	PRINT_INFO(CFG80211_DBG, "Number of SSIDs %d\n",
		   request->n_ssids);

	PRINT_INFO(CFG80211_DBG,
		   "Trigger Scan Request\n");

	if (request->n_ssids)
		scan_type = WILC_FW_ACTIVE_SCAN;
	else
		scan_type = WILC_FW_PASSIVE_SCAN;

	ret = wilc_scan(vif, WILC_FW_USER_SCAN, scan_type, scan_ch_list,
			request->n_channels, cfg_scan_result, (void *)priv,
			request);

	if (ret) {
		priv->scan_req = NULL;
		priv->cfg_scanning = false;
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"Device is busy: Error(%d)\n", ret);

	}

	return ret;
}

#if 0
int connect(struct wiphy *wiphy, struct wilc_vif *vif,
		   struct cfg80211_connect_params *sme)
{
	//struct wilc_vif *vif = netdev_priv(dev);
	struct wilc_priv *priv = &vif->priv;
	struct host_if_drv *wfi_drv = priv->hif_drv;
	int ret;
	u32 i;
	u8 security = WILC_FW_SEC_NO;
	enum authtype auth_type = WILC_FW_AUTH_ANY;
	u32 cipher_group;
	struct cfg80211_bss *bss;
	void *join_params;
	u8 ch;

	vif->connecting = true;

	PRINT_INFO(vif->ndev, CFG80211_DBG,
		   "Connecting to SSID [%s] on netdev [%p] host if [%x]\n",
		   sme->ssid, dev, (u32)priv->hif_drv);

	if (vif->iftype == WILC_CLIENT_MODE)
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Connected to Direct network,OBSS disabled\n");

	PRINT_D(vif->ndev, CFG80211_DBG, "Required SSID= %s\n, AuthType= %d\n",
		sme->ssid, sme->auth_type);

	memset(priv->wep_key, 0, sizeof(priv->wep_key));
	memset(priv->wep_key_len, 0, sizeof(priv->wep_key_len));

	PRINT_D(vif->ndev, CFG80211_DBG, "sme->crypto.wpa_versions=%x\n",
		sme->crypto.wpa_versions);
	PRINT_D(vif->ndev, CFG80211_DBG, "sme->crypto.cipher_group=%x\n",
		sme->crypto.cipher_group);
	PRINT_D(vif->ndev, CFG80211_DBG, "sme->crypto.n_ciphers_pairwise=%d\n",
		sme->crypto.n_ciphers_pairwise);
	for (i = 0; i < sme->crypto.n_ciphers_pairwise; i++)
		PRINT_D(vif->ndev, CORECONFIG_DBG,
			"sme->crypto.ciphers_pairwise[%d]=%x\n", i,
			sme->crypto.ciphers_pairwise[i]);

	cipher_group = sme->crypto.cipher_group;
	if (cipher_group != 0) {
		PRINT_INFO(vif->ndev, CORECONFIG_DBG,
			   ">> sme->crypto.wpa_versions: %x\n",
			   sme->crypto.wpa_versions);
		if (cipher_group == WLAN_CIPHER_SUITE_WEP40) {
			security = WILC_FW_SEC_WEP;
			PRINT_D(vif->ndev, CFG80211_DBG,
				"WEP Default Key Idx = %d\n", sme->key_idx);

			for (i = 0; i < sme->key_len; i++)
				PRINT_D(vif->ndev, CORECONFIG_DBG,
				"WEP Key Value[%d] = %d\n", i, sme->key[i]);

			priv->wep_key_len[sme->key_idx] = sme->key_len;
			memcpy(priv->wep_key[sme->key_idx], sme->key,
			       sme->key_len);

			wilc_set_wep_default_keyid(vif, sme->key_idx);
			wilc_add_wep_key_bss_sta(vif, sme->key, sme->key_len,
						 sme->key_idx);
		} else if (cipher_group == WLAN_CIPHER_SUITE_WEP104) {
			security = WILC_FW_SEC_WEP_EXTENDED;

			priv->wep_key_len[sme->key_idx] = sme->key_len;
			memcpy(priv->wep_key[sme->key_idx], sme->key,
			       sme->key_len);

			wilc_set_wep_default_keyid(vif, sme->key_idx);
			wilc_add_wep_key_bss_sta(vif, sme->key, sme->key_len,
						 sme->key_idx);
		} else if (sme->crypto.wpa_versions & NL80211_WPA_VERSION_2) {
			if (cipher_group == WLAN_CIPHER_SUITE_TKIP)
				security = WILC_FW_SEC_WPA2_TKIP;
			else
				security = WILC_FW_SEC_WPA2_AES;
		} else if (sme->crypto.wpa_versions & NL80211_WPA_VERSION_1) {
			if (cipher_group == WLAN_CIPHER_SUITE_TKIP)
				security = WILC_FW_SEC_WPA_TKIP;
			else
				security = WILC_FW_SEC_WPA_AES;
		} else {
			ret = -ENOTSUPP;
			PRINT_ER(dev, "Unsupported cipher\n");
			goto out_error;
		}
	}

	if ((sme->crypto.wpa_versions & NL80211_WPA_VERSION_1) ||
	    (sme->crypto.wpa_versions & NL80211_WPA_VERSION_2)) {
		for (i = 0; i < sme->crypto.n_ciphers_pairwise; i++) {
			u32 ciphers_pairwise = sme->crypto.ciphers_pairwise[i];

			if (ciphers_pairwise == WLAN_CIPHER_SUITE_TKIP)
				security |= WILC_FW_TKIP;
			else
				security |= WILC_FW_AES;
		}
	}

	PRINT_INFO(vif->ndev, CFG80211_DBG, "Adding key with cipher group %x\n",
		   cipher_group);

	PRINT_INFO(vif->ndev, CFG80211_DBG, "Authentication Type = %d\n",
		   sme->auth_type);
	switch (sme->auth_type) {
	case NL80211_AUTHTYPE_OPEN_SYSTEM:
		PRINT_INFO(vif->ndev, CFG80211_DBG, "In OPEN SYSTEM\n");
		auth_type = WILC_FW_AUTH_OPEN_SYSTEM;
		break;

	case NL80211_AUTHTYPE_SHARED_KEY:
		auth_type = WILC_FW_AUTH_SHARED_KEY;
		PRINT_INFO(vif->ndev, CFG80211_DBG, "In SHARED KEY\n");
		break;

	default:
		PRINT_INFO(vif->ndev, CFG80211_DBG,
			   "Automatic Authentication type= %d\n",
			   sme->auth_type);
		break;
	}

	if (sme->crypto.n_akm_suites) {
		if (sme->crypto.akm_suites[0] == WLAN_AKM_SUITE_8021X)
			auth_type = WILC_FW_AUTH_IEEE8021;
	}

	if (wfi_drv->usr_scan_req.scan_result) {
		netdev_err(vif->ndev, "%s: Scan in progress\n", __func__);
		ret = -EBUSY;
		goto out_error;
	}

	bss = cfg80211_get_bss(wiphy, sme->channel, sme->bssid, sme->ssid,
			       sme->ssid_len, IEEE80211_BSS_TYPE_ANY,
			       IEEE80211_PRIVACY(sme->privacy));

	if (!bss) {
		ret = -EINVAL;
		goto out_error;
	}
	if (ether_addr_equal_unaligned(vif->bssid, bss->bssid)) {
		ret = -EALREADY;
		goto out_put_bss;
	}

	join_params = wilc_parse_join_bss_param(bss, &sme->crypto);
	if (!join_params) {
		PRINT_ER(vif->ndev, "%s: failed to construct join param\n",
			 __func__);
		ret = -EINVAL;
		goto out_put_bss;
	}
	ch = ieee80211_frequency_to_channel(bss->channel->center_freq);
	PRINT_D(vif->ndev, CFG80211_DBG, "Required Channel = %d\n", ch);
	vif->wilc->op_ch = ch;
	if (vif->iftype != WILC_CLIENT_MODE)
		vif->wilc->sta_ch = ch;

	wilc_wlan_set_bssid(dev, bss->bssid, WILC_STATION_MODE);

	wfi_drv->conn_info.security = security;
	wfi_drv->conn_info.auth_type = auth_type;
	wfi_drv->conn_info.ch = ch;
	wfi_drv->conn_info.conn_result = cfg_connect_result;
	wfi_drv->conn_info.arg = priv;
	wfi_drv->conn_info.param = join_params;

	ret = wilc_set_join_req(vif, bss->bssid, sme->ie, sme->ie_len);
	if (ret) {
		PRINT_ER(dev, "wilc_set_join_req(): Error(%d)\n", ret);
		ret = -ENOENT;
		if (vif->iftype != WILC_CLIENT_MODE)
			vif->wilc->sta_ch = WILC_INVALID_CHANNEL;
		wilc_wlan_set_bssid(dev, NULL, WILC_STATION_MODE);
		wfi_drv->conn_info.conn_result = NULL;
		kfree(join_params);
		goto out_put_bss;
	}
	kfree(join_params);
	vif->bss = bss;
	cfg80211_put_bss(wiphy, bss);
	return 0;

out_put_bss:
	cfg80211_put_bss(wiphy, bss);

out_error:
	vif->connecting = false;
	return ret;
}
#endif

int wilc_cfg80211_init(struct wilc_dev *wilc, int io_type,
		       const struct wilc_hif_func *ops)
{
	int i, ret;
	//struct wilc *wl;
	struct wilc_vif *vif;
	(void)vif; //FIXME

	//wl = wilc_create_wiphy(dev);
	//if (!wl) {
	//	pr_err("failed to create wiphy\n");
	//	return -EINVAL;
	//}

	wlan_init_locks(wilc);

	ret = cfg_init(wilc);
	if (ret)
		goto free_wl;

	///wilc_debugfs_init();
	//*wilc = wl;
	wilc->io_type = io_type;

	wilc->hif_func = ops;


	for (i = 0; i < NQUEUES; i++)
			INIT_LIST_HEAD(&wilc->txq[i].txq_head.list);

	INIT_LIST_HEAD(&wilc->rxq_head.list);
	INIT_LIST_HEAD(&wilc->vif_list);


	//FIXME
	/* struct mq_attr attr;

	attr.mq_flags = 0;
	attr.mq_maxmsg = 10;
	attr.mq_msgsize = 256;
	attr.mq_curmsgs = 0;*/

	wilc->hif_workqueue = create_singlethread_workqueue("WILC_wq");
	if (!wilc->hif_workqueue) {
		ret = -1;
		goto free_debug_fs;
	}

	vif = wilc_netdev_ifc_init(wilc, "wlan%d", WILC_STATION_MODE,
			NL80211_IFTYPE_STATION, false);


	//if (IS_ERR(vif)) {
	//	ret = PTR_ERR(vif);
	//	goto free_wq;
	//}

	//wilc_sysfs_init(wl);

	return 0;
//free_wq:
	///destroy_workqueue(wilc->hif_workqueue);
free_debug_fs:
	///wilc_debugfs_remove();
	cfg_deinit(wilc);
free_wl:
	wlan_deinit_locks(wilc);
	//wiphy_unregister(wl->wiphy);
	//wiphy_free(wl->wiphy);
	return ret;
}


void wlan_init_locks(struct wilc_dev *wl)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"Initializing Locks ...\n\n");
	pthread_mutex_init(&wl->vif_mutex, NULL);
	pthread_mutex_init(&wl->rxq_cs, NULL);
	pthread_mutex_init(&wl->cfg_cmd_lock, NULL);
	pthread_mutex_init(&wl->deinit_lock, NULL);
	pthread_mutex_init(&wl->hif_cs, NULL);
	pthread_mutex_init(&wl->cs, NULL);
	pthread_mutex_init(&wl->txq_event_mutex, NULL);
	pthread_mutex_init(&wl->debug_thread_started_mutex, NULL);

	pthread_spin_init(&wl->txq_spinlock, PTHREAD_PROCESS_PRIVATE);
	pthread_mutex_init(&wl->txq_add_to_head_cs, NULL);

	pthread_cond_init( &wl->txq_event, NULL );
	pthread_cond_init(&wl->debug_thread_started, NULL);


	//wl->txq_event = 0;

	wl->cfg_event = 0;
	wl->sync_event = 0;
	wl->txq_thread_started = 0;
	///init_srcu_struct(&wl->srcu);
}

void wlan_deinit_locks(struct wilc_dev *wl)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"De-Initializing Locks\n");
	pthread_mutex_destroy(&wl->hif_cs);
	pthread_mutex_destroy(&wl->rxq_cs);
	pthread_mutex_destroy(&wl->cfg_cmd_lock);
	pthread_mutex_destroy(&wl->vif_mutex);
	pthread_mutex_destroy(&wl->txq_add_to_head_cs);
	pthread_mutex_destroy(&wl->cs);
	pthread_mutex_destroy(&wl->deinit_lock);
	pthread_mutex_destroy(&wl->txq_event_mutex);
	pthread_mutex_destroy(&wl->debug_thread_started_mutex);
	///cleanup_srcu_struct(&wl->srcu);
	pthread_cond_destroy(&wl->debug_thread_started);
	pthread_cond_destroy(&wl->txq_event);
}


int wilc_init_host_int(struct wilc_vif *vif)
{
	int ret;
	//struct wilc_vif *vif = netdev_priv(net);
	struct wilc_priv *priv = &vif->priv;

	///slogf(_SLOGC_NETWORK, _SLOG_ERROR,"Host[%p][%p]\n", net, net->ieee80211_ptr);

	fprintf(stderr, "[%s] create timer\n", __func__);
	SIGEV_THREAD_INIT(&priv->eap_buff_timer_event, eap_buff_timeout, vif, 0);
	if (timer_create(CLOCK_REALTIME, &priv->eap_buff_timer_event, &priv->eap_buff_timer) == -1)
	{
		fprintf(stderr, "[%s] create timer fail\n", __func__);
	}
	//setup_timer(&priv->eap_buffer_eventr, eap_buff_timeout, 0);

	vif->p2p_listen_state = false;

	pthread_mutex_init(&priv->scan_req_lock, NULL);
	ret = wilc_init(vif, &priv->hif_drv);
	if (ret)
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"Error while initializing hostinterface\n");

	return ret;
}

void wilc_deinit_host_int(struct wilc_vif *vif)
{
	int ret;
	//struct wilc_vif *vif = netdev_priv(net);
	struct wilc_priv *priv = &vif->priv;

	vif->p2p_listen_state = false;

	///flush_workqueue(vif->wilc->hif_workqueue);
	pthread_mutex_destroy(&priv->scan_req_lock);
	ret = wilc_deinit(vif);

	//del_timer_sync(&priv->eap_buff_timer);
	timer_delete(priv->eap_buff_timer);

	if (ret)
		PRINT_ER(net, "Error while deinitializing host interface\n");
}
