/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2012 - 2018 Microchip Technology Inc., and its subsidiaries.
 * All rights reserved.
 */


#ifndef NM_WFI_CFGOPERATIONS
#define NM_WFI_CFGOPERATIONS
#include "wilc_wfi_netdevice.h"
#include "ieee80211.h"
#include "type_defs.h"
#include "cfg80211.h"








struct wiphy {
  u8 perm_addr[ETH_ALEN];
  u8 addr_mask[ETH_ALEN];
};






bool wilc_wfi_p2p_rx(struct wilc_vif *vif, u8 *buff, u32 size);
void cfg_connect_result(enum conn_event conn_disconn_evt,u8 mac_status, void *priv_data);
void wilc_wlan_set_bssid(struct wilc_vif *vif, u8 *bssid, u8 mode);
//int wilc_cfg80211_init(struct wilc_dev *wilc, int io_type, onst struct wilc_hif_func *ops);

#endif
