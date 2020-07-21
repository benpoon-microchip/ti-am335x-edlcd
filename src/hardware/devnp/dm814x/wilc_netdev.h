#ifndef WILC_NETDEV_H
#define WILC_NETDEV_H

extern int wait_for_recovery;

static inline void ether_addr_copy(uint8_t *dst, const uint8_t *src)
{

	uint16_t *a = (uint16_t *)dst;
	const uint16_t *b = (const uint16_t *)src;

	a[0] = b[0];
	a[1] = b[1];
	a[2] = b[2];

}



static inline bool ether_addr_equal_unaligned(const u8 *addr1, const u8 *addr2)
{

	return memcmp(addr1, addr2, ETH_ALEN) == 0;
}


void wilc_mac_indicate(struct wilc_dev *wilc);
void wilc_wlan_set_bssid(struct wilc_vif *vif, u8 *bssid, u8 mode);
int wlan_initialize_threads(struct wilc_dev *wilc);

#endif
