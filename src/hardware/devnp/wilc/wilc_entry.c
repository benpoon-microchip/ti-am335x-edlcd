/*
 * Copyright (c) 2007, 2014, 2015 QNX Software Systems. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <io-pkt/iopkt_driver.h>
#include <sys/io-pkt.h>
#include <sys/syspage.h>
#include <sys/device.h>
#include <device_qnx.h>
#include <net/if_ether.h>
#include <net/if_media.h>
#include <net/netbyte.h>
#include <sys/slogcodes.h>
#include <sys/malloc.h>
#include <sys/sockio.h>
#include <sys-nto/bpfilter.h>
#if 1
#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_ioctl.h>
#endif
#include "wilc_main.h"
#include "wilc_wlan.h"
#include "wilc_wlan_if.h"
#include "wilc_hif.h"
#include "wilc_netdev.h"
#include "wilc_wfi_netdevice.h"
#include "wilc_wifi_cfgoperations.h"
#include "cfg80211.h"
#include <proto.h>

#if defined(__NetBSD__) || defined(__QNXNTO__)
#define IS_UP(_ic) \
	    (((_ic)->ic_ifp->if_flags & IFF_UP) &&          \
		         ((_ic)->ic_ifp->if_flags & IFF_RUNNING))
#endif
#define IS_UP_AUTO(_ic) \
	    (IS_UP(_ic) && (_ic)->ic_roaming == IEEE80211_ROAMING_AUTO)

int wilc_entry(void *dll_hdl, struct _iopkt_self *iopkt, char *options);
int wilc_drv_init(struct ifnet *);
void wilc_stop(struct ifnet *, int);
void wilc_start(struct ifnet *);
int wilc_ioctl(struct ifnet *, unsigned long, caddr_t);
int wilc_process_interrupt(void *, struct nw_work_thread *);
int wilc_enable_interrupt(void *);
void wilc_shutdown(void *);

const struct sigevent * wilc_isr(void *, int);

struct _iopkt_drvr_entry IOPKT_DRVR_ENTRY_SYM(wilc) = IOPKT_DRVR_ENTRY_SYM_INIT(wilc_entry);

#ifdef VARIANT_a
#include <nw_dl.h>
/* This is what gets specified in the stack's dl.c */
struct nw_dll_syms wilc_syms[] = {
        {"iopkt_drvr_entry", &IOPKT_DRVR_ENTRY_SYM(wilc)},
        {NULL, NULL}
};
#endif

const uint8_t etherbroadcastaddr[ETHER_ADDR_LEN] =
    { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };


#define	WILC_ATTACHED		0x0001		/* attach has succeeded */
#define WILC_ENABLED		0x0002		/* chip is enabled */



int wilc_attach(struct device *, struct device *, void *);
int wilc_detach(struct device *, int);

struct wilc_vif* vif;
extern struct scan_results scan_result[10];
extern int scan_num;
extern int scan_finish;
int scan_idx;

CFATTACH_DECL(wilc,
	sizeof(struct wilc_dev),
	NULL,
	wilc_attach,
	wilc_detach,
	NULL);

#if 0
int wilc_reset(struct ifnet *ifp)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[ENTRY_DEBUG] wilc_reset()\n");
	return 0;
}

static void wilc_newassoc(struct ieee80211_node *ni, int isnew)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[ENTRY_DEBUG] wilc_newassoc()\n");
}
static void wilc_updateslot(struct ifnet *ifp)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[ENTRY_DEBUG] wilc_updateslot()\n");
}
static int wilc_wme_update(struct ieee80211com *ic)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[ENTRY_DEBUG] wilc_wme_update()\n");
	return 0;
}
#endif
static void wilc_watchdog(struct ifnet *ifp)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[ENTRY_DEBUG] wilc_watch()\n");
}
#if 0
struct ieee80211_node * wilc_node_alloc(struct ieee80211com *ic)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[ENTRY_DEBUG] wilc_node_alloc()\n");
	//return ic;
}
#endif
int wilc_media_change(struct ifnet *ifp)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[ENTRY_DEBUG] wilc_media_change()\n");
	int error;

	error = ieee80211_media_change(ifp);
	if (error == ENETRESET) {
		error = 0;
	}
	return error;
}

#if 0
int wilc_newstate(struct ieee80211com *ic, enum ieee80211_state nstate, int arg)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR, "[ENTRY_DEBUG] wilc_newstate()");
	//slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[ENTRY_DEBUG] wilc_newstate(), nstate=%d, arg=%d\n", nstate, arg);
	//slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[ENTRY_DEBUG] wilc_newstate() %s -> %s\n", ieee80211_state_name[ic->ic_state],ieee80211_state_name[nstate]);
	return 0;
}

u_int8_t wilc_node_getrssi(struct ieee80211com *ic, const struct ieee80211_node *ni)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[ENTRY_DEBUG] wilc_node_getrssi()\n");
	return 0;
}
#endif

int wilc_enable(struct wilc_dev *sc)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[ENTRY_DEBUG] wilc_enable()\n");
	sc->sc_flags |= WILC_ENABLED;
	return 0;
}

int wilc_disable(struct wilc_dev *sc)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[ENTRY_DEBUG] wilc_disable()\n");
	sc->sc_flags &= ~WILC_ENABLED;
	return 0;
}

int wilc_startrecv(struct wilc_dev *sc)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[ENTRY_DEBUG] wilc_startrecv()\n");
	return 0;
}

static void wilc_tx_complete(void *priv, int status)
{
	struct tx_complete_data *pv_data = priv;

	if (status == 1)
	{
		//slogf(_SLOGC_NETWORK, _SLOG_ERROR,"Packet sentSize= %d Add= %p \n", pv_data->size, pv_data->buff);
	}
	else
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"Couldn't send pkt Size= %d Add= %p\n", pv_data->size, pv_data->buff);

	free_ptr(pv_data);
}

int wilc_drv_init_extra(struct wilc_dev *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ifnet *ifp = ic->ic_ifp;
	struct ieee80211_node *ni;
	enum ieee80211_phymode mode;
	int error = 0;

	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] In\n", __func__);
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] ifp= %p!!!", __func__, ifp);

	if ((error = wilc_enable(sc)) != 0)
	{
		return error;
	}
	//s = splnet();

	/*
	 * Stop anything previously setup.  This is safe
	 * whether this is the first time through or not.
	 */
	//ath_stop(ifp);

	/*
	 * Reset the link layer address to the latest value.
	 */
	IEEE80211_ADDR_COPY(ic->ic_myaddr, LLADDR(ifp->if_sadl));

#if 0
	//ath_hal_set_lladdr(w_hal, ic->ic_myaddr);

	/*
	 * The basic interface to setting the hardware in a good
	 * state is ``reset''.  On return the hardware is known to
	 * be powered up and with interrupts disabled.  This must
	 * be followed by initialization of the appropriate bits
	 * and then setup of the interrupt mask.
	 */
	//hchan.channel = ic->ic_ibss_chan->ic_freq;
	//hchan.channelFlags = ic->ic_ibss_chan->ic_flags;
	//if (!ath_hal_reset(w_hal, ic->ic_opmode, &hchan, AH_TRUE, &status)) {
	//	printf("%s: unable to reset hardware; hal status %u\n",
	//		ifp->if_xname, status);
	//	error = EIO;
	//	goto done;
	//}
	//

	ath_set_slot_time(sc);

	if ((error = ath_initkeytable(sc)) != 0) {
		printf("%s: unable to reset the key cache\n",
		    ifp->if_xname);
		goto done;
	}
#endif

	if ((error = wilc_startrecv(sc)) != 0) {
		printf("%s: unable to start recv logic\n", ifp->if_xname);
		goto done;
	}

	/*
	 * Enable interrupts.
	 */
	//sc->sc_imask = HAL_INT_RX | HAL_INT_TX
	//    | HAL_INT_RXEOL | HAL_INT_RXORN
	//    | HAL_INT_FATAL | HAL_INT_GLOBAL;
#ifndef IEEE80211_STA_ONLY
	//if (ic->ic_opmode == IEEE80211_M_HOSTAP)
		//sc->sc_imask |= HAL_INT_MIB;
#endif
	///ath_hal_set_intr(w_hal, sc->sc_imask);

	ifp->if_flags |= IFF_RUNNING;
	ic->ic_state = IEEE80211_S_INIT;

	/*
	 * The hardware should be ready to go now so it's safe
	 * to kick the 802.11 state machine as it's likely to
	 * immediately call back to us to send mgmt frames.
	 */

	ic->ic_bss = malloc(sizeof(struct ieee80211_node), M_DEVBUF, M_NOWAIT | M_ZERO);
	//ic->ic_ibss_chan = malloc(sizeof(struct ieee80211_channel), M_DEVBUF, M_NOWAIT | M_ZERO);
	ic->ic_ibss_chan = &ic->ic_channels[5];
	ic->ic_des_chan = &ic->ic_channels[5];
	ic->ic_curchan = &ic->ic_channels[5];

	ni = ic->ic_bss;
	ni->ni_chan = ic->ic_ibss_chan;
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] freq=%d\n", __func__, ni->ni_chan->ic_freq);
	mode = ieee80211_chan2mode(ic, ni->ni_chan);
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] mode=%d\n", __func__, mode);
	int test=ieee80211_chan2ieee(ic, ic->ic_ibss_chan);
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] test=%d\n", __func__, test);

	//if (mode != sc->sc_curmode)
	//	ath_setcurmode(sc, mode);


	if (ic->ic_opmode != IEEE80211_M_MONITOR) {
		// comment below as system crash with unkown reason
		//ieee80211_new_state(ic, IEEE80211_S_SCAN, -1);

	} else {
		//ieee80211_new_state(ic, IEEE80211_S_RUN, -1);
	}
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] out\n", __func__);
done:
	//splx(s);
	return error;
}

#if 0
int wilc_send_mgmt(struct ieee80211com *ic,
				    struct ieee80211_node *ni, int arg1, int arg2)
{
	return 0;
}

void wilc_recv_mgmt(struct ieee80211com *ic, struct mbuf *m,
    struct ieee80211_node *ni, int arg1, int arg2, u_int32_t arg3)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[ENTRY_DEBUG] wilc_recv_mgmt()\n");
}

void wilc_node_free(struct ieee80211com *ic, struct ieee80211_node *ni)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[ENTRY_DEBUG] wilc_node_free()\n");
}
#endif

struct channel chan[] = {{2412, IEEE80211_CHAN_2GHZ},{2417, IEEE80211_CHAN_2GHZ}, {2422, IEEE80211_CHAN_2GHZ}, {2427, IEEE80211_CHAN_2GHZ}, {2432, IEEE80211_CHAN_2GHZ}, \
						{2437, IEEE80211_CHAN_2GHZ}, {2442, IEEE80211_CHAN_2GHZ}, {2447, IEEE80211_CHAN_2GHZ}, {2452, IEEE80211_CHAN_2GHZ}, {2457, IEEE80211_CHAN_2GHZ}, \
						{2462, IEEE80211_CHAN_2GHZ}, {2467, IEEE80211_CHAN_2GHZ}, {2472, IEEE80211_CHAN_2GHZ},{2484, IEEE80211_CHAN_2GHZ}};
#define NUM_CHANNEL_SUPPORTED	14

int wilc_getchannels(struct wilc_dev *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ifnet *ifp = ic->ic_ifp;
	int i;
	u_int ix;


	/*
	 * Convert HAL channels to ieee80211 ones and insert
	 * them in the table according to their channel number.
	 */
	for (i = 0; i < NUM_CHANNEL_SUPPORTED; i++) {
		ix = ieee80211_mhz2ieee(chan[i].freq, chan[i].flags);
		if (ix > IEEE80211_CHAN_MAX) {
			slogf(_SLOGC_NETWORK, _SLOG_INFO, "[%s] %s bad hal channel %u (%u/%x) ignored", __func__, ifp->if_xname, ix, chan[i].freq, chan[i].flags);
			continue;
		}
		slogf(_SLOGC_NETWORK, _SLOG_INFO, "[%s] ix=%d freq= %d", __func__, ix,  chan[i].freq);

		/* NB: flags are known to be compatible */
		if (ic->ic_channels[ix].ic_freq == 0) {
			ic->ic_channels[ix].ic_freq = chan[i].freq;
			ic->ic_channels[ix].ic_flags = chan[i].flags;
		} else {
			/* channels overlap; e.g. 11g and 11b */
			ic->ic_channels[ix].ic_flags |= chan[i].flags;
		}

	}

	/* set an initial channel */
	ic->ic_ibss_chan = &ic->ic_channels[0];

	return 0;
}

static const struct ieee80211_rateset ieee80211_rateset_11b =
{ 5, { 1, 2, 5 ,11 } };
static const struct ieee80211_rateset ieee80211_rateset_11g =
{ 8, { 6, 9, 12, 18, 24, 36, 48, 54 } };
static const struct ieee80211_rateset ieee80211_rateset_11n =
{ 8, { 7, 14, 21, 28, 43, 57, 65, 72 } };

int wilc_rate_setup(struct wilc_dev *sc, u_int mode)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ieee80211_rateset *rs;
	int i;

	rs = &ic->ic_sup_rates[mode];

	switch (mode) {
	case IEEE80211_MODE_11B:

		for (i = 0; i < ieee80211_rateset_11b.rs_nrates; i++)
			rs->rs_rates[i] = ieee80211_rateset_11b.rs_rates[i] ;
		rs->rs_nrates = ieee80211_rateset_11b.rs_nrates;

		break;
	case IEEE80211_MODE_11G:
		for (i = 0; i < ieee80211_rateset_11g.rs_nrates; i++)
			rs->rs_rates[i] = ieee80211_rateset_11g.rs_rates[i] ;
		rs->rs_nrates = ieee80211_rateset_11g.rs_nrates;
		break;
	//case IEEE80211_MODE_11N:
		//break;
	default:
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] invalid mode %u\n", __func__, mode);
		return 1;
	}

	return 0;
}


/*
 * Initial driver entry point.
 */
int wilc_entry(void *dll_hdl,  struct _iopkt_self *iopkt, char *options)
{
	int		instance, single;
	struct device	*dev;
	void	*attach_args;

	/* parse options */
	slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] In\n", __func__);

	/* do options imply single? */
	single = 1;

	/* initialize to whatever you want to pass to wilc_attach() */
	attach_args = NULL;

	for (instance = 0;;) {
		/* Apply detection criteria */

		/* Found one */
		dev = NULL; /* No Parent */
		if (dev_attach("wlan", options, &wilc_ca, attach_args,
		    &single, &dev, NULL) != EOK) {
			break;
		}
		dev->dv_dll_hdl = dll_hdl;
		instance++;

		if (/* done_detection || */ single)
			break;
	}

	if (instance > 0)
		return EOK;

	return ENODEV;
}

extern int linux_sdio_probe(struct wilc_dev *wilc);

int wilc_attach(struct device *parent, struct device *self, void *aux)
{

	int	err;
	struct wilc_dev	*wilc;
	struct ifnet	*ifp;
	//uint8_t	enaddr[ETHER_ADDR_LEN];
	//struct qtime_entry	*qtp;
	struct ieee80211com *ic;
	int ret;
	unsigned char mac_add[ETH_ALEN] = {0};

	slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] In\n", __func__);

	/* initialization and attach */

	wilc = (struct wilc_dev *)self;
	ifp = &wilc->sc_ec.ec_if;
	ic = &wilc->sc_ic;
#if 0
	ifp = &wilc->sc_ec.ec_if;
#else

	linux_sdio_probe(wilc);

	vif = wilc_get_wl_to_vif(wilc); //FIXME: validating vif
	//slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] DEBUG: vif ptr =%p\n", __func__, vif);

	ret = wilc_init_host_int(vif);
	if (ret < 0) {
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"Failed to initialize host interface\n");
		return ret;
	}

	/*
	 * Construct channel list based on the current regulation domain.
	 */
	ret = wilc_getchannels(wilc);
	if (ret != 0)
		return -EIO;

	wilc_rate_setup(wilc, IEEE80211_MODE_11B);
	wilc_rate_setup(wilc, IEEE80211_MODE_11G);
#endif

//  For interrupt test only, no use
	wilc->sc_iopkt = iopkt_selfp;
	wilc->sc_irq = 41;


	if ((err = interrupt_entry_init(&wilc->sc_inter, 0, NULL,
	    IRUPT_PRIO_DEFAULT)) != EOK)
		return err;

	wilc->sc_inter.func   = wilc_process_interrupt;
	wilc->sc_inter.enable = wilc_enable_interrupt;
	wilc->sc_inter.arg    = wilc;

	wilc->sc_iid = -1; /* not attached yet */

	wilc->sc_sdhook = shutdownhook_establish(wilc_shutdown, wilc);

	pthread_mutex_init(&wilc->rx_mutex, NULL);
	IFQ_SET_MAXLEN(&wilc->rx_queue, IFQ_MAXLEN);
	wilc->rx_queue.ifq_tail = NULL;
	wilc->rx_queue.ifq_head = NULL;
	INIT_LIST_HEAD(&wilc->rx_q.list);

	/* set capabilities */
#if 0
	ifp->if_capabilities_rx = IFCAP_CSUM_IPv4 | IFCAP_CSUM_TCPv4 | IFCAP_CSUM_UDPv4;
	ifp->if_capabilities_tx = IFCAP_CSUM_IPv4 | IFCAP_CSUM_TCPv4 | IFCAP_CSUM_UDPv4;

	wilc->sc_ec.ec_capabilities |= ETHERCAP_JUMBO_MTU;
#endif

	ifp->if_softc = wilc;
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	/* Set callouts */
	ifp->if_ioctl = wilc_ioctl;
	ifp->if_start = wilc_start;
	ifp->if_init = wilc_drv_init;
	ifp->if_watchdog = wilc_watchdog;
	ifp->if_stop = wilc_stop;
	IFQ_SET_READY(&ifp->if_snd);

	/* More callouts for 80211... */
	strcpy(ifp->if_xname, wilc->sc_dev.dv_xname);


	//ic->ic_reset = wilc_reset;
	//ic->ic_newassoc = wilc_newassoc;
	//ic->ic_updateslot = wilc_updateslot;
	//ic->ic_wme.wme_update = wilc_wme_update;
	ic->ic_ifp = ifp;
	ic->ic_phytype = IEEE80211_T_OFDM;
	ic->ic_opmode = IEEE80211_M_STA;
	ic->ic_caps =	IEEE80211_C_IBSS	/* ibss, nee adhoc, mode */
						//| IEEE80211_C_HOSTAP	/* hostap mode */
						//| IEEE80211_C_MONITOR	/* monitor mode */
						| IEEE80211_C_SHPREAMBLE	/* short preamble supported */
						| IEEE80211_C_SHSLOT	/* short slot time supported */
						| IEEE80211_C_WPA	/* capable of WPA1+WPA2 */
						| IEEE80211_C_TXFRAG	/* handle tx frags */
						| IEEE80211_C_WEP;


	/* Bump MAC addr per interface */
	wilc->mac_status = WILC_MAC_STATUS_INIT;

	wilc_mac_open(vif, mac_add);

	memcpy(ic->ic_myaddr, mac_add, ETHER_ADDR_LEN);
	slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] ic->ic_myaddr = 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x \n", __func__, ic->ic_myaddr[0], ic->ic_myaddr[1], ic->ic_myaddr[2], ic->ic_myaddr[3], ic->ic_myaddr[4], ic->ic_myaddr[5]);


	if_attach(ifp);

#if 0
	/* Normal ethernet */
	ether_ifattach(ifp, enaddr);
#else
	/* 80211 */
	//memcpy(ic->ic_myaddr, enaddr, ETHER_ADDR_LEN); //TODO
	ieee80211_ifattach(&wilc->sc_ic);

#endif

	/* override default methods */
	//ic->ic_node_alloc = wilc_node_alloc;
	//ic->ic_node_free = wilc_node_free;
	//ic->ic_node_getrssi = wilc_node_getrssi;
	//ic->ic_newstate = wilc_newstate;
	//ic->ic_recv_mgmt = wilc_recv_mgmt;
	//ic->ic_send_mgmt = wilc_send_mgmt;

	/* complete initialization */
	ieee80211_media_init(ic, wilc_media_change, ieee80211_media_status);


	slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] Out\n", __func__);

	return EOK;
}

#if 0
void wilc_set_multicast(struct wilc_dev *wilc)
{
	printf("sam_set_multicast()!!!\r\n");
	struct ethercom			*ec = &wilc->sc_ec;
	struct ifnet			*ifp = &ec->ec_if;
	struct ether_multi		*enm;
	struct ether_multistep		step;

	ifp->if_flags &= ~IFF_ALLMULTI;

	ETHER_FIRST_MULTI(step, ec, enm);
	while (enm != NULL) {
                if (memcmp(enm->enm_addrlo, enm->enm_addrhi, ETHER_ADDR_LEN)) {
                        /*
                         * We must listen to a range of multicast addresses.
                         * For now, just accept all multicasts, rather than
                         * trying to filter out the range.
                         * At this time, the only use of address ranges is
                         * for IP multicast routing.
                         */
                        ifp->if_flags |= IFF_ALLMULTI;
			break;
                }
		/* Single address */
		printf("Add %2x:%2x:%2x:%2x:%2x:%2x to mcast filter\n",
		       enm->enm_addrlo[0],enm->enm_addrlo[1],
		       enm->enm_addrlo[0],enm->enm_addrlo[1],
		       enm->enm_addrlo[0],enm->enm_addrlo[1]);
	}

	if ((ifp->if_flags & IFF_ALLMULTI) != 0) {
		printf("Enable multicast promiscuous\n");
	} else {
		printf("Disable multicast promiscuous\n");
	}
}
#endif

int wilc_drv_init(struct ifnet *ifp)
{
	slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] In\n", __func__);

	int		ret;
	struct wilc_dev	*wilc;
	//struct ieee80211com *ic;

	wilc = ifp->if_softc;
	//ic = &wilc->sc_ic;
	///linux_sdio_probe(wilc);

	//ieee80211_ifattach(ic);
	/*
	 * - enable hardware.
	 *   - look at ifp->if_capenable_[rx/tx]
	 *   - enable promiscuous / multicast filter.
	 * - attach to interrupt.
	 *
	 *
	 */



	if(memcmp(wilc->cfg.current_address, LLADDR(ifp->if_sadl), ifp->if_addrlen)) {
		memcpy(wilc->cfg.current_address, LLADDR(ifp->if_sadl), ifp->if_addrlen);
		/* update the hardware */
	}

	if (wilc->sc_iid == -1) {
		if ((ret = InterruptAttach_r(wilc->sc_irq, wilc_isr,
			wilc, sizeof(*wilc), _NTO_INTR_FLAGS_TRK_MSK)) < 0) {

			return -ret;
		}

		wilc->sc_iid = ret;
	}

	//sam_set_multicast(sam);
	ifp->if_flags |= IFF_RUNNING;


	wilc_drv_init_extra(wilc);

	slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] out\n", __func__);

	return EOK;
}

void
wilc_stop(struct ifnet *ifp, int disable)
{
	slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] In\n", __func__);
	struct wilc_dev	*wilc;

	/*
	 * - Cancel any pending io
	 * - Clear any interrupt source registers
	 * - Clear any interrupt pending registers
	 * - Release any queued transmit buffers.
	 */

	wilc = ifp->if_softc;

	if (disable) {
		if (wilc->sc_iid != -1) {
			InterruptDetach(wilc->sc_iid);
			wilc->sc_iid = -1;
		}
		/* rxdrain */
	}

	ifp->if_flags &= ~IFF_RUNNING;
}

void wilc_start(struct ifnet *ifp)
{
	struct wilc_dev		*wilc;
	struct mbuf		*m;
	struct nw_work_thread	*wtp;
	struct tx_complete_data *tx_data = NULL;
	int queue_count;

	slogf(_SLOGC_NETWORK, _SLOG_INFO, "[%s] In\n", __func__);

	//char test_buf[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0xf0, 0x05, 0xe5, 0x47, 0x91, 0x08, 0x06, 0x00, 0x01,
	//		0x08, 0x00, 0x06, 0x04, 0x00, 0x01, 0xf8, 0xf0, 0x05, 0xe5, 0x47, 0x91, 0x00, 0x00, 0x00, 0x00,
	//		00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xa8, 0x02, 0x4f};


	wilc = ifp->if_softc;
	wtp = WTP;

	for (;;) {
		IFQ_POLL(&ifp->if_snd, m);
		if (m == NULL)
		{
			slogf(_SLOGC_NETWORK, _SLOG_ERROR,"m=NULL\n");
			break;
		}
		/*
		 * Can look at m to see if you have the resources
		 * to transmit it.
		 */

		IFQ_DEQUEUE(&ifp->if_snd, m);

#if NBPFILTER > 0
		if (ifp->if_bpf)
			bpf_mtap(ifp->if_bpf, m);
#endif

		///slogf(_SLOGC_NETWORK, _SLOG_ERROR,"buf1 len=%d, data=0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", m->m_hdr.mh_len, m->m_hdr.mh_data[0],m->m_hdr.mh_data[1], m->m_hdr.mh_data[2], m->m_hdr.mh_data[3], m->m_hdr.mh_data[4],m->m_hdr.mh_data[5], m->m_hdr.mh_data[6], m->m_hdr.mh_data[7], m->m_hdr.mh_data[8], m->m_hdr.mh_data[9], m->m_hdr.mh_data[10], m->m_hdr.mh_data[11], m->m_hdr.mh_data[12], m->m_hdr.mh_data[13], m->m_hdr.mh_data[14], m->m_hdr.mh_data[15]);
		//slogf(_SLOGC_NETWORK, _SLOG_ERROR,"buf2 len=%d, data=0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", m->m_pkthdr.len, m->m_dat[0], m->m_dat[1], m->m_dat[2], m->m_dat[3], m->m_dat[4], m->m_dat[5], m->m_dat[6], m->m_dat[7], m->m_dat[8], m->m_dat[9], m->m_dat[10], m->m_dat[11], m->m_dat[12], m->m_dat[13], m->m_dat[14], m->m_dat[15]);
		///slogf(_SLOGC_NETWORK, _SLOG_ERROR,"buf2 len=%d, data=0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x", m->m_pkthdr.len, m->m_pktdat[0], m->m_pktdat[1], m->m_pktdat[2], m->m_pktdat[3], m->m_pktdat[4], m->m_pktdat[5], m->m_pktdat[6], m->m_pktdat[7], m->m_pktdat[8], m->m_pktdat[9], m->m_pktdat[10], m->m_pktdat[11], m->m_pktdat[12], m->m_pktdat[13], m->m_pktdat[14], m->m_pktdat[15]);
		/* You're now committed to transmitting it */
		if (wilc->cfg.verbose) {
			printf("Packet sent\n");
		}

		tx_data = create_ptr(sizeof(*tx_data));
		if (!tx_data) {
			slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] Failed to alloc memory for tx_data struct\n", __func__);
			m_freem(m);
			break;
		}

		//tx_data->buff = m->m_dat;
		//tx_data->size = m->m_pkthdr.len;
		/*
		tx_data->buff = m->m_hdr.mh_data;
		tx_data->size = m->m_hdr.mh_len;
		*/
		//tx_data->skb  = skb;

		//Test: send ARP packet
		///tx_data->buff = test_buf;
		///tx_data->size = sizeof(test_buf);

		tx_data->buff = create_ptr(m->m_pkthdr.len);
		m_copydata(m, 0, m->m_pkthdr.len, tx_data->buff);
		tx_data->size = m->m_pkthdr.len;

		// print the frag packet
		/*
		struct mbuf		*m2;
		int			num_frags;


		for (num_frags = 0, m2 = m; m2; num_frags++) {
		m2 = m2->m_next;
		if (m2)
			fprintf(stderr,"m2 len=%d, data=0x%x 0x%x 0x%x 0x%x\r\n", m2->m_hdr.mh_len, m2->m_hdr.mh_data[0],m2->m_hdr.mh_data[1], m2->m_hdr.mh_data[2], m2->m_hdr.mh_data[3]);
		}
		fprintf(stderr,"num_frags=%d\n", num_frags);
		*/

		//fprintf(stderr, "m->m_pkthdr.len = %d\n", m->m_pkthdr.len);

		//if (m->m_hdr.mh_nextpkt != NULL)
		//	fprintf(stderr, "m->m_hdr.mh_nextpkt != NULL\n");

		//if (m->m_hdr.mh_next != NULL)
		//	fprintf(stderr, "m->m_hdr.mh_next != NULL\n");

		char* text = tx_data->buff;
		///slogf(_SLOGC_NETWORK, _SLOG_ERROR,"buf1 data=0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\r\n",  text[0], text[1], text[2], text[3], text[4], text[5], text[6], text[7], text[8], text[9], text[10], text[11], text[12], text[13], text[14], text[15]);
		///slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] Sending pkt Size= %d Add= %p \n", __func__, tx_data->size, tx_data->buff);
		///slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] Adding tx pkt to TX Queue\n", __func__);

		vif->netstats.tx_packets++;
		vif->netstats.tx_bytes += tx_data->size;
		tx_data->vif = vif;
		queue_count = txq_add_net_pkt(vif, (void *)tx_data,
						  tx_data->buff, tx_data->size,
						  wilc_tx_complete);

		if (queue_count > FLOW_CTRL_UP_THRESHLD) {
			//struct wilc_vif *vif;
			//int srcu_idx;
			slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] queue_count is overflow\n", __func__);
			//srcu_idx = srcu_read_lock(&wilc->srcu);
			//list_for_each_entry(vif, &wilc->vif_list, list) {
			//	if (vif->mac_opened)
			//		netif_stop_queue(vif->ndev);
			//}
			//srcu_read_unlock(&wilc->srcu, srcu_idx);
		}


		m_freem(m);

		ifp->if_opackets++;  // for ifconfig -v
		// or if error:  ifp->if_oerrors++;
	}

	NW_SIGUNLOCK_P(&ifp->if_snd_ex, iopkt_selfp, wtp);


	//struct _iopkt_inter	*ient;
	//ient = &wilc->sc_inter;
	//interrupt_queue(wilc->sc_iopkt, ient);

}

void set_ssid_func()
{
	struct wilc_priv *priv = &vif->priv;
	struct host_if_drv *wfi_drv = priv->hif_drv;
	struct wilc_conn_info *conn_info = &wfi_drv->conn_info;

	conn_info->security = 0;
	conn_info->auth_type = 1;

}

static int
wilc_ioctl_set80211(struct wilc_dev* sc, struct ieee80211com *ic, u_long cmd,
    struct ieee80211req *ireq)
{
    int error;
    struct ifnet *ifp = sc->sc_ic.ic_ifp;
    int cnt = 0;
    struct ieee80211req_mlme mlme;
    struct ieee80211req_mlme *mlme_req = &mlme;
    struct ieee80211req_key wk;
    struct ieee80211req_key *wk_req = &wk;

    struct wilc_priv *priv = &vif->priv;
    struct host_if_drv *wfi_drv = priv->hif_drv;
    static struct cfg80211_crypto_settings crypto;
    struct cfg80211_bss bss;

    size_t offset;
    void *join_params;
    int ap_found = 0;
    static int test_flag = 0;
    u8 *ies, *rsn_ie;
    struct cfg80211_bss_ies bss_ies;
    int ret = 0;
    static int req_ie_len;
    u8 req_ie[100];
    int i = 0; // for counting

    //uint8_t test_bssid[6] = {0x10, 0x7b, 0x44, 0xea, 0x1c, 0xa8};
    uint8_t test_bssid[6] = {0x50, 0xc7, 0xbf, 0xae, 0xe1, 0x7a};
    //const struct ieee80211_authenticator *auth;

    slogf(_SLOGC_NETWORK, _SLOG_INFO, "SIOCS80211: i_type = %d\n", ireq->i_type);
    error = 0;
    switch (ireq->i_type) {
    case IEEE80211_IOC_WEP:
        switch (ireq->i_val) {
        case IEEE80211_WEP_OFF:
            ic->ic_flags &= ~IEEE80211_F_PRIVACY;
            ic->ic_flags &= ~IEEE80211_F_DROPUNENC;
            break;
        case IEEE80211_WEP_ON:
            ic->ic_flags |= IEEE80211_F_PRIVACY;
            ic->ic_flags |= IEEE80211_F_DROPUNENC;
            break;
        case IEEE80211_WEP_MIXED:
            ic->ic_flags |= IEEE80211_F_PRIVACY;
            ic->ic_flags &= ~IEEE80211_F_DROPUNENC;
            break;
        }
        error = ENETRESET;
        break;

    case IEEE80211_IOC_AUTHMODE:
		slogf(_SLOGC_NETWORK, _SLOG_INFO, "SIOCS80211: i_type = %d\n", ireq->i_type);
        slogf(_SLOGC_NETWORK, _SLOG_INFO, "IEEE80211_IOC_AUTHMODE set %d\n", ireq->i_val);
        switch (ireq->i_val) {
        case IEEE80211_AUTH_WPA:


			break;
        case IEEE80211_AUTH_8021X:  /* 802.1x */
        	break;
        case IEEE80211_AUTH_OPEN:   /* open */
			wfi_drv->conn_info.auth_type = WILC_FW_AUTH_OPEN_SYSTEM;
			slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] IEEE80211_IOC_AUTHMODE, conn_info.security = %d, conn_info.auth_type = %d ", __func__,wfi_drv->conn_info.security, wfi_drv->conn_info.auth_type);
			break;

        case IEEE80211_AUTH_SHARED: /* shared-key */
        	wfi_drv->conn_info.auth_type = WILC_FW_AUTH_SHARED_KEY;
        	break;
        case IEEE80211_AUTH_AUTO:   /* auto */
            //TODO: set auth mode
            break;
        default:
            return EINVAL;
        }
        switch (ireq->i_val) {
        case IEEE80211_AUTH_WPA:    /* WPA w/ 802.1x */
            ic->ic_flags |= IEEE80211_F_PRIVACY;
            ireq->i_val = IEEE80211_AUTH_8021X;
            break;
        case IEEE80211_AUTH_OPEN:   /* open */
            ic->ic_flags &= ~(IEEE80211_F_WPA|IEEE80211_F_PRIVACY);
            break;
        case IEEE80211_AUTH_SHARED: /* shared-key */
        case IEEE80211_AUTH_8021X:  /* 802.1x */
            ic->ic_flags &= ~IEEE80211_F_WPA;
            /* both require a key so mark the PRIVACY capability */
            ic->ic_flags |= IEEE80211_F_PRIVACY;
            break;
        case IEEE80211_AUTH_AUTO:   /* auto */
            ic->ic_flags &= ~IEEE80211_F_WPA;
            /* XXX PRIVACY handling? */
            /* XXX what's the right way to do this? */
            break;
        }
        /* NB: authenticator attach/detach happens on state change */
        ic->ic_bss->ni_authmode = ireq->i_val;
        /* XXX mixed/mode/usage? */
        //ic->ic_auth = auth;
        error = ENETRESET;
        break;

    case IEEE80211_IOC_ROAMING:
        slogf(_SLOGC_NETWORK, _SLOG_INFO, "IEEE80211_IOC_ROAMING set: %d\n", ireq->i_val);
        if (!(IEEE80211_ROAMING_DEVICE <= ireq->i_val &&
            ireq->i_val <= IEEE80211_ROAMING_MANUAL))
            return EINVAL;
        ic->ic_roaming = ireq->i_val;
        /* XXXX reset? */
        break;
    case IEEE80211_IOC_PRIVACY:
        slogf(_SLOGC_NETWORK, _SLOG_INFO, "IEEE80211_IOC_PRIVACY set: %d\n", ireq->i_val);
        if (ireq->i_val) {
            /* XXX check for key state? */
            ic->ic_flags |= IEEE80211_F_PRIVACY;
        }
        else
        {
            ic->ic_flags &= ~IEEE80211_F_PRIVACY;
            wfi_drv->conn_info.security = WILC_FW_SEC_NO;
        }
        break;
    case IEEE80211_IOC_DROPUNENCRYPTED:
        slogf(_SLOGC_NETWORK, _SLOG_INFO, "IEEE80211_IOC_DROPUNENCRYPTED set: %d\n", ireq->i_val);
        if (ireq->i_val)
            ic->ic_flags |= IEEE80211_F_DROPUNENC;
        else
            ic->ic_flags &= ~IEEE80211_F_DROPUNENC;
        break;
    case IEEE80211_IOC_WPAKEY:
    	slogf(_SLOGC_NETWORK, _SLOG_INFO, "IEEE80211_IOC_WPAKEY set: \n");
    	fprintf(stderr, "MCHP: IEEE80211_IOC_WPAKEY set: \n");

    	const u8 *rx_mic = NULL;
    	const u8 *tx_mic = NULL;
    	u8 mode = WILC_FW_SEC_NO;
    	u8 op_mode;
    	int keylen;
    	u8 key_index;

        slogf(_SLOGC_NETWORK, _SLOG_INFO, "IEEE80211_IOC_WPAKEY set: \n");
        //ieee80211req_key
        memset(wk_req, 0, sizeof(struct ieee80211req_key));
        error = copyin((char *)ireq + sizeof(*ireq), wk_req, sizeof(wk));

        fprintf(stderr, "MCHP: ik_type = 0x%x, ik_flags = 0x%x, ik_keyix = %u, ik_keylen = 0x%x\n", wk_req->ik_type, wk_req->ik_flags, wk_req->ik_keyix,  wk_req->ik_keylen);
        fprintf(stderr, "MCHP: ik_macaddr= 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n", wk_req->ik_macaddr[0], wk_req->ik_macaddr[1], wk_req->ik_macaddr[2],  wk_req->ik_macaddr[3], wk_req->ik_macaddr[4], wk_req->ik_macaddr[5]);
#if 1
        slogf(_SLOGC_NETWORK, _SLOG_INFO, "IEEE80211_IOC_WPAKEY set: log1 \n");
        keylen = wk_req->ik_keylen;
        switch (wk_req->ik_type)
        {
        	case IEEE80211_CIPHER_WEP:
        		// to do
        	break;
        	case IEEE80211_CIPHER_TKIP:
        		// to do
        		slogf(_SLOGC_NETWORK, _SLOG_INFO, "IEEE80211_IOC_WPAKEY set: log2 \n");
        		if (wk_req->ik_keylen > 16) {
					rx_mic = wk_req->ik_keydata + 24;
					tx_mic = wk_req->ik_keydata + 16;
					keylen = wk_req->ik_keylen - 16;
				}
        		op_mode = WILC_STATION_MODE;
        	break;
        	case IEEE80211_CIPHER_AES_CCM:

        		slogf(_SLOGC_NETWORK, _SLOG_INFO, "IEEE80211_IOC_WPAKEY set: log3 \n");
        		op_mode = WILC_STATION_MODE;
        	break;
        	default:
        		slogf(_SLOGC_NETWORK, _SLOG_INFO, "IEEE80211_IOC_WPAKEY set: log4 \n");
        	break;
        }

        slogf(_SLOGC_NETWORK, _SLOG_INFO, "IEEE80211_IOC_WPAKEY set: log5, wk_req->ik_flags = 0x%x \n", wk_req->ik_flags);
        if (wk_req->ik_flags & IEEE80211_KEY_GROUP)
        {
        	slogf(_SLOGC_NETWORK, _SLOG_INFO, "IEEE80211_IOC_WPAKEY set: log6 \n");
        	// group key
        	key_index = wk_req->ik_keyix;
        	fprintf(stderr, "IEEE80211_IOC_WPAKEY set:  wk_req->ik_keydata = 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x \n", wk_req->ik_keydata[0], wk_req->ik_keydata[1], wk_req->ik_keydata[2], wk_req->ik_keydata[3], wk_req->ik_keydata[4], wk_req->ik_keydata[5], wk_req->ik_keydata[6], wk_req->ik_keydata[7]);

        	ret = wilc_add_rx_gtk(vif, wk_req->ik_keydata, keylen,
        						      key_index, 6,
									  &wk_req->ik_keyrsc, rx_mic, tx_mic,
        						      op_mode, mode);

        }
        else
        {
        	slogf(_SLOGC_NETWORK, _SLOG_INFO, "IEEE80211_IOC_WPAKEY set: log7 \n");
        	// pairwise key
        	key_index = 0;
        	fprintf(stderr, "IEEE80211_IOC_WPAKEY set:  wk_req->ik_keydata = 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x \n", wk_req->ik_keydata[0], wk_req->ik_keydata[1], wk_req->ik_keydata[2], wk_req->ik_keydata[3], wk_req->ik_keydata[4], wk_req->ik_keydata[5], wk_req->ik_keydata[6], wk_req->ik_keydata[7]);
        	//wk_req->ik_keydata[0] = 0x10;
        	ret = wilc_add_ptk(vif, wk_req->ik_keydata, keylen, wk_req->ik_macaddr,
        						   rx_mic, tx_mic, op_mode, mode,
        						   key_index);

        }
#endif
        //TODO:
        break;
    case IEEE80211_IOC_DELKEY:
        slogf(_SLOGC_NETWORK, _SLOG_INFO, "IEEE80211_IOC_DELKEY set: \n");
        //TODO:
        break;
    case IEEE80211_IOC_MLME:

    	if (ireq->i_len != sizeof(mlme))
    	{
			error = EINVAL;
    	}
		else
		{
			error = copyin((char *)ireq + sizeof(*ireq), mlme_req, sizeof(mlme));

			fprintf(stderr, "IEEE80211_IOC_MLME i_value = %d, i_len = %d, im_op = %d, im_ssid_len = %d, im_ssid = %s, im_macaddr = 0x%x 0x%x 0x%x\n", ireq->i_val, ireq->i_len, mlme_req->im_op, mlme_req->im_ssid_len, mlme_req->im_ssid, mlme_req->im_macaddr[0], mlme_req->im_macaddr[1], mlme_req->im_macaddr[2]);
			slogf(_SLOGC_NETWORK, _SLOG_INFO, "IEEE80211_IOC_MLME i_value = %d, i_len = %d, im_op = %d, im_ssid_len = %d, im_ssid = %s, im_macaddr = 0x%x 0x%x 0x%x\n", ireq->i_val, ireq->i_len, mlme_req->im_op, mlme_req->im_ssid_len, mlme_req->im_ssid, mlme_req->im_macaddr[0], mlme_req->im_macaddr[1], mlme_req->im_macaddr[2]);
			//TODO:
			if ( mlme_req->im_op == IEEE80211_MLME_ASSOC)
			{
				ap_found = 0;
				for (cnt = 0; cnt<scan_num; cnt++)
				{
					if (!memcmp(scan_result[cnt].bssid, mlme_req->im_macaddr, IEEE80211_ADDR_LEN))
					{
						slogf(_SLOGC_NETWORK, _SLOG_INFO, "IEEE80211_IOC_MLME, Find AP \n");
						ap_found = 1;
						break;
					}
				}

				if (ap_found)
				//if ((ap_found) && (test_flag == 0))
				{
					test_flag = 1;

					if (ieee80211_is_probe_resp(scan_result[cnt].mgmt->frame_control))
					{
						offset = offsetof(struct ieee80211_mgmt, u.probe_resp.variable);
						slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] IEEE80211_IOC_MLME, probe_resp, offset = %d", __func__,offset);
					}
					else if (ieee80211_is_beacon(scan_result[cnt].mgmt->frame_control))
					{
						offset = offsetof(struct ieee80211_mgmt, u.beacon.variable);
						slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] IEEE80211_IOC_MLME, is_beacon, offset = %d", __func__, offset);
					}

					// assige the value to bss

					bss.ies = &bss_ies;

					ies = scan_result[cnt].mgmt->u.beacon.variable;
					bss.ies->data = scan_result[cnt].mgmt->u.beacon.variable;
					bss.ies->len = scan_result[cnt].frame_len - offset;

					bss.beacon_interval = scan_result[cnt].mgmt->u.beacon.beacon_int;
					bss.channel = scan_result[cnt].channel;
					bss.capability = scan_result[cnt].mgmt->u.beacon.capab_info;
					memcpy(bss.bssid, scan_result[cnt].bssid, IEEE80211_ADDR_LEN);

					// hardcode crypto value;

					slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] IEEE80211_IOC_MLME, crypto.cipher_group = 0x%x, crypto.n_ciphers_pairwise = 0x%x, crypto.ciphers_pairwise[0] = 0x%x, crypto.n_akm_suites = 0x%x, crypto.akm_suites[0] = 0x%x\n", __func__,crypto.cipher_group, crypto.n_ciphers_pairwise, crypto.ciphers_pairwise[0], crypto.n_akm_suites, crypto.akm_suites[0]);

					/*
					rsn_ie = cfg80211_find_ie(WLAN_EID_RSN, bss.ies->data, bss.ies->len);
					if (rsn_ie) {
						slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] IEEE80211_IOC_MLME, rsn_ie[8] = 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n", __func__,rsn_ie[8], rsn_ie[9], rsn_ie[10], rsn_ie[11],rsn_ie[12], rsn_ie[13]);
						req_ie = rsn_ie;
						req_ie_len = rsn_ie[1] + 2;

						//memcpy(&crypto.cipher_group, &rsn_ie[4], 4);
						crypto.cipher_group = rsn_ie[4]<<24 | rsn_ie[5]<<16 | rsn_ie[6]<<8 | rsn_ie[7];
						crypto.n_ciphers_pairwise = rsn_ie[8];
						//memcpy(&crypto.ciphers_pairwise[0], &rsn_ie[10], 4);
						crypto.ciphers_pairwise[0] = rsn_ie[10]<<24 | rsn_ie[11]<<16 | rsn_ie[12]<<8 | rsn_ie[13];
						crypto.n_akm_suites = rsn_ie[10 + 4*crypto.n_ciphers_pairwise];
						//memcpy(&crypto.akm_suites[0], &rsn_ie[10 + 4*crypto.n_ciphers_pairwise + 2], 4);
						crypto.akm_suites[0] = rsn_ie[10 + 4*crypto.n_ciphers_pairwise + 2]<<24 | rsn_ie[10 + 4*crypto.n_ciphers_pairwise + 2 + 1]<<16 | rsn_ie[10 + 4*crypto.n_ciphers_pairwise + 2 + 2]<<8 | rsn_ie[10 + 4*crypto.n_ciphers_pairwise + 2 +3];

						slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] IEEE80211_IOC_MLME, crypto.cipher_group = 0x%x, crypto.n_ciphers_pairwise = 0x%x, crypto.ciphers_pairwise[0] = 0x%x, crypto.n_akm_suites = 0x%x, crypto.akm_suites[0] = 0x%x\n", __func__,crypto.cipher_group, crypto.n_ciphers_pairwise, crypto.ciphers_pairwise[0], crypto.n_akm_suites, crypto.akm_suites[0]);
					}
					else
					{
						req_ie = NULL;
						req_ie_len = 0;
						crypto.cipher_group =0;
						crypto.n_ciphers_pairwise = 0;
						crypto.n_akm_suites = 0;
					}
					*/
					/*
					crypto.cipher_group =0;
					crypto.n_ciphers_pairwise = 3;
					crypto.n_akm_suites = 2;
					crypto.ciphers_pairwise[0] = 0xff;
					crypto.ciphers_pairwise[1] = 0xff;
					crypto.ciphers_pairwise[2] = 0xff;
					crypto.akm_suites[0] = 0xff;
					crypto.akm_suites[1] = 0xff;
					*/

					join_params = wilc_parse_join_bss_param(&bss, &crypto);
					//
					wilc_wlan_set_bssid(vif, scan_result[cnt].bssid, WILC_STATION_MODE);
					wfi_drv->conn_info.conn_result = cfg_connect_result;
					slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] IEEE80211_IOC_MLME, conn_info.conn_result = %p", __func__,wfi_drv->conn_info.conn_result);

					// hardcode for test
					//wfi_drv->conn_info.security = 0;
					//wfi_drv->conn_info.auth_type = WILC_FW_AUTH_OPEN_SYSTEM;
					//u8 ies[5] = {0x3b, 0x3, 0x51, 0x51, 0x52};

					slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] IEEE80211_IOC_MLME, conn_info.security = %d, conn_info.auth_type = %d ", __func__,wfi_drv->conn_info.security, wfi_drv->conn_info.auth_type);


					//

					wfi_drv->conn_info.ch = scan_result[cnt].channel;
					//wfi_drv->conn_info.arg = priv;
					wfi_drv->conn_info.param = join_params;
					//set_ssid_func();
					///wilc_set_join_req(vif, scan_result[cnt].bssid, ies, sizeof(ies));
					slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] IEEE80211_IOC_MLME, req_ie_len = %d", __func__,req_ie_len);
					wilc_set_join_req(vif, scan_result[cnt].bssid, req_ie, req_ie_len);
				}
			}
			else if ( mlme_req->im_op == IEEE80211_MLME_DISASSOC || mlme_req->im_op == IEEE80211_MLME_DEAUTH)
			{
				vif->connecting = false;
				wilc_wlan_set_bssid(vif, NULL, WILC_STATION_MODE);

				slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] Disconnecting AP", __func__);

				ret = wilc_disconnect(vif);
				if (ret != 0) {
					slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] Error in disconnecting AP", __func__);
				}

				//vif->bss = NULL;
			}
		}

        //TODO:
        break;
    case IEEE80211_IOC_OPTIE:
    	slogf(_SLOGC_NETWORK, _SLOG_INFO, "IEEE80211_IOC_OPTIE i_value = %d, i_len = %d\n", ireq->i_val, ireq->i_len);
    	error = copyin((char *)ireq + sizeof(*ireq), req_ie, ireq->i_len);
    	//req_ie = ireq->i_data;
        if (ireq->i_len > 0)
        {
        	slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] IEEE80211_IOC_OPTIE, req_ie[8] = 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n", __func__,req_ie[0], req_ie[1], req_ie[2], req_ie[3], req_ie[8], req_ie[9], req_ie[10], req_ie[11],req_ie[12], req_ie[13]);

			req_ie_len = req_ie[1] + 2;
			slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] IEEE80211_IOC_OPTIE, req_ie_len = %d", __func__, req_ie_len);

			//memcpy(&crypto.cipher_group, &req_ie[4], 4);
			crypto.cipher_group = req_ie[4]<<24 | req_ie[5]<<16 | req_ie[6]<<8 | req_ie[7];
			crypto.n_ciphers_pairwise = req_ie[8];
			//memcpy(&crypto.ciphers_pairwise[0], &req_ie[10], 4);
			for (i = 0; i < crypto.n_ciphers_pairwise; i++)
				crypto.ciphers_pairwise[i] = req_ie[i*4 + 10]<<24 | req_ie[i*4 + 11]<<16 | req_ie[i*4 + 12]<<8 | req_ie[i*4 + 13];

			crypto.n_akm_suites = req_ie[10 + 4*crypto.n_ciphers_pairwise];
			//memcpy(&crypto.akm_suites[0], &req_ie[10 + 4*crypto.n_ciphers_pairwise + 2], 4);
			for (i = 0; i < crypto.n_akm_suites; i++)
				crypto.akm_suites[0] = req_ie[(10 + 4*crypto.n_ciphers_pairwise + 2) + i*4]<<24 | req_ie[(10 + 4*crypto.n_ciphers_pairwise + 2) + i*4 + 1]<<16 | req_ie[(10 + 4*crypto.n_ciphers_pairwise + 2) + i*4 + 2]<<8 | req_ie[(10 + 4*crypto.n_ciphers_pairwise + 2) + i*4 +3];

        }
        else
        {
        	//req_ie = NULL;
			req_ie_len = 0;
			crypto.cipher_group =0;
			crypto.n_ciphers_pairwise = 0;
			crypto.n_akm_suites = 0;

        }
        //TODO:
        break;
    case IEEE80211_IOC_COUNTERMEASURES:
        slogf(_SLOGC_NETWORK, _SLOG_INFO, "IEEE80211_IOC_COUNTERMEASURES set: %d\n", ireq->i_val);
        if (ireq->i_val) {
            if ((ic->ic_flags & IEEE80211_F_WPA) == 0)
                return EINVAL;
            ic->ic_flags |= IEEE80211_F_COUNTERM;
        } else
            ic->ic_flags &= ~IEEE80211_F_COUNTERM;
        break;
    case IEEE80211_IOC_WPA:
        slogf(_SLOGC_NETWORK, _SLOG_INFO, "IEEE80211_IOC_WPA set: %d\n", ireq->i_val);
        if (ireq->i_val > 3)
            return EINVAL;
        /* XXX verify ciphers available */
        ic->ic_flags &= ~IEEE80211_F_WPA;
        switch (ireq->i_val) {
        case 1:
            ic->ic_flags |= IEEE80211_F_WPA1;
            wfi_drv->conn_info.security = WILC_FW_SEC_WPA_AES;
            //TODO
            break;
        case 2:
            ic->ic_flags |= IEEE80211_F_WPA2;
            wfi_drv->conn_info.security = WILC_FW_SEC_WPA2_AES;
            //TODO
            break;
        case 3:
            ic->ic_flags |= IEEE80211_F_WPA1 | IEEE80211_F_WPA2;
            wfi_drv->conn_info.security = WILC_FW_SEC_WPA2_AES;
            //TODO: prefer WPA
            break;
        }

		//TODO: enable wpa
        break;
#define IEEE80211_IOC_WAPI_ENABLED  99
    case IEEE80211_IOC_WAPI_ENABLED:
		//TODO
        break;

    case IEEE80211_IOC_SCAN_REQ:
        ic->ic_state = IEEE80211_S_SCAN;
        fprintf(stderr, "[%s] IEEE80211_IOC_SCAN_REQ scan\n", __func__);
		slogf(_SLOGC_NETWORK, _SLOG_INFO, "[%s] scan\n", __func__);

		if (vif->priv.cfg_scanning == false)
		{
			scan_finish = 0;
			scan_num = 0;
			scan_idx = 0;

			// Initilize scan_results variable;
			for (cnt = 0; cnt < MAX_SCAN_AP; cnt++)
			{
				if(scan_result[cnt].mgmt)
					kfree(scan_result[cnt].mgmt);

				memset(&scan_result[cnt], 0, sizeof(struct scan_results));
			}

			//do scan


			struct cfg80211_scan_request local_scan_req;
			struct cfg80211_scan_request *scan_req =  &local_scan_req;

			scan_req->ie_len = 0;
			scan_req->ie = NULL;

			scan_req->n_ssids = 1;
			scan_req->n_channels = 11;
			scan_req->channels[0].ic_freq = 2412;
			scan_req->channels[1].ic_freq = 2417;
			scan_req->channels[2].ic_freq = 2422;
			scan_req->channels[3].ic_freq = 2427;
			scan_req->channels[4].ic_freq = 2432;
			scan_req->channels[5].ic_freq = 2437;
			scan_req->channels[6].ic_freq = 2442;
			scan_req->channels[7].ic_freq = 2447;
			scan_req->channels[8].ic_freq = 2452;
			scan_req->channels[9].ic_freq = 2457;
			scan_req->channels[10].ic_freq = 2462;

			scan_req->ssids = (struct cfg80211_ssid *) create_ptr(sizeof(struct cfg80211_ssid));
			//memcpy(scan_req->ssids->ssid, "mchp_demo", 9);
			memset(scan_req->ssids->ssid, 0, sizeof(scan_req->ssids->ssid));
			scan_req->ssids->ssid_len = 0;

			// passive scan test
			//scan_req->n_ssids = 0;
			//scan_req->duration = 2000;
			//scan_req->ssids = NULL;


			scan(vif, scan_req);

		}
		else
		{
			slogf(_SLOGC_NETWORK, _SLOG_ERROR,"Scanning is in operation, skip..\n");
		}

		break;

#if(_NTO_VERSION >= 700)
    // Extension added on 6.5.0 only
    // https://jira.bbqnx.net/browse/COREOS-109658
    // http://reviews.ott.qnx.com/r/143017/
    // http://svn.ott.qnx.com/product/branches/6.5.0/trunk/lib/socket/public/net8021/ieee80211_ioctl.h
#else
    case IEEE80211_IOC_CANCEL_SCAN:
        slogf(_SLOGC_NETWORK, _SLOG_INFO, "QNX extention: cancel_scan \n");
        //TODO: cancel scan
#endif
        break;

    default:
        slogf(_SLOGC_NETWORK, _SLOG_INFO, "not supported yet\n");
        error = EINVAL;
        break;
    }
    if (error == ENETRESET && !IS_UP_AUTO(ic))
        error = 0;

    return error;
}

static int wilc_get_scan(struct ieee80211req *ireq);

static int
wilc_ioctl_get80211(struct wilc_dev* sc, struct ieee80211com *ic, u_long cmd,
    struct ieee80211req *ireq)
{
    int error = 0;
    int i; //counter

    slogf(_SLOGC_NETWORK, _SLOG_INFO, "SIOCG80211: i_type = %d\n", ireq->i_type);
    switch (ireq->i_type) {

    case IEEE80211_IOC_AUTHMODE:
        if (ic->ic_flags & IEEE80211_F_WPA)
            ireq->i_val = IEEE80211_AUTH_WPA;
        else
            ireq->i_val = ic->ic_bss->ni_authmode;
        break;

    case IEEE80211_IOC_WPA:
        switch (ic->ic_flags & IEEE80211_F_WPA) {
        case IEEE80211_F_WPA1:
            ireq->i_val = 1;
            break;
        case IEEE80211_F_WPA2:
            ireq->i_val = 2;
            break;
        case IEEE80211_F_WPA1 | IEEE80211_F_WPA2:
            ireq->i_val = 3;
            break;
        default:
            ireq->i_val = 0;
            break;
        }
        slogf(_SLOGC_NETWORK, _SLOG_INFO, "IEEE80211_IOC_WPA, get %d\n", ireq->i_val);
        break;
    case IEEE80211_IOC_ROAMING:
        slogf(_SLOGC_NETWORK, _SLOG_INFO, "IEEE80211_IOC_ROAMING, get %d\n", ic->ic_roaming);
        ireq->i_val = ic->ic_roaming;
        break;
    case IEEE80211_IOC_PRIVACY:
        ireq->i_val = (ic->ic_flags & IEEE80211_F_PRIVACY) != 0;
        slogf(_SLOGC_NETWORK, _SLOG_INFO, "IEEE80211_IOC_PRIVACY, get %d\n", ireq->i_val);
        break;
    case IEEE80211_IOC_DROPUNENCRYPTED:
        ireq->i_val = (ic->ic_flags & IEEE80211_F_DROPUNENC) != 0;
        break;
    case IEEE80211_IOC_COUNTERMEASURES:
        ireq->i_val = (ic->ic_flags & IEEE80211_F_COUNTERM) != 0;
        break;
    case IEEE80211_IOC_DRIVER_CAPS:
        ireq->i_val = ic->ic_caps>>16;
        ireq->i_len = ic->ic_caps&0xffff;
        break;

    case IEEE80211_IOC_OPTIE:

        if (ic->ic_opt_ie == NULL)
            return EINVAL;
        /* NB: truncate, caller can check length */
        if (ireq->i_len > ic->ic_opt_ie_len)
            ireq->i_len = ic->ic_opt_ie_len;

        error = copyout(ic->ic_opt_ie, (char *)ireq + sizeof(*ireq), ireq->i_len);
        break;

    case IEEE80211_IOC_SCAN_RESULTS:
    	slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] IEEE80211_IOC_SCAN_RESULTS, In", __func__);
    	fprintf(stderr,"[%s] IEEE80211_IOC_SCAN_RESULTS, In", __func__);
    	error = wilc_get_scan(ireq);
    	slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] IEEE80211_IOC_SCAN_RESULTS, Out", __func__);
        break;

    default:
        slogf(_SLOGC_NETWORK, _SLOG_INFO, "not supported yet\n");
        error = EINVAL;
        break;
    }

    return error;
}

int wilc_get_scan(struct ieee80211req *ireq)
{
    static uint8_t result_buffer[1024]; //FIXME: is the size enough?
    struct ieee80211req_scan_result *result = (struct ieee80211req_scan_result *)result_buffer;
    int space;
    uint8_t *p;
    uint8_t *cp;
    int i,j;
    int cnt;

    p = (unsigned char *)(ireq + 1);
    space = ireq->i_len;

    if (0) { // TODO: if result is not ready, hold on
        return -EAGAIN;
    }


    const u8 *erp_elm, *rates_elm, *rsn_elm, *vendor_spec_elm;
	u8 *ies;
	int ies_len;
	size_t offset;


	slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] Number of Scanned AP = %d", __func__, scan_num);
    for (i=0; i< scan_num; i++)
    {
    	/*
    	slogf(_SLOGC_NETWORK, _SLOG_INFO,"\r\n");
    	slogf(_SLOGC_NETWORK, _SLOG_INFO,"\r\n");
    	slogf(_SLOGC_NETWORK, _SLOG_INFO,"\r\n");

        slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] SIOCG80211NWID, scan_result[%d].ssid = %s", __func__, i, scan_result[i].ssid);
        slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] SIOCG80211NWID, scan_result[%d].bssid = %x %x %x %x %x %x ", __func__, i, scan_result[i].bssid[0], scan_result[i].bssid[1], scan_result[i].bssid[2], scan_result[i].bssid[3], scan_result[i].bssid[4], scan_result[i].bssid[5]);
        slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] SIOCG80211NWID, scan_result[%d].chan = %d", __func__, i, scan_result[i].channel);
        slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] SIOCG80211NWID, scan_result[%d].rssi = %d", __func__, i, scan_result[i].rssi);
        slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] SIOCG80211NWID, scan_result[%d].frame_len = %d", __func__, i, scan_result[i].frame_len);
        slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] SIOCG80211NWID, scan_result[%d].mgmt->frame_control = 0x%x", __func__, i, scan_result[i].mgmt->frame_control);
		*/
        if (ieee80211_is_probe_resp(scan_result[i].mgmt->frame_control))
        {
			offset = offsetof(struct ieee80211_mgmt, u.probe_resp.variable);
			//slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s]probe_resp, offset = %d", __func__,offset);
        }
		else if (ieee80211_is_beacon(scan_result[i].mgmt->frame_control))
		{
			offset = offsetof(struct ieee80211_mgmt, u.beacon.variable);
			//slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s]is_beacon, offset = %d", __func__, offset);
		}
		ies = scan_result[i].mgmt->u.beacon.variable;
		ies_len = scan_result[i].frame_len - offset;
		slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] ies_len = %d", __func__, ies_len);

		erp_elm = cfg80211_find_ie(WLAN_EID_ERP_INFO, ies, ies_len);
		rates_elm = cfg80211_find_ie(WLAN_EID_SUPP_RATES, ies, ies_len);

		rsn_elm = cfg80211_find_ie(WLAN_EID_RSN, ies, ies_len);
		if (rsn_elm != NULL)
		{
			slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] SIOCG80211NWID, rsn_elm len = %d", __func__, rsn_elm[1]);

			//for (cnt = 0; cnt < rsn_elm[1]; cnt++)
			//{
			//	slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] SIOCG80211NWID, rsn_elm [%d] 0x%x", __func__, cnt, rsn_elm[cnt+2]);
			//}

		}
		vendor_spec_elm = cfg80211_find_ie(WLAN_EID_VENDOR_SPECIFIC, ies, ies_len);

        memset(result, 0, sizeof(*result));
        //TODO: fill these fields
        result->isr_freq = 2407 + scan_result[i].channel * 5; // convert channel to freq
        result->isr_flags = 0x0490; // bss band? To Do:
        result->isr_rssi = scan_result[i].rssi;
        result->isr_intval = (uint8_t) scan_result[i].mgmt->u.beacon.beacon_int & 0xFF; // beacon period?
        result->isr_capinfo = (uint8_t) scan_result[i].mgmt->u.beacon.capab_info & 0xFF; //cap_info

        if (erp_elm != NULL)
        	result->isr_erp = erp_elm[2]; // erp flags
        else
        	result->isr_erp = 0;

        memcpy(result->isr_bssid, scan_result[i].bssid, IEEE80211_ADDR_LEN);

        if (rates_elm != NULL)
        {
        	result->isr_nrates = rates_elm[1]; // number of supported rates
        	for (j = 0; j < result->isr_nrates; j++)
        		result->isr_rates[j] = rates_elm[2+j];
        }
        else
        	result->isr_nrates = 0;

        //memcpy(result->isr_rates, <TODO: rates>, result->isr_nrates); // list of supported rates
        result->isr_ssid_len = strlen((const char *)scan_result[i].ssid); // SSID length
        slogf(_SLOGC_NETWORK, _SLOG_INFO, "result->isr_ssid_len = %d", result->isr_ssid_len);
        /*
        slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] SIOCG80211NWID, result[%d]->isr_ssid_len = %d", __func__, i, result->isr_ssid_len);
        slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] SIOCG80211NWID, result[%d]->isr_intval = %d", __func__, i, result->isr_intval);
        slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] SIOCG80211NWID, result[%d]->isr_capinfo = %d", __func__, i, result->isr_capinfo);
        slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] SIOCG80211NWID, result[%d]->isr_erp = %d", __func__, i, result->isr_erp);
        slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] SIOCG80211NWID, result[%d]->isr_nrates = %d", __func__, i, result->isr_nrates);
        slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] SIOCG80211NWID, result[%d]->isr_ssid_len = %d", __func__, i, result->isr_ssid_len);
		*/
        cp = (uint8_t *)(result+1);

        memcpy(cp, scan_result[i].ssid, result->isr_ssid_len);
        cp  += result->isr_ssid_len;
        result->isr_ie_len = result->isr_ssid_len; // SSID first
        slogf(_SLOGC_NETWORK, _SLOG_INFO, "1 result->isr_ie_len = %d", result->isr_ie_len);
        //memcpy(cp, ies, ies_len);
        //result->isr_ie_len += ies_len;
        //if (ies_len > 255)
        {
        	fprintf(stderr, "scan: ssid = %s, ssid_len = %d, rssi = %d\n", scan_result[i].ssid, result->isr_ssid_len, (int8_t) result->isr_rssi);
			if (rsn_elm != NULL)
			{
				fprintf(stderr, "with rsn: ie_len = %d\n", rsn_elm[1] + 2);
				memcpy(cp, rsn_elm, rsn_elm[1] + 2);
				result->isr_ie_len += (rsn_elm[1] + 2);
				cp += (rsn_elm[1] + 2);
			}
			if (vendor_spec_elm != NULL)
			{
				fprintf(stderr, "with vendor spec: ie_len = %d\n", vendor_spec_elm[1] + 2);
				memcpy(cp, vendor_spec_elm, vendor_spec_elm[1] + 2);
				result->isr_ie_len += (vendor_spec_elm[1] + 2);
			}
        }
        //else
        //{
        //	fprintf(stderr, "ies_len < 255: ssid = %s, ie_len = %d, ssid_len = %d\n", scan_result[i].ssid, ies_len, result->isr_ssid_len);
        //	memcpy(cp, ies, ies_len);
        //	result->isr_ie_len += ies_len;
        //}
        slogf(_SLOGC_NETWORK, _SLOG_INFO, "2 ies_len = %d", ies_len);
        slogf(_SLOGC_NETWORK, _SLOG_INFO, "2 result->isr_ie_len = %d", result->isr_ie_len);
        /*
        cp  -= result->isr_ssid_len;
        if (memcmp(scan_result[i].ssid, "MASTERS", 7) == 0)
        {
        	slogf(_SLOGC_NETWORK, _SLOG_INFO, "3 result->isr_ie_len = %d", result->isr_ie_len);
			for (j = 0; j < result->isr_ie_len; j++)
			{
				slogf(_SLOGC_NETWORK, _SLOG_INFO, "cp[%d] = 0x%x", j, cp[j]);
			}
        }
        */
#if 0
        // TOOD: calculate IE length
        // for (ie_idx = 0; ie_idx < ie_cnt; ie_idx++) {
        //     memcpy(cp, ie[ie_idx].data, ie[ie_idx].len); // store IE structure
        //     cp += ie[ie_idx].len;
        //     result->isr_ie_len += ie_len;
        // }
#endif
        result->isr_len = sizeof(*result) + result->isr_ie_len;
        result->isr_len = (result->isr_len + 3) & ~3; // padding

        if (space < result->isr_len) {
            slogf(_SLOGC_NETWORK, _SLOG_INFO, "[%s] no more space, skip", __func__);
            break;
        }

        copyout(result, p, result->isr_len); // concatenate results

        p += result->isr_len;
        space -= result->isr_len;

    }

    ireq->i_len -= space;
    // done
    return 0;
}

int wilc_ioctl(struct ifnet *ifp, unsigned long cmd, caddr_t data)
{
	int		error;
	struct wilc_dev *sc = ifp->if_softc;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ifreq *ifr = (struct ifreq *)data;
	//struct ieee80211req *req;
	struct ieee80211chanreq *chanreq;
	int i; //counter
	struct ieee80211_nwid nwid;

	struct wilc_priv *priv = &vif->priv;
	struct host_if_drv *wfi_drv = priv->hif_drv;

	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] cmd=0x%0x!!!", __func__, (unsigned int) cmd);
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] ifp= %p!!!", __func__, ifp);
	error = 0;

	assert(ic);
	assert(sc);
#if 0
	//TODO:
	if (sc->state != STATE_INIT) {
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] Not in INIT, cmd=0x%0x", __func__, (unsigned int) cmd);
		error = ether_ioctl(ifp, cmd, data);
		if (error == ENETRESET) {
			/*
			 * Multicast list has changed; set the
			 * hardware filter accordingly.
			 */
			slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] mc list changed, flag=0x%0x", __func__, ifp->if_flags);
			if (ifp->if_flags & IFF_RUNNING) {
				/*
				 * Multicast list has changed; set the
				 * hardware filter accordingly.
				 */
				//TODO:
			}

			error = 0;
		}
		return error;
	}
#endif

	switch (cmd) {
	case SIOCSIFMEDIA:
	case SIOCGIFMEDIA:
		error = ifmedia_ioctl(ifp, (struct ifreq *) data,
				&ic->ic_media, cmd);
		break;

	case SIOCADDMULTI:
	case SIOCDELMULTI:
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"sam_ioctl (), SIOCADDMULTI\n");
		error = (cmd == SIOCADDMULTI) ?
			ether_addmulti(ifreq_getaddr(cmd, ifr), &sc->sc_ec) :
			ether_delmulti(ifreq_getaddr(cmd, ifr), &sc->sc_ec);
		if (error == ENETRESET) {
			slogf(_SLOGC_NETWORK, _SLOG_ERROR,"sam_ioctl (), SIOCADDMULTI, ENETRESET\n");
			if (ifp->if_flags & IFF_RUNNING)
				//ath_mode_init(sc);
				error = 0;
		}

		break;

	case SIOCS80211:	// 80211 set
		error = wilc_ioctl_set80211(sc, ic, cmd, (struct ieee80211req*)data);
		break;

	case SIOCG80211:
		error = wilc_ioctl_get80211(sc, ic, cmd, (struct ieee80211req*)data);
		break;

	case SIOCS80211NWID: // set ssid
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"sam_ioctl (), SIOCS80211NWID\n");
		fprintf(stderr,"SIOCS80211NWID is received\r\n");

		//u8 bssid[] = {0x58, 0xef, 0x68, 0x24, 0xe1, 0xf3};	//linksys
		//char bssid[] = "MASTERS";
		//u8 bssid[] = {0x10, 0x7b, 0x44, 0xea, 0x1c, 0xa8};	//asus
		//char bssid[] = "mchp_demo";
		u8 bssid[] = {0x50, 0xc7, 0xbf, 0xae, 0xe1, 0x7a};	//tplink
		//u8 ies[5] = {0x3b, 0x3, 0x51, 0x51, 0x52};

		wilc_wlan_set_bssid(vif, bssid, WILC_STATION_MODE);
#if 0
		wfi_drv->conn_info.conn_result = cfg_connect_result;
		//wfi_drv->conn_info.security = security;
		//wfi_drv->conn_info.auth_type = WILC_FW_AUTH_OPEN_SYSTEM;
		//wfi_drv->conn_info.ch = ch;
		//wfi_drv->conn_info.arg = priv;
		//wfi_drv->conn_info.param = join_params;
		set_ssid_func();
		wilc_set_join_req(vif,bssid, ies, sizeof(ies));
		///error = ieee80211_ioctl(ifp, cmd, data);
#endif
		break;

	case SIOCG80211NWID:	// TODO:

		//fprintf(stderr, "ioctl: SIOCG80211NWID\r\n");
		memset(&nwid, 0, sizeof(nwid));
		//memcpy(nwid.i_nwid, "iPhone",6);

		// To Do: temperory method to get the SSID name, need to correct it.
		for (i=0; i< scan_num; i++)
		{

			if (strncmp(vif->bssid, scan_result[i].bssid,6) == 0)
			{
				//fprintf(stderr, "match!\n");
				memcpy(nwid.i_nwid, scan_result[i].ssid, strlen(scan_result[i].ssid));
				break;
			}
		}



		nwid.i_len = strlen(nwid.i_nwid);
		error = copyout(&nwid, data + sizeof(*ifr), sizeof(nwid));

		//nwid = (struct ieee80211_nwid *)data;
		//memcpy(nwid.i_nwid, "mchp_demo", 9);
		//nwid->i_len = 9;

		break;

	case SIOCS80211NWKEY:	// set password
		slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] SIOCS80211NWKEY", __func__);
		break;

	case SIOCG80211NWKEY:
		slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] SIOCG80211NWKEY", __func__);
		break;

	case SIOCS80211CHANNEL:
		slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] SIOCS80211CHANNEL, not supported yet", __func__);
		//chanreq = (struct ieee80211chanreq *)data;
		break;

	case SIOCG80211CHANNEL:
		//not pass this event to ieee80211_ioctl to avoid failure
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"sam_ioctl (), SIOCG80211CHANNEL\n");
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"ic_myaddr = 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n", ic->ic_myaddr[0], ic->ic_myaddr[1], ic->ic_myaddr[2], ic->ic_myaddr[3], ic->ic_myaddr[4], ic->ic_myaddr[5]);
		chanreq = (struct ieee80211chanreq *)data;
		if (ic == NULL || ic->ic_des_chan == NULL)
			slogf(_SLOGC_NETWORK, _SLOG_ERROR,"ic == NULL\n");
		else
		{
			if (ic->ic_des_chan == NULL)
				slogf(_SLOGC_NETWORK, _SLOG_ERROR," ic_des_chan == NULL\n");
			else if (ic->ic_ibss_chan == NULL)
				slogf(_SLOGC_NETWORK, _SLOG_ERROR," ic_ibss_chan == NULL\n");
			else if (ic->ic_curchan == NULL)
				slogf(_SLOGC_NETWORK, _SLOG_ERROR," ic_curchan == NULL\n");
			else
			{
				//slogf(_SLOGC_NETWORK, _SLOG_INFO,"chanreq=%d, state=%s, ic_opmode=%d, freq=%d, freq=%d freq=%d\n", chanreq->i_channel, ieee80211_state_name[ic->ic_state], ic->ic_opmode, ic->ic_des_chan->ic_freq, ic->ic_ibss_chan->ic_freq, ic->ic_curchan->ic_freq);
				chanreq->i_channel = ieee80211_chan2ieee(ic, ic->ic_des_chan);		// Convert channel to IEEE channel number.
			}
		}
		//error = ieee80211_ioctl(ifp, cmd, data);

		slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] SIOCG80211CHANNEL finish\n", __func__);
		break;

	case SIOCS80211BSSID:
		error = ieee80211_ioctl(ic, cmd, data);
		slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] SIOCS80211BSSID", __func__);
		if (error == ENETRESET) {
			//TODO
			if (!(ic->ic_flags & IEEE80211_F_DESBSSID)) {
			} else {
				/* zero_mac means bss_stop */
			}
			if (error < 0) error = -error;
		}
		error = 0;
		break;

	case SIOCG80211BSSID:
		slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] SIOCG80211BSSID", __func__);
		struct ieee80211_bssid *cur_bssid;
		cur_bssid = (struct ieee80211_bssid *)data;
		if (!error)
			IEEE80211_ADDR_COPY(cur_bssid->i_bssid, vif->bssid);
			//IEEE80211_ADDR_COPY(bssid->i_bssid,
			//		ic->ic_bss->ni_bssid);

		break;

	default:
		slogf(_SLOGC_NETWORK, _SLOG_INFO,"default setting\n");
		error = ether_ioctl(ifp, cmd, data);
		if (error == ENETRESET) {
			/*
			 * Multicast list has changed; set the
			 * hardware filter accordingly.
			 */
			if ((ifp->if_flags & IFF_RUNNING) == 0) {
				/*
				 * Interface is currently down: sam_init()
				 * will call sam_set_multicast() so
				 * nothing to do
				 */
			} else {
				/*
				 * interface is up, recalculate and
				 * reprogram the hardware.
				 */
				///sam_set_multicast(sam);
			}
			error = 0;
		}
		break;
	}

	return error;
}

int
wilc_detach(struct device *dev, int flags)
{
	slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] In\n", __func__);
	struct wilc_dev	*wilc;
	struct ifnet	*ifp;

	/*
	 * Clean up everything.
	 *
	 * The interface is going away but io-pkt is staying up.
	 */
	wilc = (struct wilc_dev *)dev;
#if 0
	ifp = &wilc->sc_ec.ec_if;
#else
	ifp = wilc->sc_ic.ic_ifp;
#endif
	wilc_stop(ifp, 1);
#if 0
	ether_ifdetach(ifp);
#else
	ieee80211_ifdetach(&wilc->sc_ic);
#endif

	if_detach(ifp);

	shutdownhook_disestablish(wilc->sc_sdhook);

	return EOK;
}

void
wilc_shutdown(void *arg)
{
	slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] In\n", __func__);
	struct wilc_dev	*wilc;

	/* All of io-pkt is going away.  Just quiet hardware. */

	wilc = arg;

#if 1
	wilc_stop(&wilc->sc_ec.ec_if, 1);
#else
	wilc_stop(wilc->sc_ic.ic_ifp, 1);
#endif
}

#ifndef HW_MASK
const struct sigevent * wilc_isr(void *arg, int iid)
{
	struct wilc_dev		*wilc;
	struct _iopkt_inter	*ient;

	wilc = arg;
	ient = &wilc->sc_inter;

	/*
	 * Close window where this is referenced in sam_enable_interrupt().
	 * We may get an interrupt, return a sigevent and have another
	 * thread start processing on SMP before the InterruptAttach()
	 * has returned.
	 */
	wilc->sc_iid = iid;

	InterruptMask(wilc->sc_irq, iid);

	return interrupt_queue(wilc->sc_iopkt, ient);
}
#else
const struct sigevent *
wilc_isr(void *arg, int iid)
{
	struct wilc_dev		*wilc;
	struct _iopkt_self	*iopkt;
	const struct sigevent	*evp;
	struct inter_thread	*itp;

	wilc = arg;
	iopkt = wilc->sc_iopkt;
	evp = NULL;

#ifdef READ_CAUSE_IN_ISR
	/*
	 * Trade offs.
	 * - Doing this here means another register read across the bus.
	 * - If not sharing interrupts, this boils down to exactly the
	 *   same amount of work but doing more of it in the isr.
	 * - If sharing interupts, can short circuit some work in the
	 *   stack here.
	 * - Maybe trade off is to only do it if we're detecting
	 *   spurious interrupts which should happen under heavy
	 *   shared interrupt load?
	 */
#ifdef READ_CAUSE_ONLY_ON_SPURIOUS
	if (ient->spurrious) {
#endif
		if (ient->on_list == 0 &&
		    (wilc->sc_intr_cause = i82544->reg[I82544_ICR]) == 0) {
			return NULL; /* Not ours */
		}
		wilc->sc_flag |= CAUSE_VALID;
#ifdef READ_CAUSE_ONLY_ON_SPURIOUS
	}
#endif
#endif

	/*
	 * We have to make sure the interrupt is masked regardless
	 * of our on_list status.  This is because of a window where
	 * a shared (spurious) interrupt comes after on_list
	 * is knocked down but before the enable() callout is made.
	 * If enable() then happened to run after we masked, we
	 * could end up on the list without the interrupt masked
	 * which would cause the kernel more than a little grief
	 * if one of our real interrupts then came in.
	 *
	 * This window doesn't exist when using kermask since the
	 * interrupt isn't unmasked until all the enable()s run
	 * (mask count is tracked by kernel).
	 */

	/*
	 * If this was controling real hardware, mask of
	 * interrupts here. eg from i82544 driver:
	 */
	i82544->reg[I82544_IMC] = 0xffffffff;

	return interrupt_queue(wilc->sc_iopkt, ient);
}
#endif
int
wilc_process_interrupt(void *arg, struct nw_work_thread *wtp)
{
	struct wilc_dev		*wilc;
	struct mbuf			*m;
	struct ifnet		*ifp;
	struct ether_header	*eh;

	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] In\n", __func__);
	//printf("wilc_process_interrupt log1\n");

	wilc = arg;
#if 0
	ifp = &wilc->sc_ec.ec_if;
#else
	ifp = wilc->sc_ic.ic_ifp;
#endif

	//if ((wilc->sc_intr_cnt++ % 1000) == 0) {
		/* Send a packet up */
		m = m_getcl_wtp(M_DONTWAIT, MT_DATA, M_PKTHDR, wtp);
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s]  log2\n", __func__);
		//printf("[%s]  log2\n", __func__);
		if (!m) {
            ifp->if_ierrors++;  // for ifconfig -v
			return 1;
		}

		m->m_pkthdr.len = m->m_len = sizeof(*eh);

		// ip_input() needs this
		m->m_pkthdr.rcvif = ifp;
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s]  log3\n", __func__);
		//printf("wilc_process_interrupt log3\n");
		// dummy up a broadcasted IP packet for testing
		eh = mtod(m, struct ether_header *);
		eh->ether_type = ntohs(ETHERTYPE_IP);
		memcpy(eh->ether_dhost, etherbroadcastaddr, ETHER_ADDR_LEN);

		ifp->if_ipackets++; // for ifconfig -v
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s]  log4\n", __func__);
		(*ifp->if_input)(ifp, m);

		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s]  log5\n", __func__);
		//printf("wilc_process_interrupt %d\n", wilc->sc_intr_cnt);
	//}

	/*
	 * return of 1 means were done.
	 *
	 * If we notice we're taking a long time (eg. processed
	 * half our rx descriptors) we could early out with a
	 * return of 0 which lets other interrupts be processed
	 * without calling our interrupt_enable func.  This
	 * func will be called again later.
	 */
	return 1;
}
#ifndef HW_MASK
int wilc_enable_interrupt(void *arg)
{
	slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] In\n", __func__);
	struct wilc_dev	*wilc;

	wilc = arg;
	InterruptUnmask(wilc->sc_irq, wilc->sc_iid);

	return 1;
}
#else
int
wilc_enable_interrupt(void *arg)
{
	struct wilc_dev	*wilc;

	wilc = arg;
	/* eg from i82544 driver */

	i82544->reg[I82544_IMS] = i82544->intrmask;

	return 1;
}
#endif



#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL$ $Rev$")
#endif
