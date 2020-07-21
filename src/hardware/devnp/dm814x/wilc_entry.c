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
#include    <sys/sockio.h>
#if 1
#include <net80211/ieee80211_var.h>
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

CFATTACH_DECL(wilc,
	sizeof(struct wilc_dev),
	NULL,
	wilc_attach,
	wilc_detach,
	NULL);

int wilc_reset(struct ifnet *ifp)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[ENTRY_DEBUG] wilc_reset()\n");
	//return 0;
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
	//return 0;
}
static void wilc_watchdog(struct ifnet *ifp)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[ENTRY_DEBUG] wilc_watch()\n");
}

struct ieee80211_node * wilc_node_alloc(struct ieee80211com *ic)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[ENTRY_DEBUG] wilc_node_alloc()\n");
	//return ic;
}

int wilc_media_change(struct ifnet *ifp)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[ENTRY_DEBUG] wilc_media_change()\n");
	//return 0;
}

int wilc_newstate(struct ieee80211com *ic, enum ieee80211_state nstate, int arg)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[ENTRY_DEBUG] wilc_newstate()\n");
	//slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[ENTRY_DEBUG] wilc_newstate(), nstate=%d, arg=%d\n", nstate, arg);
	//slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[ENTRY_DEBUG] wilc_newstate() %s -> %s\n", ieee80211_state_name[ic->ic_state],ieee80211_state_name[nstate]);
	return 0;
}

u_int8_t wilc_node_getrssi(struct ieee80211com *ic, const struct ieee80211_node *ni)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[ENTRY_DEBUG] wilc_node_getrssi()\n");
	//return 0;
}

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
	//return 0;
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
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"Packet sentSize= %d Add= %p \n", pv_data->size, pv_data->buff);
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


struct channel chan[] = {{2412, IEEE80211_CHAN_2GHZ},{2417, IEEE80211_CHAN_2GHZ}, {2422, IEEE80211_CHAN_2GHZ}, {2427, IEEE80211_CHAN_2GHZ}, {2432, IEEE80211_CHAN_2GHZ}, \
						{2437, IEEE80211_CHAN_2GHZ}, {2442, IEEE80211_CHAN_2GHZ}, {2447, IEEE80211_CHAN_2GHZ}, {2452, IEEE80211_CHAN_2GHZ}, {2457, IEEE80211_CHAN_2GHZ}, \
						{2462, IEEE80211_CHAN_2GHZ}, {2467, IEEE80211_CHAN_2GHZ}, {2472, IEEE80211_CHAN_2GHZ},{2484, IEEE80211_CHAN_2GHZ}};
#define NUM_CHANNEL_SUPPORTED	14

int wilc_getchannels(struct wilc_dev *sc)
{
	struct ieee80211com *ic = &sc->sc_ic;
	struct ifnet *ifp = &ic->ic_ifp;
	int i, ix;


	/*
	 * Convert HAL channels to ieee80211 ones and insert
	 * them in the table according to their channel number.
	 */
	for (i = 0; i < NUM_CHANNEL_SUPPORTED; i++) {
		ix = ieee80211_mhz2ieee(chan[i].freq, chan[i].flags);
		if (ix > IEEE80211_CHAN_MAX) {
			slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] bad hal channel %u (%u/%x) ignored\n\n", __func__, ifp->if_xname, ix, chan[i].freq, chan[i].flags);
			continue;
		}
		slogf(_SLOGC_NETWORK, _SLOG_INFO, "[%s] ix=%d freq= %d \n", __func__, ix,  chan[i].freq);

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
	int i, maxrates;

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

int wilc_attach(struct device *parent, struct device *self, void *aux)
{

	int	err;
	struct wilc_dev	*wilc;
	struct ifnet	*ifp;
	//uint8_t	enaddr[ETHER_ADDR_LEN];
	struct qtime_entry	*qtp;
	struct ieee80211com *ic;
	int ret;

	slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] In\n", __func__);

	/* initialization and attach */

	wilc = (struct wilc_dev *)self;
	ic = &wilc->sc_ic;
#if 0
	ifp = &wilc->sc_ec.ec_if;
#else

	linux_sdio_probe(wilc);

	vif = wilc_get_wl_to_vif(wilc);
	//slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] DEBUG: vif ptr =%p\n", __func__, vif);

	ret = wilc_init_host_int(vif);
	if (ret < 0) {
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"Failed to initialize host interface\n");
		return ret;
	}

	//wilc->sc_ic.ic_ifp = (malloc)(sizeof(ifnet));
	//ifp = wilc->sc_ic.ic_ifp;

	ifp = &wilc->sc_ec.ec_if;
	ic->ic_ifp = ifp;

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


	ic->ic_reset = wilc_reset;
	ic->ic_newassoc = wilc_newassoc;
	ic->ic_updateslot = wilc_updateslot;
	ic->ic_wme.wme_update = wilc_wme_update;
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


	if_attach(ifp);

#if 0
	/* Normal ethernet */
	ether_ifattach(ifp, enaddr);
#else
	/* 80211 */
	//memcpy(ic->ic_myaddr, enaddr, ETHER_ADDR_LEN);
	ieee80211_ifattach(&wilc->sc_ic);

#endif

	/* override default methods */
	ic->ic_node_alloc = wilc_node_alloc;
	ic->ic_node_free = wilc_node_free;
	ic->ic_node_getrssi = wilc_node_getrssi;
	ic->ic_newstate = wilc_newstate;
	ic->ic_recv_mgmt = wilc_recv_mgmt;
	ic->ic_send_mgmt = wilc_send_mgmt;

	/* complete initialization */
	//ieee80211_media_init(ic, wilc_media_change, ieee80211_media_status);


	wilc->sc_sdhook = shutdownhook_establish(wilc_shutdown, wilc);

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
	unsigned char mac_add[ETH_ALEN] = {0};
	struct ieee80211com *ic;


	wilc = ifp->if_softc;
	ic = &wilc->sc_ic;
	///linux_sdio_probe(wilc);

	wilc->mac_status = WILC_MAC_STATUS_INIT;

	wilc_mac_open(vif, mac_add);

	memcpy(ic->ic_myaddr, mac_add, ETHER_ADDR_LEN);
	ieee80211_ifattach(ic);
	/*
	 * - enable hardware.
	 *   - look at ifp->if_capenable_[rx/tx]
	 *   - enable promiscuous / multicast filter.
	 * - attach to interrupt.
	 *
	 *
	 */

// scan test
#if 0
	static int test_cnt =0;

	if (test_cnt == 0)
	{
	struct cfg80211_scan_request*scan_req = create_ptr(sizeof(struct cfg80211_scan_request));
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"log1\n");
	struct ieee80211_channel * channel = create_ptr(sizeof(struct ieee80211_channel));
	scan_req->channels[0] = channel;

	scan_req->n_ssids = 1;
	scan_req->n_channels = 11;
	scan_req->channels[0]->ic_freq = 2412;

	scan_req->channels[1] = (struct ieee80211_channel *) create_ptr(sizeof(struct ieee80211_channel));
	scan_req->channels[1]->ic_freq = 2417;
	scan_req->channels[2] = (struct ieee80211_channel *) create_ptr(sizeof(struct ieee80211_channel));
	scan_req->channels[2]->ic_freq = 2422;
	scan_req->channels[3] = (struct ieee80211_channel *) create_ptr(sizeof(struct ieee80211_channel));
	scan_req->channels[3]->ic_freq = 2427;
	scan_req->channels[4] = (struct ieee80211_channel *) create_ptr(sizeof(struct ieee80211_channel));
	scan_req->channels[4]->ic_freq = 2432;
	scan_req->channels[5] = (struct ieee80211_channel *) create_ptr(sizeof(struct ieee80211_channel));
	scan_req->channels[5]->ic_freq = 2437;
	scan_req->channels[6] = (struct ieee80211_channel *) create_ptr(sizeof(struct ieee80211_channel));
	scan_req->channels[6]->ic_freq = 2442;
	scan_req->channels[7] = (struct ieee80211_channel *) create_ptr(sizeof(struct ieee80211_channel));
	scan_req->channels[7]->ic_freq = 2447;
	scan_req->channels[8] = (struct ieee80211_channel *) create_ptr(sizeof(struct ieee80211_channel));
	scan_req->channels[8]->ic_freq = 2452;
	scan_req->channels[9] = (struct ieee80211_channel *) create_ptr(sizeof(struct ieee80211_channel));
	scan_req->channels[9]->ic_freq = 2457;
	scan_req->channels[10] = (struct ieee80211_channel *) create_ptr(sizeof(struct ieee80211_channel));
	scan_req->channels[10]->ic_freq = 2462;

	scan_req->ssids = (struct cfg80211_ssid *) create_ptr(sizeof(struct cfg80211_ssid));
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"log2\n");
	memcpy(scan_req->ssids->ssid, "mchp_demo", 9);
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"log3\n");
	scan_req->ssids->ssid_len = 9;
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"log4\n");
	scan(vif, scan_req);

	test_cnt++;
	}

#endif

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

	char test_buf[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0xf0, 0x05, 0xe5, 0x47, 0x91, 0x08, 0x06, 0x00, 0x01,
			0x08, 0x00, 0x06, 0x04, 0x00, 0x01, 0xf8, 0xf0, 0x05, 0xe5, 0x47, 0x91, 0x00, 0x00, 0x00, 0x00,
			00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xa8, 0x02, 0x4f};


	wilc = ifp->if_softc;
	wtp = WTP;

	for (;;) {
		IFQ_POLL(&ifp->if_snd, m);
		if (m == NULL)
		{
			printf("m=NULL\r\n");
			break;
		}
		/*
		 * Can look at m to see if you have the resources
		 * to transmit it.
		 */

		IFQ_DEQUEUE(&ifp->if_snd, m);


		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"buf1 len=%d, data=0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\r\n", m->m_hdr.mh_len, m->m_hdr.mh_data[0],m->m_hdr.mh_data[1], m->m_hdr.mh_data[2], m->m_hdr.mh_data[3], m->m_hdr.mh_data[4],m->m_hdr.mh_data[5], m->m_hdr.mh_data[6], m->m_hdr.mh_data[7],  m->m_hdr.mh_data[8],m->m_hdr.mh_data[9], m->m_hdr.mh_data[10], m->m_hdr.mh_data[11], m->m_hdr.mh_data[12],m->m_hdr.mh_data[13], m->m_hdr.mh_data[14], m->m_hdr.mh_data[15]);
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"buf2 len=%d, data=0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\r\n", m->m_pkthdr.len, m->m_dat[0], m->m_dat[1], m->m_dat[2], m->m_dat[3], m->m_dat[4], m->m_dat[5], m->m_dat[6], m->m_dat[7], m->m_pkthdr.len, m->m_dat[8], m->m_dat[9], m->m_dat[10], m->m_dat[11], m->m_dat[12], m->m_dat[13], m->m_dat[14], m->m_dat[15]);
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
		tx_data->buff = m->m_hdr.mh_data;
		tx_data->size = m->m_hdr.mh_len;
		//tx_data->skb  = skb;

		//Test: send ARP packet
		///tx_data->buff = test_buf;
		///tx_data->size = sizeof(test_buf);

		char* text = tx_data->buff;
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"buf1 data=0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\r\n",  text[0], text[1], text[2], text[3], text[4], text[5], text[6], text[7], text[8], text[9], text[10], text[11], text[12], text[13], text[14], text[15]);
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] Sending pkt Size= %d Add= %p \n", __func__, tx_data->size, tx_data->buff);
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] Adding tx pkt to TX Queue\n", __func__);

		vif->netstats.tx_packets++;
		vif->netstats.tx_bytes += tx_data->size;
		tx_data->vif = vif;
		queue_count = txq_add_net_pkt(vif, (void *)tx_data,
						  tx_data->buff, tx_data->size,
						  wilc_tx_complete);

		if (queue_count > FLOW_CTRL_UP_THRESHLD) {
			struct wilc_vif *vif;
			int srcu_idx;
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


	struct _iopkt_inter	*ient;
	ient = &wilc->sc_inter;
	//interrupt_queue(wilc->sc_iopkt, ient);
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] interrupt_queue\n", __func__);
}

void set_ssid_func()
{
	struct wilc_priv *priv = &vif->priv;
	struct host_if_drv *wfi_drv = priv->hif_drv;
	struct wilc_conn_info *conn_info = &wfi_drv->conn_info;

	conn_info->security = 0;
	conn_info->auth_type = 1;

}

int wilc_ioctl(struct ifnet *ifp, unsigned long cmd, caddr_t data)
{
	struct wilc_dev	*wilc;
	int		error;
	struct wilc_dev *sc = ifp->if_softc;
	struct ieee80211com *ic = &sc->sc_ic;
	struct ifreq *ifr = (struct ifreq *)data;
	struct ieee80211req *req;
	struct ieee80211chanreq *chanreq;

	struct wilc_priv *priv = &vif->priv;
	struct host_if_drv *wfi_drv = priv->hif_drv;


	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] cmd=0x%0x!!!\n", __func__, (unsigned int) cmd);
	wilc = ifp->if_softc;
	error = 0;

	switch (cmd) {

	case SIOCADDMULTI:
	case SIOCDELMULTI:
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"sam_ioctl (), SIOCADDMULTI\n");
		error = (cmd == SIOCADDMULTI) ?
		ether_addmulti(ifr, &sc->sc_ec) :
		ether_delmulti(ifr, &sc->sc_ec);
		if (error == ENETRESET) {
			slogf(_SLOGC_NETWORK, _SLOG_ERROR,"sam_ioctl (), SIOCADDMULTI, ENETRESET\n");
			if (ifp->if_flags & IFF_RUNNING)
				//ath_mode_init(sc);
			error = 0;
		}

		break;

	case SIOCSIFFLAGS:
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"sam_ioctl (), SIOCSIFFLAGS\n");
		if (ifp->if_flags & IFF_UP) {
			if (ifp->if_flags & IFF_RUNNING) {
				slogf(_SLOGC_NETWORK, _SLOG_ERROR,"sam_ioctl (), IFF_RUNNING\n");
			}
			else
			{
				slogf(_SLOGC_NETWORK, _SLOG_ERROR,"sam_ioctl (), IFF_UP\n");

				wilc_drv_init(ifp);
			}
		}

		break;

	case SIOCSIFADDR:
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"sam_ioctl (), SIOCSIFADDR\n");
		ifp->if_flags |= IFF_UP;
		break;
	case SIOCS80211NWID: // set ssid
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"sam_ioctl (), SIOCS80211NWID\n");

		//u8 bssid[] = {0x58, 0xef, 0x68, 0x24, 0xe1, 0xf3};	//linksys
		//char bssid[] = "MASTERS";
		u8 bssid[] = {0x10, 0x7b, 0x44, 0xea, 0x1c, 0xa8};	//asus
		//char bssid[] = "mchp_demo";
		//u8 bssid[] = {0x50, 0xc7, 0xbf, 0xae, 0xe1, 0x7a};	//tplink
		u8 ies[5] = {0x3b, 0x3, 0x51, 0x51, 0x52};

		wilc_wlan_set_bssid(vif, bssid, WILC_STATION_MODE);

		wfi_drv->conn_info.conn_result = cfg_connect_result;
		//wfi_drv->conn_info.security = security;
		//wfi_drv->conn_info.auth_type = WILC_FW_AUTH_OPEN_SYSTEM;
		//wfi_drv->conn_info.ch = ch;
		//wfi_drv->conn_info.arg = priv;
		//wfi_drv->conn_info.param = join_params;
		set_ssid_func();
		wilc_set_join_req(vif,bssid, ies, sizeof(ies));
		///error = ieee80211_ioctl(ifp, cmd, data);

		break;

	//case SIOCS80211NWKEY:
	//	break;
	//case SIOCG80211NWKEY:
	//	break;
	//case SIOCS80211CHANNEL:
	//	break;
	case SIOCS80211:	// 80211 set
		req = (struct ieee80211req *)data;
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] SIOCS80211, req type= %d\n", __func__, req->i_type);
		if (req->i_type == IEEE80211_IOC_SCAN_REQ)
		{
			slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] scan\n", __func__);
			error = ieee80211_ioctl(ifp, cmd, data);
		}
		break;
	case SIOCS80211NWKEY:	// set password
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
				slogf(_SLOGC_NETWORK, _SLOG_INFO,"chanreq=%d, state=%s, ic_opmode=%d, freq=%d, freq=%d freq=%d\n", chanreq->i_channel, ieee80211_state_name[ic->ic_state], ic->ic_opmode, ic->ic_des_chan->ic_freq, ic->ic_ibss_chan->ic_freq, ic->ic_curchan->ic_freq);
				chanreq->i_channel = ieee80211_chan2ieee(ic, ic->ic_des_chan);		// Convert channel to IEEE channel number.
			}
		}
		//error = ieee80211_ioctl(ifp, cmd, data);

		slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] SIOCG80211CHANNEL finish\n", __func__);
		break;
	//case SIOCGIFMEDIA:
	//	break;

	case SIOCGIFGENERIC:



		break;
	case SIOCGLIFPHYADDR:
		//not pass this event to ieee80211_ioctl to avoid failure
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"sam_ioctl in (), SIOCGLIFPHYADDR\n");

		//	ieee80211_ioctl(ifp, cmd, data);
		break;



	default:
		slogf(_SLOGC_NETWORK, _SLOG_INFO,"default setting\n");
		error = ieee80211_ioctl(ifp, cmd, data);
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

#if 0
	wilc_stop(&wilc->sc_ec.ec_if, 1);
#else
	wilc_stop(wilc->sc_ic.ic_ifp, 1);
#endif
}

#ifndef HW_MASK
const struct sigevent * wilc_isr(void *arg, int iid)
{
	slogf(_SLOGC_NETWORK, _SLOG_INFO,"[%s] In\n", __func__);
	printf("wilc_isr ()\n");

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
	printf("wilc_process_interrupt log1\n");

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
		printf("[%s]  log2\n", __func__);
		if (!m) {
            ifp->if_ierrors++;  // for ifconfig -v
			return 1;
		}

		m->m_pkthdr.len = m->m_len = sizeof(*eh);

		// ip_input() needs this
		m->m_pkthdr.rcvif = ifp;
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s]  log3\n", __func__);
		printf("wilc_process_interrupt log3\n");
		// dummy up a broadcasted IP packet for testing
		eh = mtod(m, struct ether_header *);
		eh->ether_type = ntohs(ETHERTYPE_IP);
		memcpy(eh->ether_dhost, etherbroadcastaddr, ETHER_ADDR_LEN);

		ifp->if_ipackets++; // for ifconfig -v
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s]  log4\n", __func__);
		(*ifp->if_input)(ifp, m);

		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s]  log5\n", __func__);
		printf("wilc_process_interrupt %d\n", wilc->sc_intr_cnt);
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
