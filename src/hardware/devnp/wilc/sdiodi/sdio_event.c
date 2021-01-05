/*
 * $QNXLicenseC:
 * Copyright 2009, QNX Software Systems.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"). You
 * may not reproduce, modify or distribute this software except in
 * compliance with the License. You may obtain a copy of the License
 * at: http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" basis,
 * WITHOUT WARRANTIES OF ANY KIND, either express or implied.
 *
 * This file may contain contributions from others, either as
 * contributors under the License or as licensors under other terms.
 * Please review this entire file for other proprietary rights or license
 * notices, as well as the QNX Development Suite License Guide at
 * http://licensing.qnx.com/license-guide/ for other information.
 * $
 */

#include <sys/slog.h>
#include <sys/slogcodes.h>

#include "proto.h"


interruptCb g_intr_cb = NULL;
int g_report_to_drv = 0;


int sdio_intr_callback_register(interruptCb cb)
{
	g_intr_cb = cb;


	return (SDIO_SUCCESS);
}

static void sdio_interrupt(sdio_ext_t *sdio)
{
	//printf("[sdio_interrupt] In\n");
	int ret = sdio->ivalidate(sdio->hchdl,0,0);
	if (ret == SDIO_INTR_CARD)
	{
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] SDIO_INTR_CARD \n", __func__);
		if (sdio->card_intr == 1) {
			pthread_sleepon_lock();
			sdio->card_intr++;
			pthread_sleepon_signal(&sdio->card_intr);
			pthread_sleepon_unlock();
		}
	}
			///g_report_to_drv = 1;
			///	g_intr_cb();
	sdio->istatus = sdio->iprocess(sdio->hchdl, sdio->cmd);

	//slogf(99,1,"%s istatus = 0x%x", __FUNCTION__, sdio->istatus);
	if (sdio->istatus) {
		if (sdio->istatus & ~SDMMC_INT_SERVICE) {
			if (sdio->wait_cmd == 1) {
				pthread_sleepon_lock();
				sdio->wait_cmd++;
				pthread_sleepon_signal(&sdio->wait_cmd);
				pthread_sleepon_unlock();
			}
		}
		if (sdio->istatus & SDMMC_INT_SERVICE) {
			// FIXME We need to queue this event if we are not the 
			// owner of this event, otherwise we could lose interrupt
			if (sdio->wait_srv == 1) {
				pthread_sleepon_lock();
				sdio->wait_srv++;
				printf("[sdio_interrupt] issue srv\n");
				pthread_sleepon_signal(&sdio->wait_srv);
				pthread_sleepon_unlock();
			} else {
				atomic_add(&sdio->pend_srv, 1);
			}
		}
	}

}

int sdio_event_get(void *hdl, int wait)
{
	sdio_ext_t	*sdio = (sdio_ext_t *)hdl;
	uint64_t	tmo;
	int			ret = SDIO_SUCCESS;
#if 0
	if (sdio->pend_srv) {
		atomic_sub(&sdio->pend_srv, 1);
		return (SDIO_SUCCESS);
	}

	// TODO!!!
	// Review the timing to enable interrupt
	if (!wait)
		return (SDIO_FAILURE);

	//sdio->wait_srv = 1;
	sdio->card_intr = 1;
	//tmo = (uint64_t)wait * 1000 * 1000 * 1000;
	tmo = 100 * 1000 * 1000;

	/* No pending event, wait */

#endif
	sdio->card_intr = 1;

	//if (sdio->card_intr > 1)
	//{
	//	g_intr_cb();
	//}
	//else
	{
		//slogf(99,1,"[%s] In", __FUNCTION__);
		pthread_sleepon_lock();
		sdio->ienable(sdio->hchdl, SDIO_INTR_SDIO, 1);
		//slogf(99,1,"[%s] log1", __FUNCTION__);
		//fprintf(stderr,"[%s] log1", __FUNCTION__);
		if (pthread_sleepon_timedwait(&sdio->card_intr, 1 * 1000 * 1000) != EOK) {
		//if (pthread_sleepon_wait(&sdio->card_intr) != EOK) {
			sdio->ienable(sdio->hchdl, SDIO_INTR_SDIO, 0);
			pthread_sleepon_unlock();
			ret = SDIO_FAILURE;
		}
		else
		{
			sdio->ienable(sdio->hchdl, SDIO_INTR_SDIO, 0);
			pthread_sleepon_unlock();

			g_intr_cb();
		}
	}
	//slogf(99,1,"[%s] log2", __FUNCTION__);


#if 0
	pthread_sleepon_lock();
	if (sdio->wait_srv == 1) {
		//slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] log 3 \n", __func__);
		if (pthread_sleepon_timedwait(&sdio->wait_srv, tmo) != EOK) {
			//slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] log 4 \n", __func__);
			ret = SDIO_FAILURE;
		}
	}
	pthread_sleepon_unlock();

	if (ret != SDIO_FAILURE)
		sdio->ienable(sdio->hchdl, SDIO_INTR_SDIO, 0);
#endif
	sdio->card_intr = 0;
	//sdio->wait_srv = 0;
	return (ret);
}

void *sdio_event_handler(void *data)
{
	struct _pulse	pulse;
	iov_t			iov;
	int				rcvid;
	sdio_ext_t		*sdio = (sdio_ext_t *)data;

	// In case there is something has to be initialized in the event handler
	if (sdio->hdl_init)
		sdio->hdl_init(sdio->hchdl);

	SETIOV(&iov, &pulse, sizeof(pulse));

	while (1) {
		//slogf(99,1,"%s In\n", __FUNCTION__);

		if ((rcvid = MsgReceivev(sdio->chid, &iov, 1, NULL)) == -1)
		{
			fprintf(stderr, "sdio_event_handler 2\r\n");
			continue;
		}

		switch (pulse.code) {
			case SDIO_PULSE:
				sdio_interrupt(sdio);
				InterruptUnmask(sdio->hc_irq, sdio->hc_iid);
				break;
			default:
				fprintf(stderr, "sdio_event_handler5\r\n");
				if (rcvid)
					MsgReplyv(rcvid, ENOTSUP, &iov, 1);
				break;
		}
	}

	return (NULL);
}

