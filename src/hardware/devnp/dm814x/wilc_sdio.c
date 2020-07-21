#include <sys/slogcodes.h>
#include <arm/am335x.h>
//#include <proto.h>
#include <wilc_wfi_netdevice.h>

enum sdio_host_lock {
	WILC_SDIO_HOST_NO_TAKEN = 0,
	WILC_SDIO_HOST_IRQ_TAKEN = 1,
	WILC_SDIO_HOST_DIS_TAKEN = 2,
};

static enum sdio_host_lock	sdio_intr_lock = WILC_SDIO_HOST_NO_TAKEN;

struct wilc_dev *g_wilc = NULL;

#define CMD_CREATE(_cr, _opcode, _arg, _rsp, _eflags)	\
	do {								\
		(_cr).opcode = (_opcode);		\
		(_cr).argument = (_arg);		\
		(_cr).rsptype = (_rsp);			\
		(_cr).eflags = (_eflags);		\
	} while (0)


struct wilc_sdio {
	bool irq_gpio;
	int32_t block_size;
	int nint;
	bool is_init;
};


static const struct wilc_hif_func wilc_hif_sdio;

#define SDIO_CIA  0 /**< SDIO Function 0 (CIA) */
#define WILC_SDIO_BLOCK_SIZE 512

static int sdio_send_idle(sdio_ext_t *sdio){
	sdio_cmd_t	cmd;
	/* Send CMD0 with argument 0 */
	CMD_CREATE (cmd, MMC_GO_IDLE_STATE, 0, MMC_RSP_NONE, (SDMMC_CMD_INIT | SDMMC_CMD_INTR) );
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[sdio_send_idle] In\n");
	if (sdio_send_command(sdio, &cmd) != MMC_SUCCESS) {
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"%s failed.\n", __FUNCTION__);
		return SDIO_FAILURE;
	}
	return SDIO_SUCCESS;
}

static int sdio_send_op_cond(sdio_ext_t *sdio){

	sdio_cmd_t	cmd;
	// send CMD5 with arg 0
	CMD_CREATE(cmd, IO_SEND_OP_COND, 0, MMC_RSP_PRESENT |SDIO_RSP_R4, SDMMC_CMD_INTR);
	if (sdio_send_command(sdio, &cmd) != MMC_SUCCESS) {
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"%s %s failed.\n", __FUNCTION__, "SD_SEND_OP_COND");
		return SDIO_FAILURE;
	}
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"%s %s result= 0x%x\n", __FUNCTION__, "SD_SEND_OP_COND", cmd.resp[0]);


#define OCR_VDD_27_28          (1lu << 15)
#define OCR_VDD_28_29          (1lu << 16)
#define OCR_VDD_29_30          (1lu << 17)
#define OCR_VDD_30_31          (1lu << 18)
#define OCR_VDD_31_32          (1lu << 19)
#define OCR_VDD_32_33          (1lu << 20)
#define OCR_SDIO_MP            (1lu << 27) /**< Memory Present */

#define SD_MMC_VOLTAGE_SUPPORT \
		(OCR_VDD_27_28 | OCR_VDD_28_29 | \
		OCR_VDD_29_30 | OCR_VDD_30_31 | \
		OCR_VDD_31_32 | OCR_VDD_32_33)

	CMD_CREATE(cmd, IO_SEND_OP_COND, cmd.resp[0] | SD_MMC_VOLTAGE_SUPPORT , MMC_RSP_PRESENT |SDIO_RSP_R4, SDMMC_CMD_INTR);
	//sdio->command(sdio->hchdl, &cmd);
	if (sdio_send_command(sdio, &cmd) != MMC_SUCCESS) {
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"%s %s failed.\n", __FUNCTION__, "SD_SEND_OP_COND");
		return SDIO_FAILURE;
	}
	printf("%s IO_SEND_OP_COND. result= 0x%x\n", __FUNCTION__,cmd.resp[0]);

	if ((cmd.resp[0] & OCR_SDIO_MP) > 0) {
		printf("Card Type is SD_COMBO\r\n");

	} else {
		printf("Card Type is SDIO\r\n");

	}

	return SDIO_SUCCESS;

}

static int sdio_set_transfer_mode(sdio_ext_t *sdio){
	sdio_cmd_t	cmd;

	// Ask the card to publish a new relative address (RCA).

	CMD_CREATE(cmd, MMC_SET_RELATIVE_ADDR, 0 , MMC_RSP_PRESENT |SDIO_RSP_R4, SDMMC_CMD_INTR);
	if (sdio_send_command(sdio, &cmd) != MMC_SUCCESS) {
		printf("%s %s failed.\n", __FUNCTION__,"SD_SEND_OP_COND");
		return SDIO_FAILURE;
	}
	printf("%s MMC_SET_RELATIVE_ADDR. result= 0x%x\n", __FUNCTION__,cmd.resp[0]);
	uint16_t rca = (cmd.resp[0] >> 16) & 0xFFFF;

	// Select the and put it into Transfer Mode
	CMD_CREATE(cmd, MMC_SEL_DES_CARD, rca << 16 , MMC_RSP_PRESENT |SDIO_RSP_R4, SDMMC_CMD_INTR);

	if (sdio_send_command(sdio, &cmd) != MMC_SUCCESS) {
		printf("%s %s failed.\n", __FUNCTION__,"SD_SEND_OP_COND");
		return SDIO_FAILURE;
	}
	printf("%s MMC_SEL_DES_CARD. result= 0x%x\n", __FUNCTION__,cmd.resp[0]);
	return SDIO_SUCCESS;

}

static uint8_t sdio_get_max_speed(sdio_ext_t *sdio)
{
	sdio_cmd_t	cmd;
	uint32_t addr_new, addr_old;
	uint8_t addr_cis[4];
	uint8_t buf[6];
	uint8_t tplfe_max_tran_speed,i;

	/** Pointer to CIS (3B, LSB first) */
	#define SDIO_CCCR_CIS_PTR     0x09


/* Read CIS area address in CCCR area */
	addr_old = SDIO_CCCR_CIS_PTR;
	for(i = 0; i < 4; i++) {
		sdio_read_ioreg(sdio, SDIO_CIA, addr_old, &addr_cis[i]);
		addr_old++;
	}
	addr_old = addr_cis[0] + (addr_cis[1] << 8) + \
				(addr_cis[2] << 16) + (addr_cis[3] << 24);
	addr_new = addr_old;

	printf("%s addr_new= 0x%x\n", __FUNCTION__,addr_new);

	while (1) {
		/* Read a sample of CIA area */
		for(i=0; i<3; i++) {
			sdio_read_ioreg(sdio, SDIO_CIA, addr_new, &buf[i]);
			//sdio_cmd52(SDIO_CMD52_READ_FLAG, SDIO_CIA, addr_new, 0, &buf[i]);
			addr_new++;
		}
		if (buf[0] == SDIO_CISTPL_END) {
			return 1; /* Tuple error */
		}
		if (buf[0] == SDIO_CISTPL_FUNCE && buf[2] == 0x00) {
			break; /* Fun0 tuple found */
		}
		if (buf[1] == 0) {
			return 1; /* Tuple error */
		}
		/* Next address */
		addr_new += buf[1]-1;
		if (addr_new > (addr_old + 256)) {
			return 1; /* Outoff CIS area */
		}
	}
	printf("%s addr_new= 0x%x\n", __FUNCTION__,addr_new);

	addr_new -= 3;

	for(i = 0; i < 6; i++) {
		sdio_read_ioreg(sdio, SDIO_CIA, addr_new, &buf[i]);
		//sdio_cmd52(SDIO_CMD52_READ_FLAG, SDIO_CIA, addr_new, 0, &buf[i]);
		addr_new++;
	}

	tplfe_max_tran_speed = buf[5];
	printf("max speed = 0x%x\r\n", tplfe_max_tran_speed);

	if (tplfe_max_tran_speed >= 0x32)
	{
		printf("set high speed\r\n");
		// To Do: set high speed
	}
	return 0;
}

static int8_t sdio_cmd52_set_bus_width(sdio_ext_t *sdio)
{
	/**
	 * A SDIO Full-Speed alone always supports 4bit
	 */
	uint8_t u8_value;

#define 	SDIO_CCCR_CAP         0x08         /**< Card Capability */
/** 4-bit support for Low-Speed Card (RO) */
#define 	SDIO_CAP_4BLS         (0x1lu << 7)
#define   	SDIO_BUSWIDTH_4B      (0x2lu << 0)  /**< 4-bit data bus */
#define 	SDIO_CCCR_BUS_CTRL    0x07         /**< Bus Interface Control */

	// Check 4bit support in 4BLS of "Card Capability" register
	if (sdio_read_ioreg(sdio, SDIO_CIA, SDIO_CCCR_CAP, &u8_value) == SDIO_FAILURE)
	{
		return SDIO_FAILURE;
	}

	if ((u8_value & SDIO_CAP_4BLS) != SDIO_CAP_4BLS)
	{
		// No supported, it is not a protocol error
		return SDIO_SUCCESS;
	}
	// HS mode possible, then enable
	u8_value = SDIO_BUSWIDTH_4B;
	if (sdio_write_ioreg(sdio, SDIO_CIA, SDIO_CCCR_BUS_CTRL, u8_value) == SDIO_FAILURE)
	{
		return SDIO_FAILURE;
	}
	if (sdio_read_ioreg(sdio, SDIO_CIA, SDIO_CCCR_BUS_CTRL, &u8_value) == SDIO_FAILURE)
	{
			return SDIO_FAILURE;

	}

	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] u8_value = 0x%x\n", __func__, u8_value);
	//sd_mmc_card->bus_width = 4;
	//sd_mmc_debug("%d-bit bus width enabled.\n\r", (int)sd_mmc_card->bus_width);
	return SDIO_SUCCESS;
}


static int8_t sdio_cmd52_set_high_speed(sdio_ext_t *sdio)
{
	uint8_t u8_value;

#define 	SDIO_CCCR_HS          0x13         /**< High-Speed */
#define   	SDIO_SHS              (0x1lu << 0)  /**< Support High-Speed (RO) */
#define   	SDIO_EHS              (0x1lu << 1)  /**< Enable High-Speed (R/W) */

	// Check CIA.HS
	if (sdio_read_ioreg(sdio, SDIO_CIA, SDIO_CCCR_HS, &u8_value) == SDIO_FAILURE)
	{
		return SDIO_FAILURE;
	}

	if ((u8_value & SDIO_SHS) != SDIO_SHS) {
		// No supported, it is not a protocol error
		return SDIO_SUCCESS;
	}
	// HS mode possible, then enable
	u8_value = SDIO_EHS;

	if (sdio_write_ioreg(sdio, SDIO_CIA, SDIO_CCCR_HS, u8_value) == SDIO_FAILURE)
	{
		return SDIO_FAILURE;
	}
	//printf("u8_value = 0x%x\r\n", u8_value);
	if (sdio_read_ioreg(sdio, SDIO_CIA, SDIO_CCCR_HS, &u8_value) == SDIO_FAILURE)
	{
		return SDIO_FAILURE;
	}

	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] u8_value = 0x%x\n", __func__, u8_value);


	return SDIO_SUCCESS;
}

static int8_t set_high_speed_and_4_bit_width(sdio_ext_t *sdio)
{
	int speed = 48000000;
	if (sdio->bus_speed(sdio->hchdl, &speed) != MMC_SUCCESS)
		printf("set bus speed fail\r\n");

	uint8_t bw = 4;

	if (sdio->bus_width(sdio->hchdl, bw) != MMC_SUCCESS)
	printf("set bus width fail\r\n");

	return SDIO_SUCCESS;
}

int
sdio_read_iomem_v1(void *hdl, uint8_t fn, uint32_t address,
				uint8_t opcode, uint32_t blkcnt, uint32_t blksz, uint8_t *buffer, off64_t paddr)
{
	sdio_ext_t	*sdio = (sdio_ext_t *)hdl;
	int			nbytes = blkcnt * blksz;
	int rdbytes = 0;
	sdio_cmd_t	cmd;

	/*
	if (sdio->blksz[fn] != blksz) {
		if (sdio_set_blksz(hdl, fn, blksz) != SDIO_SUCCESS)
			return (SDIO_FAILURE);
	}
	*/

	cmd.opcode   = 53;
	cmd.rsptype  = SDIO_RSP_R5;
	cmd.eflags   = SDMMC_CMD_DATA_IN | SDMMC_CMD_DATA | (blkcnt > 1 ? SDMMC_CMD_DATA_MULTI : 0);
	///cmd.eflags   = SDMMC_CMD_DATA_IN | SDMMC_CMD_DATA;
	cmd.argument = MAKE_SDIO_OFFSET(address) |
				    MAKE_SDIO_OP_CODE(1) |
				    MAKE_SDIO_FUNCTION(fn) |
					MAKE_SDIO_DIR(0);

	if((blkcnt==1)||(blkcnt==0))
		cmd.argument |= blksz;		// byte mode
	else
		cmd.argument |= MAKE_SDIO_BLOCK_MODE(1)|(blkcnt & 0x1ff );		// block mode

	//cmd.blkcnt   = blkcnt;			// block count;
	//cmd.size     = blksz;			// block size
	if (sdio->eflags & SDMMC_CAP_DMA) {
		CACHE_INVAL(&sdio->cachectl, buffer, paddr, nbytes);

		if (sdio->setup_dma(sdio, paddr, nbytes, MMC_DIR_IN) != nbytes)
			return (SDIO_FAILURE);

		cmd.eflags |= SDMMC_CMD_DATA_DMA;
		if (sdio_send_command(sdio, &cmd) != MMC_SUCCESS)
			return (SDIO_FAILURE);

		if (sdio->dma_done(sdio, MMC_DIR_IN) != MMC_SUCCESS)
			return (SDIO_FAILURE);
	} else {

		int byte_ret = sdio->setup_pio(sdio, (char *)buffer, nbytes, MMC_DIR_IN);
		//printf("byte_ret = %d, nbytes = %d\n", byte_ret, nbytes);
		if ( byte_ret != nbytes)
		{
			return (SDIO_FAILURE);
		}
;
		cmd.data = buffer;			// buffer pointer

		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] sdio_send_command\n", __func__);
		if (sdio_send_command(sdio, &cmd) != MMC_SUCCESS)
			return (SDIO_FAILURE);
		if ((rdbytes = sdio->pio_done(sdio, (char *)buffer, nbytes, MMC_DIR_IN) )!= nbytes){
			printf("Warning: get %d bytes.\n",rdbytes);
			return (SDIO_FAILURE);
		}

		if (blkcnt > 1)
		printf("CMD53 read...\r\n");
		//printf("Debug: get %d bytes. data0=0x%x, data1=0x%x, data2=0x%x, data3=0x%x\n", rdbytes, buffer[0], buffer[1], buffer[2], buffer[3]);



	}

	return (SDIO_SUCCESS);
}

int
sdio_write_iomem_v1(void *hdl, uint8_t fn, uint32_t address,
				uint8_t opcode, uint32_t blkcnt, uint32_t blksz, uint8_t *buffer, off64_t paddr)
{

	sdio_ext_t	*sdio = (sdio_ext_t *)hdl;
	int			nbytes = blkcnt * blksz;
	sdio_cmd_t	cmd;

	//return SDIO_SUCCESS;
	/*
	if (sdio->blksz[fn] != blksz) {
		if (sdio_set_blksz(hdl, fn, blksz) != SDIO_SUCCESS)
			return (SDIO_FAILURE);
	}
	 */

	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] blkcnt = %d, blksz = %d\n", __func__, blkcnt, blksz);

	/*
	if (blkcnt > 1)
	{
		blkcnt = 2;
		blksz = 256;
		nbytes = blkcnt * blksz;
		sdio->block_size(sdio->hchdl, 256);
	}
	*/
	cmd.opcode   = 53;
	cmd.rsptype  = SDIO_RSP_R5;
	cmd.eflags   = SDMMC_CMD_DATA | (blkcnt > 1 ? SDMMC_CMD_DATA_MULTI : 0);
	//cmd.eflags   = SDMMC_CMD_DATA ;
	cmd.argument = MAKE_SDIO_OFFSET(address) |
				    MAKE_SDIO_OP_CODE(1) |
				    MAKE_SDIO_FUNCTION(fn) |
				    MAKE_SDIO_DIR(1);
	//cmd.blkcnt   = blkcnt;			// block count;
	//cmd.size     = blksz;			// block size

	if((blkcnt==1)||(blkcnt==0))
		cmd.argument |= blksz;		// byte mode
	else
		///cmd.argument |= blksz;		// byte mode
		//cmd.argument |= (blkcnt & 0x1ff );		// block mode
		cmd.argument |= MAKE_SDIO_BLOCK_MODE(1)|(blkcnt & 0x1ff );		// block mode

	if (sdio->eflags & SDMMC_CAP_DMA) {
		CACHE_FLUSH(&sdio->cachectl, buffer, paddr, blksz * blkcnt);

		if (sdio->setup_dma(sdio, paddr, nbytes, MMC_DIR_OUT) != nbytes)
			return (SDIO_FAILURE);

		cmd.eflags |= SDMMC_CMD_DATA_DMA;
		if (sdio_send_command(sdio, &cmd) != MMC_SUCCESS)
			return (SDIO_FAILURE);



		if (sdio->dma_done(sdio, 0) != MMC_SUCCESS)
			return (SDIO_FAILURE);
	} else {

		//if (sdio->setup_pio(sdio, (char *)buffer, nbytes, MMC_DIR_IN) != nbytes)
		if (sdio->setup_pio(sdio, (char *)buffer, nbytes, MMC_DIR_OUT) != nbytes)
			return (SDIO_FAILURE);

		cmd.data = buffer;			// buffer pointer

		if (sdio_send_command(sdio, &cmd) != MMC_SUCCESS)
			return (SDIO_FAILURE);

		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] send_command finish\n", __func__);
		if (sdio->pio_done(sdio, (char *)buffer, nbytes, MMC_DIR_OUT) != nbytes)
			return (SDIO_FAILURE);

		sdio->wait_cmd = 1;
		pthread_sleepon_lock();
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] transfer wait\n", __func__);
		if (pthread_sleepon_timedwait(&sdio->wait_cmd, 1000 * 1000 * 1000) != EOK)
		{
			slogf(_SLOGC_NETWORK, _SLOG_ERROR,"wait cmd53 transfer timeout...\n");
		}
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] transfer finish\n", __func__);
		pthread_sleepon_unlock();

	}

	return (SDIO_SUCCESS);
}

static void wilc_sdio_interrupt(struct wilc_dev *wilc)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] In \n", __func__);
	if (sdio_intr_lock == WILC_SDIO_HOST_DIS_TAKEN)
		return;
	sdio_intr_lock = WILC_SDIO_HOST_IRQ_TAKEN;
	//sdio_release_host(func);
	wilc_handle_isr(wilc);
	//sdio_claim_host(func);
	sdio_intr_lock = WILC_SDIO_HOST_NO_TAKEN;
	//wake_up_interruptible(&sdio_intr_waitqueue);
}

void sdio_interrupt(void)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] In \n", __func__);
	wilc_sdio_interrupt(g_wilc);
}
static int wilc_sdio_cmd52(sdio_ext_t *sdio, uint8_t fn, struct sdio_cmd52 *cmd)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] log0\n", __func__);
	int ret;
	uint8_t data;
	//printf("[wilc_sdio_cmd52] debug log1 \r\n");
	do{
		if (cmd->read_write) {  /* write */
			slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s_log_w] addr=0x%x, data=0x%x \n", __func__,cmd->address,cmd->data);
			if (cmd->raw) {
				slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] log11\n", __func__);
				//sdio_writeb(func, cmd->data, cmd->address, &ret);
				//printf("wilc_sdio_cmd52 raw start\r\n");
				ret = sdio_write_ioreg(sdio, cmd->function, cmd->address, cmd->data);
				//printf("raw1 cmd->address=0x%x, cmd->data=0x%x, ret= %d \r\n", cmd->address, cmd->data, ret);
				//data = sdio_readb(func, cmd->address, &ret);
				ret = sdio_read_ioreg(sdio, cmd->function, cmd->address, &data);

				cmd->data = data;
				//printf("raw2 cmd->address=0x%x, cmd->data=0x%x, ret= %d \r\n", cmd->address, cmd->data, ret);
			} else {
				slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] log1\n", __func__);
				///slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] log0\n", __func__);
				//sdio_writeb(func, cmd->data, cmd->address, &ret);
				//printf("cmd->address=0x%x, cmd->data=0x%x \r\n", cmd->address, cmd->data);
				ret = sdio_write_ioreg(sdio, cmd->function, cmd->address, cmd->data);
				slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] log2\n", __func__);

			}
		} else {        /* read */

			//data = sdio_readb(func, cmd->address, &ret);
			///slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] log1\n", __func__);
			ret = sdio_read_ioreg(sdio, cmd->function, cmd->address, &data);
			cmd->data = data;
			slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s_log_r] addr=0x%x, data=0x%x \n", __func__,cmd->address,cmd->data);
		}


		if (ret)
			slogf(_SLOGC_NETWORK, _SLOG_ERROR,"%s..failed, err(%d)\n", __func__, ret);
	} while (ret);
	return ret;
}

static int wilc_sdio_cmd53(sdio_ext_t *sdio, uint8_t fn, struct sdio_cmd53 *cmd)
{
	int size, ret;
	uint32_t block_size;
	uint32_t block_cnt;

	//block_size = (cmd->block_mode == BLOCK_MODE) ? cmd->block_size: 1;

	if (cmd->block_mode)
	{
		block_cnt = cmd->count;
		block_size = cmd->block_size;
		sdio->block_size(sdio->hchdl, block_size);
	}
	else // byte mode
	{
		block_cnt = 1;
		block_size = cmd->count;
		sdio->block_size(sdio->hchdl, block_size);
	}

	if (cmd->read_write) {  /* write */

		sdio->eflags &= ~SDMMC_CAP_DMA;	// no dma
		///block_size = (cmd->block_mode == BLOCK_MODE) ? cmd->block_size: 1;

		//ret = sdio_write_iomem(sdio, cmd->function, cmd->address,
		//								0, block_cnt, block_size, (void *)cmd->buffer, 0);

		//int transmit_pwr_mode;
		//scanf("%d",&transmit_pwr_mode);

		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s_log_w] addr=0x%x, block_cnt=0x%x, block_size=0x%x, data=0x%x \n", __func__,cmd->address,block_cnt, block_size, cmd->buffer[0]);
		ret = sdio_write_iomem_v1(sdio, cmd->function, cmd->address,
								0, block_cnt, block_size, (void *)cmd->buffer, 0);
	} else {        /* read */
		//ret = sdio_memcpy_fromio(func, (void *)cmd->buffer,
		//			 cmd->address,  size);
		sdio->eflags &= ~SDMMC_CAP_DMA;	// no dma
		//block_size = (cmd->block_mode == BLOCK_MODE) ? cmd->block_size: 1;

		///printf("DEBUG1, block_size = %d, cmd->count = %d\r\n", block_size, block_cnt);

		///sdio->block_size(sdio->hchdl, cmd->count);
		ret = sdio_read_iomem_v1(sdio, cmd->function, cmd->address,
								0, block_cnt, block_size, (void *)cmd->buffer, 0);
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s_log_r] addr=0x%x, block_cnt=0x%x, block_size=0x%x, data=0x%x \n", __func__,cmd->address,block_cnt, block_size, cmd->buffer[0]);
	}


	if (ret)
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"%s..failed, err(%d)\n", __func__, ret);

	return ret;
}

static int wilc_sdio_set_func0_block_size(sdio_ext_t *sdio, uint32_t block_size)
{
	struct sdio_cmd52 cmd;
	int ret;

	cmd.read_write = 1;
	cmd.function = 0;
	cmd.raw = 0;
	cmd.address = 0x10;
	cmd.data = (uint8_t)block_size;
	ret = wilc_sdio_cmd52(sdio, 0, &cmd);
	if (ret) {
		printf("Failed cmd52, set 0x10 data...\n");
		goto fail;
	}

	cmd.address = 0x11;
	cmd.data = (uint8_t)(block_size >> 8);
	ret = wilc_sdio_cmd52(sdio, 0, &cmd);
	if (ret) {
		printf("Failed cmd52, set 0x11 data...\n");
		goto fail;
	}

	return 1;
fail:
	return 0;
}

static int wilc_sdio_set_func1_block_size(sdio_ext_t *sdio, uint32_t block_size)
{
	struct sdio_cmd52 cmd;
	int ret;

	cmd.read_write = 1;
	cmd.function = 0;
	cmd.raw = 0;
	cmd.address = 0x110;
	cmd.data = (uint8_t)block_size;
	ret = wilc_sdio_cmd52(sdio, 0, &cmd);
	if (ret) {
		printf("Failed cmd52, set 0x110 data...\n");
	}
	cmd.address = 0x111;
	cmd.data = (uint8_t)(block_size >> 8);
	ret = wilc_sdio_cmd52(sdio, 0, &cmd);
	if (ret) {
		printf("Failed cmd52, set 0x111 data...\n");
	}

	return 1;
fail:
	return 0;
}

static int sdio_set_func0_csa_address(sdio_ext_t *sdio, uint8_t fn, uint32_t adr)
{
	struct sdio_cmd52 cmd;
	int ret;

	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] address = 0x%x\n", __func__, adr);
	/**
	 *      Review: BIG ENDIAN
	 **/
	cmd.read_write = 1;
	cmd.function = 0;
	cmd.raw = 0;
	cmd.address = 0x10c;
	cmd.data = (uint8_t)adr;
	//slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] log12\n", __func__);
	ret = wilc_sdio_cmd52(sdio, fn, &cmd);
	//slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] log11\n", __func__);
	if (ret) {
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] Failed cmd52, set 0x10c data...\n", __func__);
		goto fail;
	}

	cmd.address = 0x10d;
	cmd.data = (uint8_t)(adr >> 8);
	ret = wilc_sdio_cmd52(sdio, fn, &cmd);
	if (ret) {
		printf("Failed cmd52, set 0x10d data...\n");
		goto fail;
	}

	cmd.address = 0x10e;
	cmd.data = (uint8_t)(adr >> 16);
	ret = wilc_sdio_cmd52(sdio, fn, &cmd);
	if (ret) {
		printf("Failed cmd52, set 0x10e data...\n");
		goto fail;
	}

	return 1;
fail:

	return 0;
}



static uint8_t attach_wilc(void *hdl)
{
	sdio_ext_t *sdio = (sdio_ext_t *)hdl;
#define OCR_SDIO_NF			(7 << 28)

	sdio->powerdown(sdio->hchdl);
	delay(50);
	sdio->powerup(sdio->hchdl);
	delay(100);

	//sdio_intr_enable(sdio, 1);
	sleep(1);

	// send CMD52 with bit3 of address 0x06 [SD IO card reset]
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] In\n", __func__);
	int ret = sdio_write_ioreg(sdio, 0, 0x6, 0x8);
	///printf("data write, ret =%d\r\n", ret);
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"data write, ret =%d\n", ret);
	//sdio_intr_enable(sdio, 1);
	// send CMD0 with arg 0

	if(sdio_send_idle(sdio) != SDIO_SUCCESS){
			slogf(_SLOGC_NETWORK, _SLOG_ERROR,"%s failed.\n", "sdio_send_idle.");
			return SDIO_FAILURE;
		}

	delay(10);

	if(sdio_send_op_cond(sdio) != SDIO_SUCCESS){
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"%s failed.\n", "sdio_send_op_cond.");
		return SDIO_FAILURE;
	}
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"data sdio_send_op_cond, success\r\n");


	if (sdio_set_transfer_mode(sdio) != SDIO_SUCCESS)
	{
		printf("%s failed.\n", "sdio_set_transfer_mode.\n");
		return SDIO_FAILURE;
	}

	if (sdio_get_max_speed(sdio)) {
		printf("Fail to get max speed\r\n");
		return SDIO_FAILURE;
	}


	// TRY to enable 4-bit mode

	if (sdio_cmd52_set_bus_width(sdio) == SDIO_FAILURE) {
		return SDIO_FAILURE;
	}

	set_high_speed_and_4_bit_width(sdio);


	if (sdio_cmd52_set_high_speed(sdio) == SDIO_FAILURE) {
		return SDIO_FAILURE;
	}
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"attach_wilc() return success\n");
	return SDIO_SUCCESS;
}



void wilc_wlan_power_on_sequence(struct wilc *wilc)
{
	///wilc_wlan_power(wilc, 0);
	///wilc_wlan_power(wilc, 1);
}


int linux_sdio_probe(struct wilc_dev *wilc)
{
	struct wilc *wilc_priv;
	int ret, io_type;
	static bool init_power;
	struct wilc_sdio *sdio_priv;
	//sdio_ext_t*	sdio = NULL;

	printf("[%s] In \r\n", __func__);

	if (ThreadCtl(_NTO_TCTL_IO, NULL) == -1) {
		perror("ThreadCtl");
		exit(EXIT_FAILURE);
	}

	if (sdio_init((void**)&(wilc->sdio)) != SDIO_SUCCESS)
	{
		printf("%s : Initialize SDIO interface failed.\n",__FUNCTION__);
		return 1;
	}
	//sdio_intr_enable(wilc->sdio, 1);


	attach_wilc(wilc->sdio);

	sdio_intr_enable(wilc->sdio, 1);
	printf("[linux_sdio_probe] log2 \r\n");

	g_wilc = wilc;
	sdio_intr_callback_register(sdio_interrupt);



	sdio_ext_t*	sdio = wilc->sdio;
	sdio->block_size(sdio->hchdl, WILC_SDIO_BLOCK_SIZE);


	sdio_priv = (struct wilc_sdio *) create_ptr(sizeof(*sdio_priv));
	if (!sdio_priv)
		return -1;

#ifdef CONFIG_WILC_HW_OOB_INTR

	io_type = HIF_SDIO_GPIO_IRQ;
#else
		io_type = HIF_SDIO;
#endif

#if 1

	/*
	ret = wilc_netdev_init(wilc, &wilc_priv, io_type, &wilc_hif_sdio);
	if (ret) {
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"Couldn't initialize netdev\n");
		free_ptr(sdio_priv);
		return ret;
	}
	*/

	ret = wilc_cfg80211_init(wilc, io_type, &wilc_hif_sdio);
		if (ret) {
			slogf(_SLOGC_NETWORK, _SLOG_ERROR,"Couldn't initialize netdev\n");
			kfree(sdio_priv);
			return ret;
		}
#endif
	///sdio_set_drvdata(func, wilc);
	wilc->bus_data = sdio_priv;
	///wilc->dev = &func->dev;
	///wilc->dt_dev = &func->card->dev;

	if (!init_power) {
		wilc_wlan_power_on_sequence(wilc_priv);
		init_power = 1;
	}

	//pthread_mutex_init(&wilc->hif_cs, NULL);
	///mutex_init(&wilc->cs);

	///wilc_bt_init(wilc);

	printf("Driver Initializing success\n");
	return 0;
}

static int sdio_drv_init(struct wilc_dev *wilc, bool resume)
{

	struct wilc_sdio *sdio_priv = wilc->bus_data;
	struct sdio_cmd52 cmd;
	int ret;
	uint32_t chipid;
	uint8_t fn = 0;
	int loop;

	/**
	 *      function 0 csa enable
	 **/

	cmd.read_write = 1;
	cmd.function = 0;
	cmd.raw = 1;
	cmd.address = 0x100;
	cmd.data = 0x80;
	ret = wilc_sdio_cmd52(wilc->sdio, fn, &cmd);

	if (ret) {
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"Fail cmd 52, enable csa...\n");
	}

	/**
	 *      function 0 block size
	 **/
	if (!wilc_sdio_set_func0_block_size(wilc->sdio, WILC_SDIO_BLOCK_SIZE)) {
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"Fail cmd 52, set func 0 block size...\n");

	}

	sdio_priv->block_size = WILC_SDIO_BLOCK_SIZE;
	/**
	 *      enable func1 IO
	 **/
	cmd.read_write = 1;
	cmd.function = 0;
	cmd.raw = 1;
	cmd.address = 0x2;
	cmd.data = 0x2;
	ret = wilc_sdio_cmd52(wilc->sdio, 0, &cmd);
	if (ret) {
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"Fail cmd 52, set IOE register...\n");
	}

	/**
	 *      make sure func 1 is up
	 **/
	cmd.read_write = 0;
	cmd.function = 0;
	cmd.raw = 0;
	cmd.address = 0x3;
	loop = 3;
	do {
		cmd.data = 0;
		ret = wilc_sdio_cmd52(wilc->sdio, 0, &cmd);
		if (ret) {
			slogf(_SLOGC_NETWORK, _SLOG_ERROR,"Fail cmd 52, get IOR register...\n");
		}
		if (cmd.data == 0x2)
			break;
	} while (loop--);

	if (loop <= 0) {
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"Fail func 1 is not ready...\n");
	}



	/**
	 *      func 1 is ready, set func 1 block size
	 **/
	if (!wilc_sdio_set_func1_block_size(wilc->sdio, WILC_SDIO_BLOCK_SIZE)) {
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] Fail set func 1 block size...\n", __func__);
	}

	/**
	 *      func 1 interrupt enable
	 **/
	cmd.read_write = 1;
	cmd.function = 0;
	cmd.raw = 1;
	cmd.address = 0x4;
	cmd.data = 0x3;
	ret = wilc_sdio_cmd52(wilc->sdio, 0, &cmd);
	if (ret) {
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] ail cmd 52, set IEN register...\n", __func__);
	}

	if (!resume) {
		chipid = wilc_get_chipid(wilc, 1);
	}

	sdio_priv->is_init = true;

	return 1;
}

static bool sdio_is_init(struct wilc_dev *wilc)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"sdio_is_init () log0\n");
	struct wilc_sdio *sdio_priv = wilc->bus_data;

	return sdio_priv->is_init;
	return 0;
}

static int sdio_deinit(struct wilc_dev *wilc)
{
	struct wilc_sdio *sdio_priv = wilc->bus_data;

	sdio_priv->is_init = false;

	return 1;
}

static int sdio_read_size(struct wilc_dev *wilc, uint32_t *size)
{
	uint32_t tmp;
	struct sdio_cmd52 cmd;

	/**
	 *      Read DMA count in words
	 **/
	cmd.read_write = 0;
	cmd.function = 0;
	cmd.raw = 0;
	cmd.address = 0xf2;
	cmd.data = 0;
	wilc_sdio_cmd52(wilc->sdio, 0, &cmd);
	tmp = cmd.data;

	cmd.address = 0xf3;
	cmd.data = 0;
	wilc_sdio_cmd52(wilc->sdio, 0, &cmd);
	tmp |= (cmd.data << 8);

	*size = tmp;
	return 1;
}

static int sdio_read_int(struct wilc_dev *wilc, uint32_t *int_status)
{
	//struct sdio_func *func = dev_to_sdio_func(wilc->dev);
	struct wilc_sdio *sdio_priv = wilc->bus_data;
	uint32_t tmp;
	struct sdio_cmd52 cmd;
	uint32_t irq_flags;
	int i;
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] In\n", __func__);
	if (sdio_priv->irq_gpio) {
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] log1\n", __func__);
		sdio_read_size(wilc, &tmp);

		cmd.read_write = 0;
		cmd.function = 1;
		cmd.raw = 0;
		cmd.data = 0;
		if (wilc->chip == WILC_1000) {
			cmd.address = 0xf7;
			wilc_sdio_cmd52(wilc->sdio, 0, &cmd);
			irq_flags = cmd.data & 0x1f;
		} else {
			cmd.address = 0xfe;
			wilc_sdio_cmd52(wilc->sdio, 0, &cmd);
			irq_flags = cmd.data & 0x0f;
		}
		tmp |= ((irq_flags >> 0) << IRG_FLAGS_OFFSET);

		*int_status = tmp;
	} else {
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] log2\n", __func__);
		sdio_read_size(wilc, &tmp);
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] log3\n", __func__);
		cmd.read_write = 0;
		cmd.function = 1;
		cmd.address = 0x04;
		cmd.data = 0;
		wilc_sdio_cmd52(wilc->sdio, 0, &cmd);
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] log4\n", __func__);

		if (cmd.data & BIT(0))
			tmp |= INT_0;
		if (cmd.data & BIT(2))
			tmp |= INT_1;
		if (cmd.data & BIT(3))
			tmp |= INT_2;
		if (cmd.data & BIT(4))
			tmp |= INT_3;
		if (cmd.data & BIT(5))
			tmp |= INT_4;

		for (i = sdio_priv->nint; i < MAX_NUM_INT; i++) {
			if ((tmp >> (IRG_FLAGS_OFFSET + i)) & 0x1) {
				slogf(_SLOGC_NETWORK, _SLOG_ERROR,"Unexpected interrupt (1) : tmp=%x, data=%x\n", tmp, cmd.data);
				break;
			}
		}

		*int_status = tmp;

	}

	return 1;
}

static int sdio_read(struct wilc_dev *wilc, uint32_t addr, uint8_t *buf, uint32_t size)
{
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] In\n", __func__);
	//struct sdio_func *func = dev_to_sdio_func(wilc->dev);
	struct wilc_sdio *sdio_priv = wilc->bus_data;
	uint32_t block_size = sdio_priv->block_size;
	struct sdio_cmd53 cmd;
	int nblk, nleft, ret;

	cmd.read_write = 0;
	if (addr > 0) {
		/**
		 *      has to be word aligned...
		 **/
		if (size & 0x3) {
			size += 4;
			size &= ~0x3;
		}

		/**
		 *      func 0 access
		 **/
		cmd.function = 0;
		cmd.address = 0x10f;
	} else {
		/**
		 *      has to be word aligned...
		 **/
		if (size & 0x3) {
			size += 4;
			size &= ~0x3;
		}

		/**
		 *      func 1 access
		 **/
		cmd.function = 1;
		cmd.address = 0;
	}

	nblk = size / block_size;
	nleft = size % block_size;

	if (nblk > 0) {
		cmd.block_mode = 1;
		cmd.increment = 1;
		cmd.count = nblk;
		cmd.buffer = buf;
		cmd.block_size = block_size;
		if (addr > 0) {
			if (!sdio_set_func0_csa_address(wilc->sdio, 0, addr))
				goto fail;
		}
		ret = wilc_sdio_cmd53(wilc->sdio, 0, &cmd);
		if (ret) {
			slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] Failed cmd53 [%x], block read...\n", __func__, addr);
			goto fail;
		}
		if (addr > 0)
			addr += nblk * block_size;
		buf += nblk * block_size;
	}       /* if (nblk > 0) */

	if (nleft > 0) {
		cmd.block_mode = 0;
		cmd.increment = 1;
		cmd.count = nleft;
		cmd.buffer = buf;

		cmd.block_size = block_size;

		if (addr > 0) {
			if (!sdio_set_func0_csa_address(wilc->sdio, 0, addr))
				goto fail;
		}
		ret = wilc_sdio_cmd53(wilc->sdio, 0, &cmd);
		if (ret) {
			slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] Failed cmd53 [%x], bytes read...\n", __func__, addr);
			goto fail;
		}
	}

	return 1;

fail:

	return 0;
}

static int sdio_write(struct wilc_dev *wilc, uint32_t addr, uint8_t *buf, uint32_t size)
{
	//struct sdio_func *func = dev_to_sdio_func(wilc->dev);
	struct wilc_sdio *sdio_priv = wilc->bus_data;
	uint32_t block_size = sdio_priv->block_size;
	struct sdio_cmd53 cmd;
	int nblk, nleft, ret;

	cmd.read_write = 1;
	if (addr > 0) {
		/**
		 *      has to be word aligned...
		 **/
		if (size & 0x3) {
			size += 4;
			size &= ~0x3;
		}

		/**
		 *      func 0 access
		 **/
		cmd.function = 0;
		cmd.address = 0x10f;
	} else {
		/**
		 *      has to be word aligned...
		 **/
		if (size & 0x3) {
			size += 4;
			size &= ~0x3;
		}

		/**
		 *      func 1 access
		 **/
		cmd.function = 1;
		cmd.address = 0;
	}

	nblk = size / block_size;
	nleft = size % block_size;

	if (nblk > 0) {
		cmd.block_mode = 1;
		cmd.increment = 1;
		cmd.count = nblk;
		cmd.buffer = buf;
		cmd.block_size = block_size;
		if (addr > 0) {
			if (!sdio_set_func0_csa_address(wilc->sdio, 0, addr))
				goto fail;
		}
		ret = wilc_sdio_cmd53(wilc->sdio, 0, &cmd);
		if (ret) {
			printf("Failed cmd53 [%x], block send...\n", addr);
			goto fail;
		}
		if (addr > 0)
			addr += nblk * block_size;
		buf += nblk * block_size;
	}

	if (nleft > 0) {
		cmd.block_mode = 0;
		cmd.increment = 1;
		cmd.count = nleft;
		cmd.buffer = buf;

		cmd.block_size = block_size;

		if (addr > 0) {
			if (!sdio_set_func0_csa_address(wilc->sdio, 0, addr))
				goto fail;
		}
		ret = wilc_sdio_cmd53(wilc->sdio, 0, &cmd);
		if (ret) {
			printf("Failed cmd53 [%x], bytes send...\n", addr);
			goto fail;
		}
	}

	return 1;

fail:

	return 0;
}


static int sdio_read_reg(struct wilc_dev *wilc, uint32_t addr, uint32_t *data)
{
	//struct sdio_func *func = dev_to_sdio_func(wilc->dev);
	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] log0\n", __func__);
	struct wilc_sdio *sdio_priv = wilc->bus_data;
	int ret;

	if (addr >= 0xf0 && addr <= 0xff) {
		struct sdio_cmd52 cmd;

		cmd.read_write = 0;
		cmd.function = 0;
		cmd.raw = 0;
		cmd.address = addr;
		slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] log1\n", __func__);
		ret = wilc_sdio_cmd52(wilc->sdio, 0, &cmd);

		if (ret) {
			printf("Failed cmd 52, read reg (%08x) ...\n", addr);
			goto fail;
		}
		*data = cmd.data;

	} else {


		struct sdio_cmd53 cmd;
		if (!sdio_set_func0_csa_address(wilc->sdio, 0, addr))
			goto fail;

		cmd.read_write = 0;
		cmd.function = 0;
		cmd.address = 0x10f;
		cmd.block_mode = 0;
		cmd.increment = 1;
		cmd.count = 4;
		cmd.buffer = (uint8_t *) data;

		cmd.block_size = sdio_priv->block_size;

		ret = wilc_sdio_cmd53(wilc->sdio, 0, &cmd);
		if (ret) {
			printf("Failed cmd53, read reg (%08x)...\n", addr);
			goto fail;
		}

		///slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] log0, data = 0x%x\n", __func__, *data);


	}

	///le32_to_cpus(data);

	return 1;

fail:
	return 0;
}

void wilc_sdio_remove(struct wilc_dev *wilc)
{
	//struct wilc *wilc = sdio_get_drvdata(func);

	//if (!IS_ERR(wilc->rtc_clk))
	//	clk_disable_unprepare(wilc->rtc_clk);

	wilc_netdev_cleanup(wilc);
	//wilc_bt_deinit();
}
static int wilc_sdio_reset(struct wilc_dev *wilc)
{
	struct sdio_cmd52 cmd;
	int ret;
	//struct sdio_func *func = dev_to_sdio_func(wilc->dev);

	printf("De Init SDIO\n");

	cmd.read_write = 1;
	cmd.function = 0;
	cmd.raw = 0;
	cmd.address = 0x6;
	cmd.data = 0x8;
	ret = wilc_sdio_cmd52(wilc->sdio, 0, &cmd);
	if (ret)
		printf("Fail cmd 52, reset cmd\n");
	return ret;
}
static int sdio_write_reg(struct wilc_dev *wilc, uint32_t addr, uint32_t data)
{
	//struct sdio_func *func = dev_to_sdio_func(wilc->dev);
	struct wilc_sdio *sdio_priv = wilc->bus_data;
	int ret;

	///cpu_to_le32s(&data);

	if (addr >= 0xf0 && addr <= 0xff) {
		struct sdio_cmd52 cmd;

		cmd.read_write = 1;
		cmd.function = 0;
		cmd.raw = 0;
		cmd.address = addr;
		cmd.data = data;
		ret = wilc_sdio_cmd52(wilc->sdio, 0, &cmd);
		if (ret) {
			printf("Failed cmd 52, write reg %08x ...\n", addr);
			goto fail;
		}
	} else {
		struct sdio_cmd53 cmd;

		/**
		 *      set the AHB address
		 **/
		if (!sdio_set_func0_csa_address(wilc->sdio, 0, addr))
			goto fail;

		cmd.read_write = 1;
		cmd.function = 0;
		cmd.address = 0x10f;
		cmd.block_mode = 0;
		cmd.increment = 1;
		cmd.count = 4;
		cmd.buffer = (uint8_t *)&data;
		cmd.block_size = sdio_priv->block_size;
		ret = wilc_sdio_cmd53(wilc->sdio, 0, &cmd);
		if (ret) {
			printf("Failed cmd53, write reg (%08x)...\n", addr);
			goto fail;
		}
	}

	return 1;

fail:

	return 0;

}

static int sdio_clear_int_ext(struct wilc_dev *wilc, uint32_t val)
{
	//struct sdio_func *func = dev_to_sdio_func(wilc->dev);
	struct wilc_sdio *sdio_priv = wilc->bus_data;
	int ret;
	uint32_t reg = 0;

	slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] In\n", __func__);

	if (wilc->chip == WILC_1000) {
		if (sdio_priv->irq_gpio)
			reg = val & (BIT(MAX_NUM_INT) - 1);

		/* select VMM table 0 */
		if (val & SEL_VMM_TBL0)
			reg |= BIT(5);
		/* select VMM table 1 */
		if (val & SEL_VMM_TBL1)
			reg |= BIT(6);
		/* enable VMM */
		if (val & EN_VMM)
			reg |= BIT(7);
		if (reg) {
			struct sdio_cmd52 cmd;

			cmd.read_write = 1;
			cmd.function = 0;
			cmd.raw = 0;
			cmd.address = 0xf8;
			cmd.data = reg;

			ret = wilc_sdio_cmd52(wilc->sdio, 0, &cmd);
			if (ret) {
				slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] Failed cmd52, set 0xf8 data\n", __func__);
				goto fail;
			}
		}
	} else {
		if (sdio_priv->irq_gpio) {
			reg = val & (BIT(MAX_NUM_INT) - 1);
			if (reg) {
				struct sdio_cmd52 cmd;

				cmd.read_write = 1;
				cmd.function = 0;
				cmd.raw = 0;
				cmd.address = 0xfe;
				cmd.data = reg;

				ret = wilc_sdio_cmd52(wilc->sdio, 0, &cmd);
				if (ret) {
					slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] Failed cmd52, set 0xf8 data ...\n", __func__);
					goto fail;
				}
			}
		}
		/* select VMM table 0 */
		if (val & SEL_VMM_TBL0)
			reg |= BIT(0);
		/* select VMM table 1 */
		if (val & SEL_VMM_TBL1)
			reg |= BIT(1);
		/* enable VMM */
		if (val & EN_VMM)
			reg |= BIT(2);

		if (reg) {
			struct sdio_cmd52 cmd;

			cmd.read_write = 1;
			cmd.function = 0;
			cmd.raw = 0;
			cmd.address = 0xf1;
			cmd.data = reg;

			ret = wilc_sdio_cmd52(wilc->sdio, 0, &cmd);
			if (ret) {
				slogf(_SLOGC_NETWORK, _SLOG_ERROR,"[%s] Failed cmd52, set 0xf6 data ...\n", __func__);
				goto fail;
			}
		}
	}

	return 1;
fail:
	return 0;
}

static int sdio_sync_ext(struct wilc_dev *wilc, int nint)
{
	return 1;
}

static int wilc_sdio_enable_interrupt(struct wilc_dev *nic)
{
	return 0;
}

void wilc_sdio_disable_interrupt(struct wilc_dev *nic)
{
	return 0;
}

/* Global sdio HIF function table */
static const struct wilc_hif_func wilc_hif_sdio = {
	.hif_init = sdio_drv_init,
	.hif_deinit = sdio_deinit,
	.hif_read_reg = sdio_read_reg,
	.hif_write_reg = sdio_write_reg,
	.hif_block_rx = sdio_read,
	.hif_block_tx = sdio_write,
	.hif_read_int = sdio_read_int,
	.hif_clear_int_ext = sdio_clear_int_ext,
	.hif_read_size = sdio_read_size,
	.hif_block_tx_ext = sdio_write,
	.hif_block_rx_ext = sdio_read,
	.hif_sync_ext = sdio_sync_ext,
	.enable_interrupt = wilc_sdio_enable_interrupt,
	.disable_interrupt = wilc_sdio_disable_interrupt,
	.hif_reset = wilc_sdio_reset,
	.hif_is_init = sdio_is_init,
};
