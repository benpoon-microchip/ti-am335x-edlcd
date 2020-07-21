/*
 * $QNXLicenseC:
 * Copyright 2010, QNX Software Systems.
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


#include "startup.h"
#include <arm/am335x.h>
#include "am335x_pinmux.h"
#include "beaglebone.h"
#include "am335x_pwm.h"

static void init_nfc_pin_mux(void);
static void init_pwm_pin_mux(void);
static void init_backlight(void);
static void init_mmc2_pin_mux(void);
static void init_spi0_pin_mux(void);
static void init_mmc1_pin_mux(void);

static void init_touch_pin_mux(void){
	out32(conf_ecap0_in_pwm0_out , (MODE(4) | RXACTIVE ));    						/* spi1_sclk  */
	out32(conf_uart0_ctsn        , (MODE(4) | PULLUDEN | PULLUP_EN | RXACTIVE));    /* spi1_d0  */
	out32(conf_uart0_rtsn        , (MODE(4) | PULLUDEN | PULLUP_EN | RXACTIVE));    /* spi1_d1  */
	out32(conf_xdma_event_intr0  , (MODE(4) | RXACTIVE ));    						/* spi1_cs1  */
	//kprintf("Touch SPI1\n");
}
#if BoardType == EDLCD
	static void init_mcasp0_pin_mux(void)
	{
		out32(conf_mcasp0_ahclkx    , MODE(0) );              /* MCASP0_MCLK */
		out32(conf_mcasp0_aclkx     , MODE(0) );              /* MCASP0_ACLKX */
	    out32(conf_mcasp0_fsx       , MODE(0) );              /* MCASP0_FSX   */
	    out32(conf_mcasp0_axr0      , MODE(0) );              /* MCASP0_AXR0  */
	    //out32(conf_mcasp0_axr1      , MODE(0) | RXACTIVE);    /* MCASP0_AXR1  */ Not used
	}
#endif
static void init_buzzer(void){
#if BoardType == EDLCD
	uint32_t val;
	out32(conf_mcasp0_ahclkr   ,  MODE(4));    /* eCAP2_in_PWM2_out for buzzer*/
    val = in32(PWMSSCTRL_BASE);
    val |= PWMSS2_TBCLK_EN;
    out32(PWMSSCTRL_BASE, val);

    out32(conf_mcasp0_aclkr         ,  MODE(7));     /* GPIO3_18 for buzzer control*/
#elif BoardType == EVM
    out32(conf_i2c0_scl, MODE(3)); 	//pwm1 for buzzer play

	out32(conf_mcasp0_fsx, MODE(7));/* GPIO3_15 for buzzer control*/
	//set_gpio_output(AM335X_GPIO3_BASE, 15, 0);

	out16(AM335X_EPWM1_BASE + TBPHS_OFFSET, 0);
	out16(AM335X_EPWM1_BASE + TBCNT_OFFSET, 0);
	out16(AM335X_EPWM1_BASE + TBCTL_OFFSET,TBCTL_CTRMODE | TBCTL_PHSEN | TBCTL_PRDLD | TBCTL_SYNCOSEL
					| TBCTL_HSPCLKDIV | TBCTL_CLKDIV);
	out16(AM335X_EPWM1_BASE + CMPCTL_OFFSET,
	CMPCTL_SHDWAMODE | CMPCTL_SHDWBMODE | CMPCTL_LOADAMODE | CMPCTL_LOADBMODE);
	out16(AM335X_EPWM1_BASE + AQCTLA_OFFSET, AQCTLA_ZRO | AQCTLA_CAU);
	out16(AM335X_EPWM1_BASE + CMPA_OFFSET, 0);
	out16(AM335X_EPWM1_BASE + TBPRD_OFFSET, 1999);

	out32(pwmss_ctrl, pwmss1_tbclken_en);
#elif BoardType == WT335x
	uint32_t val;
	out32(conf_gpmc_a2   ,  MODE(6));    /* eCAP2_in_PWM2_out for buzzer*/
    val = in32(PWMSSCTRL_BASE);
    val |= PWMSS1_TBCLK_EN;
    out32(PWMSSCTRL_BASE, val);

	out32(conf_gpmc_a3, MODE(7) | PULLUP_EN | PULLUDEN); //GPIO1_19
#endif
}

static void init_backlight(void){
#if BoardType == EVM
	kprintf("Init pwm backlight for Evm board!\n");
	uint32_t val;
    out32(conf_ecap0_in_pwm0_out, MODE(0));         /* ecap0_in_pwm0_out(GPIO_0_7) for backlight*/

    val = in32(PWMSSCTRL_BASE);
    val |= PWMSS0_TBCLK_EN;
    out32(PWMSSCTRL_BASE, val);
    /*Below no need the clock set*/
    out32(pwmss_ctrl,pwmss0_tbclken_dis);
	out16(AM335X_ECAP0_BASE + ECTRPHS_OFFSET, 0);
	out16(AM335X_ECAP0_BASE + ECCTL2_OFFSET,ECCTL2_CAP_APWM | ECCTL2_APWMPOL | ECCTL2_SYNCO_SEL
					| ECCTL2_SYNCI_EN | ECCTL2_TSCTRSTOP);
	out16(AM335X_ECAP0_BASE + CAP1_OFFSET, 1000);
	out16(AM335X_ECAP0_BASE + CAP2_OFFSET, 2000);
    out32(pwmss_ctrl, pwmss0_tbclken_en);
#elif BoardType == EDLCD
    out32(conf_spi0_sclk         ,  MODE(3));                          /* [P8 28] ePWM0 for backlight*/
	//EPWM0 backlight
	out32(pwmss_ctrl,pwmss0_tbclken_dis);
	out16(AM335X_EPWM0_BASE + TBPHS_OFFSET, 0);
	out16(AM335X_EPWM0_BASE + TBCNT_OFFSET, 0);
	out16(AM335X_EPWM0_BASE + TBCTL_OFFSET,
			TBCTL_CTRMODE | TBCTL_PHSEN | TBCTL_PRDLD | TBCTL_SYNCOSEL
					| TBCTL_HSPCLKDIV | TBCTL_CLKDIV);
	out16(AM335X_EPWM0_BASE + CMPCTL_OFFSET,
					CMPCTL_SHDWAMODE | CMPCTL_SHDWBMODE | CMPCTL_LOADAMODE | CMPCTL_LOADBMODE);
	out16(AM335X_EPWM0_BASE + AQCTLA_OFFSET, AQCTLA_ZRO | AQCTLA_CAU);
	out16(AM335X_EPWM0_BASE + CMPA_OFFSET, 0);
	out16(AM335X_EPWM0_BASE + TBPRD_OFFSET, 1999);

	out32(pwmss_ctrl, pwmss0_tbclken_en);
#elif BoardType == WT335x
	out32(conf_mcasp0_aclkx         ,  MODE(1));
	//EPWM0 backlight
	out32(pwmss_ctrl,pwmss0_tbclken_dis);
	out16(AM335X_EPWM0_BASE + TBPHS_OFFSET, 0);
	out16(AM335X_EPWM0_BASE + TBCNT_OFFSET, 0);
	out16(AM335X_EPWM0_BASE + TBCTL_OFFSET,
			TBCTL_CTRMODE | TBCTL_PHSEN | TBCTL_PRDLD | TBCTL_SYNCOSEL
					| TBCTL_HSPCLKDIV | TBCTL_CLKDIV);
	out16(AM335X_EPWM0_BASE + CMPCTL_OFFSET,
					CMPCTL_SHDWAMODE | CMPCTL_SHDWBMODE | CMPCTL_LOADAMODE | CMPCTL_LOADBMODE);
	out16(AM335X_EPWM0_BASE + AQCTLA_OFFSET, AQCTLA_ZRO | AQCTLA_CAU);
	out16(AM335X_EPWM0_BASE + CMPA_OFFSET, 0);
	out16(AM335X_EPWM0_BASE + TBPRD_OFFSET, 1999);

	out32(pwmss_ctrl, pwmss0_tbclken_en);
#endif
#ifdef EDLCD_backlight_const_on
    uint32_t r;
    out32(conf_spi0_sclk         ,  MODE(7) | PULLUP_EN);                          /* [P8 28] LCD_BIAS_En(GPIO0_2)       */
	/* Switch GPIO0_2 to output mode */
	r = in32(AM335X_GPIO0_BASE + GPIO_OE);
	r &= ~(1 << 2); /* set 16 bit low to enable output. */
	out32(AM335X_GPIO0_BASE + GPIO_OE, r);
	out32(AM335X_GPIO0_BASE + GPIO_SETDATAOUT, 1 << 2);
#endif

}

static void init_pwm_pin_mux(void) {
	kprintf("pwm\n");
#if BoardType == EVM
	//Control Module SELECT
	out32(conf_gpmc_a2, MODE(6)); 	//backlight
	out32(conf_i2c0_scl, MODE(3)); 	//buzzer

	out32(pwmss_ctrl,pwmss0_tbclken_dis | pwmss1_tbclken_dis | pwmss2_tbclken_dis);

	//EPWM2 buzzer
	/*out16(AM335X_EPWM2_BASE + TBPHS_OFFSET, 0);
	 out16(AM335X_EPWM2_BASE + TBCNT_OFFSET, 0);
	 out16(AM335X_EPWM2_BASE + TBCTL_OFFSET,
	 TBCTL_CTRMODE | TBCTL_PHSEN | TBCTL_PRDLD | TBCTL_SYNCOSEL
	 | TBCTL_HSPCLKDIV | TBCTL_CLKDIV);//(TB_DIV32<<10)
	 out16(AM335X_EPWM2_BASE + CMPCTL_OFFSET,
	 CMPCTL_SHDWAMODE | CMPCTL_SHDWBMODE | CMPCTL_LOADAMODE | CMPCTL_LOADBMODE);
	 out16(AM335X_EPWM2_BASE + AQCTLA_OFFSET, AQCTLA_ZRO | AQCTLA_CAU);
	 out16(AM335X_EPWM2_BASE + CMPA_OFFSET, 0);
	 out16(AM335X_EPWM2_BASE + TBPRD_OFFSET, 1999);

	 out16(AM335X_ECAP2_BASE + ECTRPHS_OFFSET, 0);
	 out16(AM335X_ECAP2_BASE + ECCTL2_OFFSET,
	 ECCTL2_CAP_APWM | ECCTL2_APWMPOL | ECCTL2_SYNCO_SEL
	 | ECCTL2_SYNCI_EN | ECCTL2_TSCTRSTOP);
	 out16(AM335X_ECAP2_BASE + CAP1_OFFSET, 1000);
	 out16(AM335X_ECAP2_BASE + CAP2_OFFSET, 2000);*/

	//EPWM1 backlight
	out16(AM335X_EPWM1_BASE + TBPHS_OFFSET, 0);
	out16(AM335X_EPWM1_BASE + TBCNT_OFFSET, 0);
	out16(AM335X_EPWM1_BASE + TBCTL_OFFSET,
			TBCTL_CTRMODE | TBCTL_PHSEN | TBCTL_PRDLD | TBCTL_SYNCOSEL
					| TBCTL_HSPCLKDIV | TBCTL_CLKDIV);
	out16(AM335X_EPWM1_BASE + CMPCTL_OFFSET,
	CMPCTL_SHDWAMODE | CMPCTL_SHDWBMODE | CMPCTL_LOADAMODE | CMPCTL_LOADBMODE);
	out16(AM335X_EPWM1_BASE + AQCTLA_OFFSET, AQCTLA_ZRO | AQCTLA_CAU);
	out16(AM335X_EPWM1_BASE + CMPA_OFFSET, 0);
	out16(AM335X_EPWM1_BASE + TBPRD_OFFSET, 1999);

	out16(AM335X_ECAP1_BASE + ECTRPHS_OFFSET, 0);
	out16(AM335X_ECAP1_BASE + ECCTL2_OFFSET,
			ECCTL2_CAP_APWM | ECCTL2_APWMPOL | ECCTL2_SYNCO_SEL
					| ECCTL2_SYNCI_EN | ECCTL2_TSCTRSTOP);
	out16(AM335X_ECAP1_BASE + CAP1_OFFSET, 1000);
	out16(AM335X_ECAP1_BASE + CAP2_OFFSET, 2000);

	out32(pwmss_ctrl, pwmss1_tbclken_en);

	//Control Module SELECT
	out32(conf_mcasp0_fsx, MODE(7));
	set_gpio_output(AM335X_GPIO3_BASE, 15, 0);
#else
	//out32(conf_spi0_sclk, MODE(3)); 	//backlight
#endif
}

/* Pinmux for BeagleBoard */
static void init_touch_int_pin_mux(void)
{
	//kprintf("gpio1_16 for touch irq!\n");
	out32(conf_gpmc_a0        , (MODE(7) | PULLUP_EN | RXACTIVE | PULLUDEN ));    /* GPIO1_16  */
    out32(conf_gpmc_a1        , (MODE(7) | PULLUP_EN | PULLUDEN));    			/* GPIO1_17  */
    out32(conf_gpmc_csn3        , (MODE(7) | PULLUP_EN | PULLUDEN));    		/* GPIO1_20  */
}
static void init_uart0_pin_mux(void)
{
    out32(conf_uart0_rxd        , (MODE(0) | PULLUP_EN | RXACTIVE | PULLUDEN));    /* UART0_RXD  */
    out32(conf_uart0_txd        , (MODE(0) | PULLUDEN ));                          /* UART0_TXD  */
    //out32(conf_uart0_ctsn       , (MODE(0) | RXACTIVE | PULLUDEN));                /* UART0_CTSN */
    //out32(conf_uart0_rtsn       , (MODE(0) | PULLUP_EN | PULLUDEN));               /* UART0_RTSN */
}

static void init_i2c0_pin_mux(void)
{
    out32(conf_i2c0_sda         , (MODE(0) | RXACTIVE | PULLUDEN | SLEWCTRL));     /* I2C_DATA  */
    out32(conf_i2c0_scl         , (MODE(0) | RXACTIVE | PULLUDEN | SLEWCTRL));     /* I2C_SCLK  */
}
static void init_wifi_pin_mux(void){
#if BoardType == EVM
	kprintf("Init the wifi pinmux for silex on Am335x Evm board!\n");
	uint32_t r;
	out32(conf_spi0_cs1         , (MODE(7) | RXACTIVE | PULLUP_EN));   /* GPIO0_6 wifi PWD */
    /* Switch GPIO0_6 to output mode */
    r = in32(AM335X_GPIO0_BASE + GPIO_OE);
    r &= ~(1 << 6); /* set 27 bit low to enable output. */
    out32(AM335X_GPIO0_BASE + GPIO_OE, r);

    /* set GPIO0_6 LOW-->HIGH to reset wifi */
    //out32(AM335X_GPIO0_BASE + GPIO_SETDATAOUT, ~(1 << 6));
    //delay(10);
    out32(AM335X_GPIO0_BASE + GPIO_SETDATAOUT, 1 << 6);

    out32(conf_mcasp0_aclkx   , MODE(7) | PULLUDEN | RXACTIVE);	/* GPIO3_14 wifi PWD */
    /* Switch GPIO3_14 to output mode */
    r = in32(AM335X_GPIO3_BASE + GPIO_OE);
    r &= ~(1 << 14); /* set 27 bit low to enable output. */
    out32(AM335X_GPIO3_BASE + GPIO_OE, r);

    /* set GPIO3_14 LOW */
    out32(AM335X_GPIO3_BASE + GPIO_SETDATAOUT, ~(1 << 14));
#endif
#if BoardType == EDLCD
    //kprintf("Init the wifi pinmux for silex on SDIO2!\n");
    init_mmc2_pin_mux();

	uint32_t r;
	out32(conf_gpmc_a6, (MODE(7) | PULLUP_EN));   /* GPIO1_22 wifi PWD */
    /* Switch GPIO1_22 to output mode */
    r = in32(AM335X_GPIO1_BASE + GPIO_OE);
    r &= ~(1 << 22); /* set 22 bit low to enable output. */
    out32(AM335X_GPIO1_BASE + GPIO_OE, r);

    /* set GPIO1_22 LOW-->HIGH to reset wifi */
    //out32(AM335X_GPIO1_BASE + GPIO_SETDATAOUT, ~(1 << 22));
    //delay(10);
    out32(AM335X_GPIO1_BASE + GPIO_SETDATAOUT, 1 << 22);

    out32(conf_gpmc_a7   , MODE(7) | PULLUDEN | PULLUP_EN);	/* GPIO1_23 wifi WOW */
    /* Switch GPIO1_23 to output mode */
    r = in32(AM335X_GPIO1_BASE + GPIO_OE);
    r &= ~(1 << 23); /* set 23 bit low to enable output. */
    out32(AM335X_GPIO1_BASE + GPIO_OE, r);

    /* set GPIO1_23 LOW */
    out32(AM335X_GPIO1_BASE + GPIO_SETDATAOUT, ~(1 << 23));

    /*Init CONTROL_MODULE for EDMA Cross Bar*/
    //out32(AM335X_CTRL_TPCC_EVT_MUX_16_19 , (0x1<< 16) | (0x2 << 24));
    //r = in32(AM335X_CTRL_TPCC_EVT_MUX_16_19);
    out32(AM335X_CTRL_TPCC_EVT_MUX_12_15 , (0x1<< 0) | (0x2 << 8));
    r = in32(AM335X_CTRL_TPCC_EVT_MUX_12_15);
    //kprintf("Init EDMA12-13 for wifi model on SDIO2 CTRL_TPCC_EVT_MUX_12_13:0x%x\n",r);
#endif

}
static init_miscellaneous(void){
	//
	out32(conf_mcasp0_fsx, MODE(7) | PULLUP_EN | PULLUDEN); //GPIO3_15 for SYS_LED
	set_gpio_value(AM335X_GPIO3_BASE,15,1);

	//initialize USB host power on
	out32(conf_usb1_drvvbus, MODE(7) | PULLUP_EN | PULLUDEN); //GPIO3_13
	set_gpio_value(AM335X_GPIO3_BASE,13,1);

}
static void init_mmc0_pin_mux(void)
{
    out32(conf_mmc0_dat3        , (MODE(0) | RXACTIVE | PULLUP_EN));               /* MMC0_DAT3  */
    out32(conf_mmc0_dat2        , (MODE(0) | RXACTIVE | PULLUP_EN));               /* MMC0_DAT2  */
    out32(conf_mmc0_dat1        , (MODE(0) | RXACTIVE | PULLUP_EN));               /* MMC0_DAT1  */
    out32(conf_mmc0_dat0        , (MODE(0) | RXACTIVE | PULLUP_EN));               /* MMC0_DAT0  */
    out32(conf_mmc0_clk         , (MODE(0) | RXACTIVE | PULLUP_EN));               /* MMC0_CLK   */
    out32(conf_mmc0_cmd         , (MODE(0) | RXACTIVE | PULLUP_EN));               /* MMC0_CMD   */

#if (BoardType == BBB) || (BoardType == EDLCD) || (BoardType == WT335x)
    out32(conf_spi0_cs1 , (MODE(5) | RXACTIVE | PULLUP_EN));  /* MMC0_CD */
#else
    out32(conf_mcasp0_aclkx, (MODE(4) | RXACTIVE | PULLUP_EN)); 				   /*EVB MMC0_CD*/
#endif

}

static void init_leds_pin_mux(void)
{
    out32(conf_gpmc_a5          , MODE(7));                                        /* USR0  */
    out32(conf_gpmc_a6          , MODE(7));                                        /* USR1  */
    out32(conf_gpmc_a7          , MODE(7));                                        /* USR2  */
    out32(conf_gpmc_a8          , MODE(7));                                        /* USR3  */
}
#if (BoardType == EVM) || (BoardType == EDLCD) || (BoardType == WT335x)
	static void init_rmii1_pin_mux(void) {
		uint32_t r;
#if BoardType == EDLCD || (BoardType == WT335x)
		//kprintf("rmii1 for EDLCD\n");
		out32(conf_mii1_txen, MODE(1)); //rmii1_txen
		out32(conf_mii1_txd1, MODE(1)); //rmii1_txd1
		out32(conf_mii1_txd0, MODE(1)); //rmii1_txd0
		out32(conf_rmii1_refclk, MODE(0) | RXACTIVE); //rmii1_refclk

		out32(conf_mii1_rxd1, MODE(1) | RXACTIVE); //rmii1_rxd1
		out32(conf_mii1_rxd0, MODE(1) | RXACTIVE); //rmii1_rxd0
		out32(conf_mii1_crs, MODE(1) | RXACTIVE); //rmii1_crs_dv
		out32(conf_mii1_rxerr, MODE(1) | RXACTIVE); //rmii1_rxer

		out32(conf_mdio_clk, MODE(0) | PULLUP_EN); //mdio_clk
		out32(conf_mdio_data, MODE(0) | RXACTIVE | PULLUP_EN); //mdio_data
		//add ETH1 reset
		out32(conf_mii1_rxdv , MODE(7) | PULLUP_EN | PULLUDEN); //GPIO3_4
		/* Switch GPIO3_4 to output mode */
		r = in32(AM335X_GPIO3_BASE + GPIO_OE);
		r &= ~(1 << 4); /* set 4 bit low to enable output. */
		out32(AM335X_GPIO3_BASE + GPIO_OE, r);
		out32(AM335X_GPIO3_BASE + GPIO_SETDATAOUT, in32(AM335X_GPIO3_BASE + GPIO_SETDATAOUT) & (~(1 << 4)));
		delay(10);
		out32(AM335X_GPIO3_BASE + GPIO_SETDATAOUT, in32(AM335X_GPIO3_BASE + GPIO_SETDATAOUT) | (1 << 4));

#else
		kprintf("rmii1 for EVM\n");
		out32(conf_mii1_txen, MODE(1)); //txen
		out32(conf_mii1_txd1, MODE(1)); //txd1
		out32(conf_mii1_txd0, MODE(1)); //txd0
		out32(conf_rmii1_refclk, MODE(0) | RXACTIVE); //clkin

		out32(conf_mii1_rxd1, MODE(1) | RXACTIVE); //rxd1
		out32(conf_mii1_rxd0, MODE(1) | RXACTIVE); //rxd0
		out32(conf_mii1_crs, MODE(1) | RXACTIVE); //crs_dv
		out32(conf_mii1_rxerr, MODE(1) | RXACTIVE); //rxerr

		out32(conf_mdio_clk, MODE(0) | PULLUP_EN); //mdc
		out32(conf_mdio_data, MODE(0) | RXACTIVE | PULLUP_EN); //mdio
#endif

	}
#else
	static void init_gmii1_pin_mux(void)
	{
		out32(conf_mii1_txclk       , MODE(0) | RXACTIVE);                             /* GMII1_TXCLK  */
		out32(conf_mii1_txd0        , MODE(0));                                        /* GMII1_TXD0   */
		out32(conf_mii1_txd1        , MODE(0));                                        /* GMII1_TXD1   */
		out32(conf_mii1_txd2        , MODE(0));                                        /* GMII1_TXD2   */
		out32(conf_mii1_txd3        , MODE(0));                                        /* GMII1_TXD3   */
		out32(conf_mii1_txen        , MODE(0));                                        /* GMII1_TXEN   */
		out32(conf_mii1_crs         , MODE(0));                                        /* GMII1_CRS    */
		out32(conf_mii1_col         , MODE(0));                                        /* GMII1_COL    */
		out32(conf_mii1_rxclk       , MODE(0) | RXACTIVE);                             /* GMII1_RXCLK  */
		out32(conf_mii1_rxd0        , MODE(0) | RXACTIVE);                             /* GMII1_RXD0   */
		out32(conf_mii1_rxd1        , MODE(0) | RXACTIVE);                             /* GMII1_RXD1   */
		out32(conf_mii1_rxd2        , MODE(0) | RXACTIVE);                             /* GMII1_RXD2   */
		out32(conf_mii1_rxd3        , MODE(0) | RXACTIVE);                             /* GMII1_RXD3   */
		out32(conf_mii1_rxerr       , MODE(0) | RXACTIVE);                             /* GMII1_RXERR  */
		out32(conf_mii1_rxdv        , MODE(0) | RXACTIVE);                             /* GMII1_RXDV   */
		out32(conf_mdio_data        , MODE(0) | RXACTIVE | PULLUP_EN);                 /* MDIO_DATA    */
		out32(conf_mdio_clk         , MODE(0) | PULLUP_EN);                            /* MDIO_CLK     */
	}
#endif

#if (BoardType == WT335x)
	static void init_rmii2_pin_mux(void) {
		uint32_t r;
		out32(conf_gpmc_a0, MODE(3)); //rmii2_txen
		out32(conf_gpmc_a4, MODE(3)); //rmii2_txd1
		out32(conf_gpmc_a5, MODE(3)); //rmii2_txd0
		out32(conf_mii1_col, MODE(1) | RXACTIVE); //rmii2_refclk

		out32(conf_gpmc_a10, MODE(3) | RXACTIVE); //rmii2_rxd1
		out32(conf_gpmc_a11, MODE(3) | RXACTIVE); //rmii2_rxd0
		out32(conf_gpmc_a9, MODE(3) | RXACTIVE); //rmii2_crs_dv
		//as describe in hardware data sheet(P24), hardware manual(P1540)
		//sma2 should be set to be compatible with rmii2_crs_dv
		out32(sma2_Section,in32(sma2_Section) | 0x1);

		out32(conf_gpmc_wpn, MODE(3) | RXACTIVE); //rmii2_rxer
		//add ETH2 reset GPIO1-17
		out32(conf_gpmc_a1, MODE(7) | PULLUP_EN | PULLUDEN); //GPIO1_17
		/* Switch GPIO1_17 to output mode */
		r = in32(AM335X_GPIO1_BASE + GPIO_OE);
		r &= ~(1 << 17); /* set 17 bit low to enable output. */
		out32(AM335X_GPIO1_BASE + GPIO_OE, r);
		out32(AM335X_GPIO1_BASE + GPIO_SETDATAOUT, in32(AM335X_GPIO1_BASE + GPIO_SETDATAOUT)&(~(1 << 17)));
		delay(10);
		out32(AM335X_GPIO1_BASE + GPIO_SETDATAOUT, in32(AM335X_GPIO1_BASE + GPIO_SETDATAOUT)|((1 << 17)));

	}
#endif
void init_pinmux()
{
	#if BoardType == BBB
		kprintf("Init the GMII pinmux for Am335x BBB board!\n");
		init_gmii1_pin_mux();
		init_touch_int_pin_mux();
		kprintf("initialize mmc1 for on-board emmc!\n");
		init_mmc1_pin_mux();
	#endif
	#if BoardType == EVM
		kprintf("Init the pinmux for Am335x Evm board!\n");
		init_wifi_pin_mux();
		init_rmii1_pin_mux();
		kprintf("Init SPI0 for touch!\n");
		init_spi0_pin_mux();
	#endif
	#if BoardType == EDLCD
		//kprintf("Init the pinmux for EDLCD Am335x board!\n");
		init_rmii1_pin_mux();
		init_touch_pin_mux();
		init_wifi_pin_mux();
		init_nfc_pin_mux();
		//init_mcasp0_pin_mux();
	#endif
#if BoardType == WT335x
		init_rmii1_pin_mux();
		init_rmii2_pin_mux();
		init_nfc_pin_mux();
		init_miscellaneous();
#endif
		init_mmc0_pin_mux();
		init_uart0_pin_mux();
		init_backlight();
		init_i2c0_pin_mux();
		init_buzzer();
		//init_leds_pin_mux();
}

/*
 **********************
 **Pinmuxes for Capes**
 **********************
 */

 /*
  * Note that pinmuxes for capes depend on what cape is used. Different capes need
  * different pinmuxes depending on what devices a cape implements (uart, lcd, etc).
  *
  * The functions below are provided as-is only. They demonstrate as an example how
  * a pinmux for a specific device on a cape could be configured. The functions provided
  * are not a complete set, and you may have to implement your own pinmux for your cape.
  * When no cape is present, a default profile of 0 is assumed.
  */

  
/*
 ****************************
 **          UART          **
 ****************************
 */

/* 
 * UART1 - Note: this multiplex choice cannot be used in conjunction with can1 or mmc1.
 * Untested - example configuration for cape uart1
 */
static void init_uart1_pin_mux(void)
{                                                                                /*  [expansion header(P8 or P9)  expansion header pin ]*/
    //out32(conf_uart1_rxd        , (MODE(0) | PULLUP_EN | RXACTIVE | PULLUDEN));  /* [P9 26] UART1_RXD  */
	out32(conf_uart1_rxd        , (MODE(0) | RXACTIVE ));  /* [P9 26] UART1_RXD  */
    out32(conf_uart1_txd        , (MODE(0) | PULLUDEN));                         /* [P9 24] UART1_TXD  */
}

/*
 * UART2 - Note: this multiplex choice cannot be used in conjunction with spi1.
 * Untested - example configuration for cape uart2
 */
static void init_uart2_pin_mux(void)
{                                                                        
#if BoardType == BBB
	out32(conf_spi0_sclk        , (MODE(1) | PULLUP_EN | RXACTIVE));              /* [P9 22] UART2_RXD  */
	out32(conf_spi0_d0          , (MODE(1) | PULLUDEN));                          /* [P9 21] UART2_TXD  */
#elif BoardType == WT335x
	out32(conf_mii1_txclk      , (MODE(1) | PULLUP_EN | RXACTIVE));              /* [P9 22] UART2_RXD  */
	out32(conf_mii1_rxclk      , (MODE(1) | PULLUDEN));
#else
	out32(conf_mii1_txclk        , (MODE(1) | PULLUP_EN | RXACTIVE));              /* [P9 22] UART2_RXD  */
	out32(conf_mii1_rxclk        ,  MODE(1) | PULLUP_EN );                             /* UART2_TXD  */
#endif

}

/*
 * UART3 - Note: this multiplex choice cannot be used in conjunction with can1 or mmc1.
 * Untested - example configuration for cape uart1
 */
static void init_uart3_pin_mux(void)
{                                                                                /*  [expansion header(P8 or P9)  expansion header pin ]*/
#if BoardType == WT335x
	out32(conf_mii1_rxd3        , (MODE(1) | PULLUP_EN | RXACTIVE));              /* [P9 22] UART2_RXD  */
	out32(conf_mii1_rxd2        , (MODE(1) | PULLUDEN));
#else
	//out32(conf_uart1_rxd        , (MODE(0) | PULLUP_EN | RXACTIVE | PULLUDEN));  /* [P9 26] UART1_RXD  */
	out32(conf_uart1_rxd        , (MODE(0) | RXACTIVE ));  /* [P9 26] UART1_RXD  */
    out32(conf_uart1_txd        , (MODE(0) | PULLUDEN));                        /* [P9 24] UART1_TXD  */
#endif
}

/* UART4 */
static void init_uart4_pin_mux(void)
{                                                                        
#if BoardType == EDLCD
	out32(conf_mii1_txd3        , (MODE(3) | PULLUP_EN | RXACTIVE));              /* [P9 22] UART2_RXD  */
	out32(conf_mii1_txd2        ,  MODE(3) | PULLUP_EN );
#elif BoardType == WT335x
	out32(conf_mii1_txd3       , (MODE(3) | PULLUP_EN | RXACTIVE));              /* [P9 22] UART2_RXD  */
	out32(conf_mii1_txd2        ,  MODE(3) | PULLUP_EN );
#else
	out32(conf_gpmc_wait0       , (MODE(6) | PULLUP_EN | RXACTIVE));             /* [P9 11] UART4_RXD  */
    out32(conf_gpmc_wpn         , (MODE(6) | PULLUDEN));  /* [P9 13] UART4_TXD  */
#endif
}

/* UART5 */
static void init_uart5_pin_mux(void)
{
#if BoardType == EDLCD
	out32(conf_mii1_col        , (MODE(3) | PULLUP_EN | RXACTIVE));              /* [P9 22] UART2_RXD  */
	out32(conf_mii1_rxdv       ,  MODE(3) | PULLUP_EN );
#elif BoardType == BBB
	out32(conf_gpmc_wait0      , (MODE(6) | PULLUP_EN | RXACTIVE));             /* [P9 11] UART4_RXD  */
    out32(conf_gpmc_wpn        , (MODE(6) | PULLUDEN));  /* [P9 13] UART4_TXD  */
#else

#endif
}

/*
 ****************************
 **          I2C           **
 ****************************
 */

/*
 * I2C1 - Note: this multiplex choice cannot be used in conjunction with spi0.
 * Untested - example configuration for cape i2c1
 */

static void init_i2c1_pin_mux(void)
{                                                                               
    out32(conf_spi0_d1          , (MODE(2) | RXACTIVE | PULLUDEN | SLEWCTRL));    /* [P9 18] I2C1_DATA  */
    out32(conf_spi0_cs0         , (MODE(2) | RXACTIVE | PULLUDEN | SLEWCTRL));    /* [P9 17] I2C1_SCLK  */
}

/* I2C2 */
static void init_i2c2_pin_mux(void)
{                                                                                
    out32(conf_uart1_ctsn       , (MODE(3) | RXACTIVE | PULLUDEN | SLEWCTRL));    /* [P9 20] I2C2_DATA  */
    out32(conf_uart1_rtsn       , (MODE(3) | RXACTIVE | PULLUDEN | SLEWCTRL));    /* [P9 19] I2C2_SCLK  */
}

/*
 ****************************
 **          SPI           **
 ****************************
 */

/*
 * SPI0 - Note: this multiplex choice cannot be used in conjunction with i2c1 and uart2.
 * Untested - example configuration for cape spi0
 */


static void init_spi0_pin_mux(void)
{                                                                                
    out32(conf_spi0_sclk        , MODE(0) | PULLUDEN | RXACTIVE);                /* [P9 22] SPI0_SCLK  */
    out32(conf_spi0_d0          , MODE(0) | PULLUDEN | PULLUP_EN | RXACTIVE);    /* [P9 21] SPI0_D0    */
    out32(conf_spi0_d1          , MODE(0) | PULLUDEN | RXACTIVE);                /* [P9 18] SPI0_D1    */
    //out32(conf_spi0_cs0       , MODE(0) | PULLUDEN | PULLUP_EN | RXACTIVE);    /* [P9 17] SPI0_CS0   */
    //out32(conf_spi0_cs1       , MODE(0) | PULLUDEN | PULLUP_EN | RXACTIVE);    /* [P9 17] SPI0_CS1   */
}


/*
 * SPI1 - Note: this multiplex choice cannot be used in conjunction with HDMI.
 */
static void init_spi1_pin_mux(void)
{                                                                                
    out32(conf_mcasp0_aclkx     , MODE(3) | PULLUDEN | RXACTIVE);                /* [P9 31] SPI1_SCLK  */
    out32(conf_mcasp0_fsx       , MODE(3) | PULLUDEN | PULLUP_EN | RXACTIVE);    /* [P9 29] SPI1_D0    */
    out32(conf_mcasp0_axr0      , MODE(3) | PULLUDEN | RXACTIVE);                /* [P9 30] SPI1_D1    */
    out32(conf_mcasp0_ahclkr    , MODE(3) | PULLUDEN | PULLUP_EN | RXACTIVE);    /* [P9 28] SPI1_CS0   */
}


/*
 ****************************
 **          MMC           **
 ****************************
 */

/*
 * MMC1 - Note: this multiplex choice cannot be used in conjunction with uart1  or uart 4.
 * Untested - example configuration for cape mmc1
 */
static void init_mmc1_pin_mux(void)
{                                                                        
    out32(conf_gpmc_ad3         , (MODE(1) | RXACTIVE));                         /* [P8  6] MMC1_DAT3  */
    out32(conf_gpmc_ad2         , (MODE(1) | RXACTIVE));                         /* [P8  5] MMC1_DAT2  */
    out32(conf_gpmc_ad1         , (MODE(1) | RXACTIVE));                         /* [P8 24] MMC1_DAT1  */
    out32(conf_gpmc_ad0         , (MODE(1) | RXACTIVE));                         /* [P8 25] MMC1_DAT0  */
    out32(conf_gpmc_csn1        , (MODE(2) | RXACTIVE | PULLUP_EN));             /* [P8 21] MMC1_CLK   */
    out32(conf_gpmc_csn2        , (MODE(2) | RXACTIVE | PULLUP_EN));             /* [P8 20] MMC1_CMD   */
    out32(conf_uart1_rxd        , (MODE(1) | RXACTIVE | PULLUP_EN));             /* [P9 26] MMC1_WP    */
    out32(conf_gpmc_wait0       , (MODE(4) | RXACTIVE));                         /* [P9 11] MMC1_CD    */
}

/*
 ****************************
 **          MMC           **
 ****************************
 */

/*
 * MMC2 - Note: this multiplex choice cannot be used in conjunction with uart1  or uart 4.
 * Untested - example configuration for cape mmc1
 */
/* 0x44E10844 0x33 DATA0*/
/* 0x44E10848 0x33 DATA1*/
/* 0x44E1084C 0x33 DATA2*/
/* 0x44E10878 0x33 DATA3*/
static void init_mmc2_pin_mux(void)
{
    out32(conf_gpmc_clk         , (MODE(3) | RXACTIVE | PULLUP_EN));                /* [P8  6] MMC2_CLK  */
    out32(conf_gpmc_csn3        , (MODE(3) | RXACTIVE | PULLUP_EN));                /* [P8  5] MMC2_CMD  */
    out32(conf_gpmc_a1         	, (MODE(3) | RXACTIVE | PULLUP_EN));                /* [P8 24] MMC2_DAT0  */
    out32(conf_gpmc_a2         	, (MODE(3) | RXACTIVE | PULLUP_EN));                /* [P8 25] MMC2_DAT1  */
    out32(conf_gpmc_a3        	, (MODE(3) | RXACTIVE | PULLUP_EN));             	/* [P8 21] MMC2_DAT2   */
    out32(conf_gpmc_be1n        , (MODE(3) | RXACTIVE | PULLUP_EN));             	/* [P8 20] MMC2_DAT2   */
    //out32(conf_uart1_rxd        , (MODE(1) | RXACTIVE | PULLUP_EN));             	/* [P9 26] MMC1_WP    */
    //out32(conf_gpmc_wait0       , (MODE(4) | RXACTIVE));                         	/* [P9 11] MMC1_CD    */
}

/*
 ****************************
 **          LCD           **
 ****************************
 */

/*
 * LCD - Note: this multiplex choice cannot be used in conjunction with hdmi.
 * Untested - example configuration for cape LCD
 */

 static void init_lcd_pin_mux(void)
{                                                            
    out32(conf_lcd_data0        , MODE(0));                                      /* [P8 45] LCD_DATA0      */
    out32(conf_lcd_data1        , MODE(0));                                      /* [P8 46] LCD_DATA1      */
    out32(conf_lcd_data2        , MODE(0));                                      /* [P8 43] LCD_DATA2      */
    out32(conf_lcd_data3        , MODE(0));                                      /* [P8 44] LCD_DATA3      */
    out32(conf_lcd_data4        , MODE(0));                                      /* [P8 41] LCD_DATA4      */
    out32(conf_lcd_data5        , MODE(0));                                      /* [P8 42] LCD_DATA5      */
    out32(conf_lcd_data6        , MODE(0));                                      /* [P8 39] LCD_DATA6      */
    out32(conf_lcd_data7        , MODE(0));                                      /* [P8 40] LCD_DATA7      */
    out32(conf_lcd_data8        , MODE(0));                                      /* [P8 37] LCD_DATA8      */
    out32(conf_lcd_data9        , MODE(0));                                      /* [P8 38] LCD_DATA9      */
    out32(conf_lcd_data10       , MODE(0));                                      /* [P8 36] LCD_DATA10     */
    out32(conf_lcd_data11       , MODE(0));                                      /* [P8 34] LCD_DATA11     */
    out32(conf_lcd_data12       , MODE(0));                                      /* [P8 35] LCD_DATA12     */
    out32(conf_lcd_data13       , MODE(0));                                      /* [P8 33] LCD_DATA13     */
    out32(conf_lcd_data14       , MODE(0));                                      /* [P8 31] LCD_DATA14     */
    out32(conf_lcd_data15       , MODE(0));                                      /* [P8 32] LCD_DATA15     */
    out32(conf_gpmc_ad15        , MODE(1));                                      /* [P8 15] LCD_DATA16     */
    out32(conf_gpmc_ad14        , MODE(1));                                      /* [P8 16] LCD_DATA17     */
    out32(conf_gpmc_ad13        , MODE(1));                                      /* [P8 11] LCD_DATA18     */
    out32(conf_gpmc_ad12        , MODE(1));                                      /* [P8 12] LCD_DATA19     */
    out32(conf_gpmc_ad11        , MODE(1));                                      /* [P8 17] LCD_DATA20     */
    out32(conf_gpmc_ad10        , MODE(1));                                      /* [P8 14] LCD_DATA21     */
    out32(conf_gpmc_ad9         , MODE(1));                                      /* [P8 13] LCD_DATA22     */
    out32(conf_gpmc_ad8         , MODE(1));                                      /* [P8 19] LCD_DATA23     */
    out32(conf_lcd_vsync        , MODE(0) | PULLUP_EN);                          /* [P8 27] LCD_VSYNC      */
    out32(conf_lcd_hsync        , MODE(0) | PULLUP_EN);                          /* [P8 29] LCD_HSYNC      */
    out32(conf_lcd_pclk         , MODE(0) | PULLUP_EN);                          /* [P8 28] LCD_PCLK       */
    out32(conf_lcd_ac_bias_en   , MODE(0) | PULLUDDIS);                         /* [P8 30] LCD_AC_BIAS_EN */
    uint32_t r;
    #if BoardType == EDLCD
    out32(conf_gpmc_a4         ,  MODE(7) | PULLUP_EN | PULLUDEN);                          /* [P8 28] LCD_STBY(GPIO1_20)       */
	/* Switch GPIO1_20 to output mode */
	r = in32(AM335X_GPIO1_BASE + GPIO_OE);
	r &= ~(1 << 20); /* set 20 bit low to enable output. */
	out32(AM335X_GPIO1_BASE + GPIO_OE, r);
	out32(AM335X_GPIO1_BASE + GPIO_SETDATAOUT, 1 << 20);

    out32(conf_gpmc_a5         ,  MODE(7) | PULLUP_EN | PULLUDEN);                          /* [P8 28] LCD_RESET(GPIO1_21)       */
	/* Switch GPIO1_21 to output mode */
	r = in32(AM335X_GPIO1_BASE + GPIO_OE);
	r &= ~(1 << 21); /* set 21 bit low to enable output. */
	out32(AM335X_GPIO1_BASE + GPIO_OE, r);
	out32(AM335X_GPIO1_BASE + GPIO_SETDATAOUT, 1 << 21);

    out32(conf_gpmc_a0         ,  MODE(7) | PULLUP_EN);                          /* [P8 28] LCD_3V3_En(GPIO1_16)       */
	/* Switch GPIO1_16 to output mode */
	r = in32(AM335X_GPIO1_BASE + GPIO_OE);
	r &= ~(1 << 16); /* set 16 bit low to enable output. */
	out32(AM335X_GPIO1_BASE + GPIO_OE, r);
	out32(AM335X_GPIO1_BASE + GPIO_SETDATAOUT, 1 << 16);

    out32(conf_mcasp0_fsr         ,  MODE(7) | PULLUP_EN);                          /* [P8 28] LCD_BIAS_En(GPIO3_19)       */
	/* Switch GPIO3_19 to output mode */
	r = in32(AM335X_GPIO3_BASE + GPIO_OE);
	r &= ~(1 << 19); /* set 16 bit low to enable output. */
	out32(AM335X_GPIO3_BASE + GPIO_OE, r);
	out32(AM335X_GPIO3_BASE + GPIO_SETDATAOUT, 1 << 19);

    out32(conf_gpmc_a11         ,  MODE(7) | PULLUP_EN);                          /* [P8 28] LCD_BIAS_En(GPIO1_27)       */
	/* Switch GPIO1_27 to output mode */
	r = in32(AM335X_GPIO1_BASE + GPIO_OE);
	r &= ~(1 << 27); /* set 16 bit low to enable output. */
	out32(AM335X_GPIO1_BASE + GPIO_OE, r);
	out32(AM335X_GPIO1_BASE + GPIO_SETDATAOUT, 1 << 27);
#elif BoardType == WT335x
	out32(conf_mcasp0_axr0, MODE(7) | PULLUP_EN | PULLUDEN); //GPIO3_16
	/* Switch GPIO3_16 to output mode */
	r = in32(AM335X_GPIO3_BASE + GPIO_OE);
	r &= ~(1 << 16); /* set 16 bit low to enable output. */
	out32(AM335X_GPIO3_BASE + GPIO_OE, r);
	out32(AM335X_GPIO3_BASE + GPIO_SETDATAOUT, 1 << 16);

	out32(conf_mcasp0_fsr, MODE(7) | PULLUP_EN | PULLUDEN);	//GPIO3_19
	/* Switch GPIO3_19 to output mode */
	r = in32(AM335X_GPIO3_BASE + GPIO_OE);
	r &= ~(1 << 19); /* set 19 bit low to enable output. */
	out32(AM335X_GPIO3_BASE + GPIO_OE, r);
	out32(AM335X_GPIO3_BASE + GPIO_SETDATAOUT, 1 << 19);
#endif

    /* backlight and/or power control */
    //out32(conf_ecap0_in_pwm0_out, MODE(7) | PULLUDDIS);                          /* [P9 42] LCD_AC_BIAS_EN */
}
/*
 ****************************
 **          HDMI          **
 ****************************
 */

/*
 * HDMI - Note: this multiplex choice cannot be used in conjunction with lcd or SPI1.
*/

 static void init_hdmi_pin_mux(void)
{                                                            
    uint32_t r;

    out32(conf_xdma_event_intr0 , MODE(3));             	/*  				 		*/
	out32(conf_lcd_data0        , MODE(0) | PULLUDDIS);		/* [P8 45] LCD_DATA0 		*/
	out32(conf_lcd_data1        , MODE(0) | PULLUDDIS);		/* [P8 46] LCD_DATA1  		*/
	out32(conf_lcd_data2        , MODE(0) | PULLUDDIS);		/* [P8 43] LCD_DATA2  		*/
	out32(conf_lcd_data3        , MODE(0) | PULLUDDIS);		/* [P8 44] LCD_DATA3  		*/
	out32(conf_lcd_data4        , MODE(0) | PULLUDDIS);		/* [P8 41] LCD_DATA4  		*/
	out32(conf_lcd_data5        , MODE(0) | PULLUDDIS);		/* [P8 42] LCD_DATA5  		*/
	out32(conf_lcd_data6        , MODE(0) | PULLUDDIS);		/* [P8 39] LCD_DATA6  		*/
	out32(conf_lcd_data7        , MODE(0) | PULLUDDIS);		/* [P8 40] LCD_DATA7  		*/
	out32(conf_lcd_data8        , MODE(0) | PULLUDDIS);		/* [P8 37] LCD_DATA8  		*/     
	out32(conf_lcd_data9        , MODE(0) | PULLUDDIS);		/* [P8 38] LCD_DATA9  		*/     
	out32(conf_lcd_data10       , MODE(0) | PULLUDDIS);		/* [P8 36] LCD_DATA10  		*/    
	out32(conf_lcd_data11       , MODE(0) | PULLUDDIS);		/* [P8 34] LCD_DATA11  		*/    
	out32(conf_lcd_data12       , MODE(0) | PULLUDDIS);		/* [P8 35] LCD_DATA12  		*/    
	out32(conf_lcd_data13       , MODE(0) | PULLUDDIS);		/* [P8 33] LCD_DATA13  		*/    
	out32(conf_lcd_data14       , MODE(0) | PULLUDDIS);		/* [P8 31] LCD_DATA14 		*/
	out32(conf_lcd_data15       , MODE(0) | PULLUDDIS);		/* [P8 32] LCD_DATA15 		*/    
	out32(conf_lcd_vsync        , MODE(0) | PULLUDDIS);		/* [P8 27] LCD_VSYNC  		*/    
	out32(conf_lcd_hsync        , MODE(0) | PULLUDDIS);		/* [P8 29] LCD_HSYNC  		*/    
	out32(conf_lcd_pclk         , MODE(0) | PULLUDDIS);		/* [P8 28] LCD_PCLK   		*/    
	out32(conf_lcd_ac_bias_en   , MODE(0) | PULLUDDIS);		/* [P8 30] LCD_AC_BIAS_EN 	*/

	/* HDMI Audio  */
    out32(conf_mcasp0_aclkx     , MODE(0) | PULLUDEN | RXACTIVE);
    out32(conf_mcasp0_fsx       , MODE(0) | PULLUDEN | RXACTIVE);
    out32(conf_mcasp0_ahclkr    , MODE(2) | PULLUDEN | RXACTIVE);                 /* McASP0_axr2 */
    out32(conf_mcasp0_ahclkx    , MODE(0) | PULLUDEN | RXACTIVE);
    out32(conf_gpmc_a11         , MODE(7) | PULLUDDIS);     /* GPIO1_27 to enable 24.576Mhz Audio clock */

    /* Switch GPIO1_27 to output mode */
    r = in32(AM335X_GPIO1_BASE + GPIO_OE);
    r &= ~(1 << 27); /* set 27 bit low to enable output. */
    out32(AM335X_GPIO1_BASE + GPIO_OE, r);

    /* set GPIO1_13 high to enable 24.576Mhz Audio clock */
    out32(AM335X_GPIO1_BASE + GPIO_SETDATAOUT, 1 << 27);
}
 static void init_nfc_pin_mux(void)
  {
  	out32(conf_gpmc_ad0         , (MODE(0) | PULLUP_EN | RXACTIVE));	/* NAND AD0 */
  	out32(conf_gpmc_ad1         , (MODE(0) | PULLUP_EN | RXACTIVE));	/* NAND AD1 */
  	out32(conf_gpmc_ad2         , (MODE(0) | PULLUP_EN | RXACTIVE));	/* NAND AD2 */
  	out32(conf_gpmc_ad3         , (MODE(0) | PULLUP_EN | RXACTIVE));	/* NAND AD3 */
  	out32(conf_gpmc_ad4         , (MODE(0) | PULLUP_EN | RXACTIVE));	/* NAND AD4 */
  	out32(conf_gpmc_ad5         , (MODE(0) | PULLUP_EN | RXACTIVE));	/* NAND AD5 */
  	out32(conf_gpmc_ad6         , (MODE(0) | PULLUP_EN | RXACTIVE));	/* NAND AD6 */
  	out32(conf_gpmc_ad7         , (MODE(0) | PULLUP_EN | RXACTIVE));	/* NAND AD7 */

  	out32(conf_gpmc_wait0       , (MODE(0) | RXACTIVE | PULLUP_EN)); /* NAND WAIT */
  	//out32(conf_gpmc_wpn         , (MODE(7) | PULLUP_EN | RXACTIVE));	/* NAND_WPN */  Pull up by hardware
  	out32(conf_gpmc_csn0        , (MODE(0) | PULLUDEN));				/* NAND_CS0 */
  	out32(conf_gpmc_advn_ale    , (MODE(0) | PULLUDEN));				/* NAND_ADV_ALE */
  	out32(conf_gpmc_oen_ren     , (MODE(0) | PULLUDEN));				/* NAND_OE */
  	out32(conf_gpmc_wen         , (MODE(0) | PULLUDEN));				/* NAND_WEN */
  	out32(conf_gpmc_be0n_cle    , (MODE(0) | PULLUDEN));				/* NAND_BE_CLE */

  	out32(GPMC_CONFIG1_0    , 0x00000800);								/* GPMC_CONFIG1 */
  	out32(GPMC_CONFIG2_0    , 0x00030300);								/* GPMC_CONFIG2 */
  	out32(GPMC_CONFIG3_0    , 0x00030300);								/* GPMC_CONFIG3 */
  	out32(GPMC_CONFIG4_0    , 0x02000000);								/* GPMC_CONFIG4 */
  	out32(GPMC_CONFIG5_0    , 0x00020404);								/* GPMC_CONFIG5 */
  	out32(GPMC_CONFIG6_0    , 0x02000000);								/* GPMC_CONFIG6 */
  	out32(GPMC_CONFIG7_0    , 0x00000f48);								/* GPMC_CONFIG7 */

  }
typedef struct cape_profile
{
	int uart1;
	int uart2;
	int uart3;
	int uart4;
	int uart5;
	int i2c1;
	int i2c2;
	int spi0;
	int spi1;
	int mmc1;
	int can0;
	int can1;
	int lcd;
	int hdmi;
} CAPE_PROFILE;

CAPE_PROFILE cape_profiles[] = {
//        uart1
//        | uart2
//        | | uart3
//        | | | uart4
//        | | | | uart5
//        | | | | | i2c1
//        | | | | | | i2c2
//        | | | | | | | spi0
//        | | | | | | | | spi1
//        | | | | | | | | | mmc1
//        | | | | | | | | | | can0
//        | | | | | | | | | | | can1
//        | | | | | | | | | | | | lcd
//        | | | | | | | | | | | | | hdmi
//        | | | | | | | | | | | | | | gpmc
        { 1,1,1,1,0,0,1,0,0,0,0,0,1,0 },	/* 00 - this is the default profile where no cape is present */
       //{ 1,0,0,1,0,1,1,0,0,1,0,0,0,1 },	/* 01 - BeagleBone Black with HDMI */
        { 1,0,0,1,0,1,1,0,0,0,0,0,1,0 },	/* 01 - BeagleBone Black with LCD */
        { 0,0,0,0,0,0,0,0,0,0,0,0,0,0 },	/* 02 - add your own profile here */
};

void init_pinmux_capes(int profile, int showchoices)
{
	// Any UARTS?
	if (cape_profiles[profile].uart1)
	{
		if (showchoices) kprintf("uart1\n");
		init_uart1_pin_mux();
	}
	if (cape_profiles[profile].uart2)
	{
		if (showchoices) kprintf("uart2\n");
		init_uart2_pin_mux();
	}
		

	/* init_uart3_pin_mux() not implemented */
	if (cape_profiles[profile].uart3)
	{
		if (showchoices) kprintf("uart3\n");
		init_uart3_pin_mux();
	}

	if (cape_profiles[profile].uart4)
	{
		if (showchoices) kprintf("uart4\n");
		init_uart4_pin_mux();
	}

	/* init_uart5_pin_mux() not implemented */
	if (cape_profiles[profile].uart5)
	{
		if (showchoices) kprintf("uart5\n");
		init_uart5_pin_mux();
	}

	// Any i2c?
	if (cape_profiles[profile].i2c1)
	{
		if (showchoices) kprintf("i2c1\n");
		init_i2c1_pin_mux();
	}
	if (cape_profiles[profile].i2c2)
	{
		if (showchoices) kprintf("i2c2\n");
		init_i2c2_pin_mux();
	}
	
	// Any SPI?

	/* init_spi0_pin_mux() not implemented */
	if (cape_profiles[profile].spi0)
	{
		if (showchoices) kprintf("spi0_cs1\n");
		init_spi0_pin_mux();
	}

	if (cape_profiles[profile].spi1)
	{
		if (showchoices) kprintf("spi1\n");
		init_spi1_pin_mux();
	}

	// MMC1?
	if (cape_profiles[profile].mmc1)
	{
		if (showchoices) kprintf("mmc1\n");
		init_mmc1_pin_mux();
	}

	
	// Any CAN?
	#if 0
	/* init_can0_pin_mux() not implemented */
	if (cape_profiles[profile].can0)
	{
		if (showchoices) kprintf("can0\n");
		init_can0_pin_mux();
	}
	#endif

	#if 0
	/* init_can1_pin_mux() not implemented */
	if (cape_profiles[profile].can1)
	{
		if (showchoices) kprintf("can1\n");
		init_can1_pin_mux();
	}
	#endif

	// LCD?
	if (cape_profiles[profile].lcd)
	{
		if (showchoices) kprintf("lcd\n");
		init_lcd_pin_mux();
	}

	// HDMI?
	if (cape_profiles[profile].hdmi)
	{
		if (showchoices) kprintf("hdmi\n");
		init_hdmi_pin_mux();
	}
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn/product/branches/6.6.0/trunk/hardware/startup/boards/ti-am335x/beaglebone/init_pinmux.c $ $Rev: 761911 $")
#endif
