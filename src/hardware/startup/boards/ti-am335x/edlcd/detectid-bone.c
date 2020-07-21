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
#include <stdlib.h>
#include <arm/omap.h>
#include <arm/am335x.h>
#include "board.h"
#include "beaglebone.h"
#include <hw/omap_i2c.h>

int bdid_read(omap_i2c_dev_t *dev, int address, BDIDENT *bdident, int showerr)
{
    uint8_t        val[32]= { 0, 0 };
    int            rr;

    memset(val, 0, 32);

    /* Read reply from Board ID EEPROM */
    rr = omap_i2c_read(dev, 0, 2, &val[0], 32); 

    if(!rr)
    {
        memcpy(&bdident->header , &val[0]                         , AM335X_BDID_HEADER_LEN);
        memcpy(&bdident->bdname , &val[AM335X_BDID_BDNAME_OFFSET] , AM335X_BDID_BDNAME_LEN);
        bdident->bdname[AM335X_BDID_BDNAME_LEN] = 0;
        return (0);
    }
    return -1;
}

void iddump(BDIDENT *bdident)
{
    kprintf("header:  %x\n", bdident->header);
    kprintf("name:    %s\n", bdident->bdname);
}

void dump(unsigned int address, BDIDENT *bdident)
{
    kprintf("\n__________Board ID__________\n");

    if (bdident->header == 0xee3355aa)
    {
        iddump(bdident);
    }    
    else
    {        
        kprintf("Invalid header [%x], expected %x\n", bdident->header, 0xee3355aa);
    }    
    kprintf("____________________________\n", address);
}
    
void get_boardid_i2c(BEAGLEBONE_ID *boneid)
{    
    int             r;    
    int             i;    
    BDIDENT         bd;    

    if (boneid==NULL)    
    {        
        kprintf("Parameter NULL: can't return board id information\n");        
        return;    
    }    
    
    memset(boneid, 0, sizeof(BEAGLEBONE_ID));    
    boneid->basebd_type     = bb_not_detected;    
    
    for (i=0; i<AM335X_I2C0_MAXCAPES; i++)    
    {        
        boneid->cape_type[i] = ct_not_detected;    
    }    
    
   	omap_i2c_dev_t beaglebone_dev = {
		.base = AM335X_I2C0_BASE,
		.clock = AM335X_I2C0_EEPROM_SPEED,
		.slave = AM335X_I2C0_BBID
	};

    omap_i2c_init(&beaglebone_dev);    
    
    r = bdid_read(&beaglebone_dev, 0x50 , &bd, 1); 
    
    if (!r)    
    {        
        dump(0x50, &bd);        
        if (strcmp(bd.bdname, "A335BNLT")==0)
        {
            boneid->basebd_type = bb_BeagleBoneBlack;
        }
        else if (strcmp(bd.bdname, "A335BONE")==0)
        {
            boneid->basebd_type = bb_BeagleBone;
        }
        else
        {                     
            boneid->basebd_type = bb_unknown;        
            kprintf("boardid=%s\n",bd.bdname);        
        }    
    }    
        /* this section is to detect Cape */
#if 0    
    for (i=0; i<AM335X_I2C0_MAXCAPES; i++)    
    {        
        r = bdid_read(AM335X_I2C0_CAPE0+i, &bd, 0);
        if (!r)        
        {            
            dump(0x51, &bd);
            boneid->cape_type[i] = ct_unknown;    
        }    
    }
#endif

    if (boneid->basebd_type == bb_BeagleBone)
    {
        kprintf("    BeagleBone (original) detected    \n\n");    
    }
    else if (boneid->basebd_type == bb_BeagleBoneBlack)
    {
        kprintf("    BeagleBone Black detected    \n\n");
    }
    else
        kprintf("    Not a BeagleBone??    \n\n");        

    /* this section is to detect Cape */
#if 0    
    for (i=0; i<AM335X_I2C0_MAXCAPES; i++)    
    {        
        if (boneid->cape_type[i] == ct_unknown)    
            kprintf("Cape #%d detected\n", i);    
    }
#endif

}
int  set_pmic_vdd1_voltage(void){
#define TPS65910_VDD1				0x22
#define	TPS65910_DEVCTRL_REG		0x3F

#define TPS65910_DEVCTRL_REG_SR_CTL_I2C_SEL_CTL_I2C	(0x1 << 4)

#define TPS65910_OP_REG_CMD_MASK			(0x1 << 7)

#define TPS65910_OP_REG_SEL_MASK			(0x7F)
#define TPS65910_OP_REG_SEL_0_9_5			(0x1F)	/* 0.9500 V */
#define TPS65910_OP_REG_SEL_1_1_3			(0x2E)	/* 1.1375 V */
#define TPS65910_OP_REG_SEL_1_2_0			(0x33)	/* 1.2000 V */
#define TPS65910_OP_REG_SEL_1_2_6			(0x38)	/* 1.2625 V */
#define TPS65910_OP_REG_SEL_1_3_2_5			(0x3D)	/* 1.3250 V */

	int            rr;
	uint8_t reg_value = 0;
	omap_i2c_dev_t beaglebone_dev = {
		.base = AM335X_I2C0_BASE,
		.clock = AM335X_I2C0_PMIC_SPEED,
		.slave = AM335X_I2C0_PMIC
	};
   	uint8_t vdd1_buf[2] = {TPS65910_VDD1,0x38};
    //kprintf("Run set_pmic_vdd1_voltage()!\n");
   	omap_i2c_init(&beaglebone_dev);
    /* read the PMIC dev ctrl reg */
    rr   = omap_i2c_read(&beaglebone_dev,TPS65910_DEVCTRL_REG, 2, &reg_value ,1);
    if(rr)
    {
        return -1;
    }
    //kprintf("Get_pmic_dev_ctrl_reg(0x%x)!\n",reg_value);
    reg_value |= TPS65910_DEVCTRL_REG_SR_CTL_I2C_SEL_CTL_I2C;
    vdd1_buf[0] = TPS65910_DEVCTRL_REG;
    vdd1_buf[1] = reg_value;
    /* Set PMIC dev ctrl regist*/
     rr   = omap_i2c_write(&beaglebone_dev, vdd1_buf , sizeof(vdd1_buf));
     if(rr)
     {
         return -1;
     }

     rr   = omap_i2c_read(&beaglebone_dev,TPS65910_DEVCTRL_REG, 1, &reg_value ,1);
     if(rr)
     {
         return -1;
     }
     //kprintf("Af set Get_pmic_dev_ctrl_reg(0x%x)!\n",reg_value);
    /* read the vdd1 reg */
    rr   = omap_i2c_read(&beaglebone_dev,TPS65910_VDD1, 1, &reg_value ,1);
    if(rr)
    {
        return -1;
    }
    //kprintf("Get_pmic_vdd1_reg(0x%x)!\n",reg_value);
    reg_value &= ~(TPS65910_OP_REG_CMD_MASK);
    vdd1_buf[0] = TPS65910_VDD1;
    vdd1_buf[1] = reg_value;
    /* VDD1_OP_REG voltage is applied*/
    rr   = omap_i2c_write(&beaglebone_dev, vdd1_buf , sizeof(vdd1_buf));
    if(rr)
    {
        return -1;
    }
    reg_value &= ~(TPS65910_OP_REG_SEL_MASK);
    reg_value |= TPS65910_OP_REG_SEL_1_2_0;
    vdd1_buf[1] = reg_value;
    /* Set the vdd1 out to 1.200 V */
    rr   = omap_i2c_write(&beaglebone_dev, vdd1_buf , sizeof(vdd1_buf));
    if(rr)
    {
        return -1;
    }

    /* read the vdd1 reg */
    rr   = omap_i2c_read(&beaglebone_dev,TPS65910_VDD1, 1, &reg_value ,1);
    if(rr)
    {
        return -1;
    }
    if((reg_value & TPS65910_OP_REG_SEL_MASK) != TPS65910_OP_REG_SEL_1_2_0){
    	kprintf("Get_pmic_vdd1_reg(0x%x)!\n",reg_value);
    	kprintf("Run set_pmic_vdd1_voltage(1.200 V) failed!\n");
    	return -1;
    }
	return (0);
}
unsigned long Detect_PLL_Config(uint32_t CLKIN, uint32_t cm_clkmode, uint32_t cm_clksel, uint32_t cm_div_m2)
{
    uint32_t N,M,M2,mult,freq,clkmode,clksel,div_m2;

    clkmode = in32(cm_clkmode);
    clksel  = in32(cm_clksel);
    div_m2  = in32(cm_div_m2);

    M  = (clksel>>8) & 0x7FF;        /* Multiplier bits 18-8 */
    N  = (clksel>>0) & 0x07F;        /* Divisor bits 6-0 */
    M2 = (div_m2>>0) & 0x1F;         /* Post-divisor bits 4-0 */
    mult = (CLKIN * M) / (N + 1);
    freq = mult / M2;
    return (freq*1000*1000);
}
unsigned long detect_frequency_using_power()
{    
    uint8_t        val[40]= { 0, 0 };    
    int            rr;
    unsigned long retval = AM335X_MPUFREQ_500M; // Default value assumes USB Powered    

   	omap_i2c_dev_t beaglebone_dev = {
		.base = AM335X_I2C0_BASE,
		.clock = AM335X_I2C0_PMIC_SPEED,
		.slave = AM335X_I2C0_PMIC
	};

    omap_i2c_init(&beaglebone_dev);    
    
    /* Write request to Power Chip */    
    rr   = omap_i2c_read(&beaglebone_dev, 0 , 2, &val[0], 32);        

    if(!rr)
    {
        /* By experiment, this bit is set when BeagleBoard powered by 5V,
        * and clear when powered by USB 
        */
        if (val[9] & 0x8)     
        {        
            retval = AM335X_MPUFREQ_720M;  
        }      
        return retval;
    }
    return -1;
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn/product/branches/6.6.0/trunk/hardware/startup/boards/ti-am335x/beaglebone/detectid-bone.c $ $Rev: 722974 $")
#endif
