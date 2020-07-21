/*
 * $QNXLicenseC:
 * Copyright 2013, QNX Software Systems.
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
#include <arm/omap.h>
#include <arm/am335x.h>
#include "beaglebone.h"
#include "board.h"

#define SHOW_CAPEMUX_CHOICES            1
#define DONT_SHOW_CAPEMUX_CHOICES       0


extern struct callout_rtn   reboot_am335x;
extern void init_qtime_am335x(void);
extern void init_pinmux(void);
extern void init_edma(void);
extern void init_clocks(void);
extern void display_clocks(char *);
extern void wdt_enable(void);
extern void wdt_disable(void);
extern void display_PLLs(void);
extern void get_boardid_i2c(BEAGLEBONE_ID *boneid);
extern unsigned long detect_frequency_using_power();
extern unsigned long Detect_PLL_Config(uint32_t CLKIN, uint32_t cm_clkmode, uint32_t cm_clksel, uint32_t cm_div_m2);
extern int  set_pmic_vdd1_voltage(void);
extern void init_pinmux_capes(int profile, int showchoices);
extern void init_raminfo_beaglebone(int ram);


#define delay_cy(x) {unsigned _delay = ((x+1)>>1); while (_delay--) asm volatile (""); }  /* ~2 cycles per loop at 720MHz, x in 1/720us units (720MHz clock) */

const struct callout_slot callouts[] = {
    { CALLOUT_SLOT( reboot, _am335x ) },
};

const struct debug_device debug_devices[] = {
    {   "8250",
        {   "0x44E09000^2.0.48000000.16",   // UART0, use the baud rate set by boot loader
        },
        init_omap,
        put_omap,
        {   &display_char_8250,
            &poll_key_8250,
            &break_detect_8250,
        }
    },
};

/*
 * main()
 *  Startup program executing out of RAM
 *
 * 1. It gathers information about the system and places it in a structure
 *    called the system page. The kernel references this structure to
 *    determine everything it needs to know about the system. This structure
 *    is also available to user programs (read only if protection is on)
 *    via _syspage->.
 *
 * 2. It (optionally) turns on the MMU and starts the next program
 *    in the image file system.
 */
int
main(int argc, char **argv, char **envv)
{
    //BEAGLEBONE_ID    boneid;
    int opt;
    int ram = 256;
    add_callout_array(callouts, sizeof(callouts));

    while ((opt = getopt(argc, argv, COMMON_OPTIONS_STRING "dm:")) != -1) {
        switch (opt) {
            case 'd':
                /* Enable WDT */
                wdt_enable();
                break;
            case 'm':
                /* Specify the RAM info */
            	ram = strtoul(optarg, NULL, 0);
                break;
            default:
                handle_common_option(opt);
                break;
        }
    }
    
    /*
     * Initialize debugging output
     */
    select_debug(debug_devices, sizeof(debug_devices));

    /*
     * Collect information on all free RAM in the system
     */

    init_clocks();

    init_pinmux();
    /*
     * Set CPU frequency
     */

    if (cpu_freq == 0)
    {
        //cpu_freq = detect_frequency_using_power();
    	//set_pmic_vdd1_voltage();
    	cpu_freq = Detect_PLL_Config(24, AM335X_CM_CLKMODE_DPLL_MPU , AM335X_CM_CLKSEL_DPLL_MPU , AM335X_CM_DIV_M2_DPLL_MPU );
    	//kprintf("CPU freq was set at %d MHz\n",cpu_freq/1000000);
    }
    //get_boardid_i2c(&boneid);
    //boneid.basebd_type = bb_BeagleBoneBlack; //for 512MB DRAM
    //boneid.basebd_type = bb_BeagleBone;	//for 256MB DRAM
    // This BSP assumes no capes. If there are any capes, select a profile
    // for init_pinmux_capes and implement the profile for your cape. Some
    // example code is in init_pinmux.c to start with, and replace '0' with
    // your profile number (likely to be profile '1'). Profile '0' is the
    // default profile, eg no cape present, and initialises the pinmux for
    // uart1-5, spi1, and i2c1-2 to be available on the cape connectors.
//#define DBG 1
#if DBG
    init_pinmux_capes(0 /*profile*/, SHOW_CAPEMUX_CHOICES);
#else
    init_pinmux_capes(0 /*profile*/, DONT_SHOW_CAPEMUX_CHOICES);
#endif

    init_raminfo_beaglebone(ram);

    /* 
     * Remove RAM used by modules in the image
     */
    alloc_ram(shdr->ram_paddr, shdr->ram_size, 1);

    if (shdr->flags1 & STARTUP_HDR_FLAGS1_VIRTUAL)
    {
        init_mmu();
    }

    init_intrinfo();

    init_edma();

    init_qtime_am335x();

    init_cacheattr();

    init_cpuinfo();

    init_hwinfo();

    //if(debug_flag > STARTUP_DEBUG_LEVEL)
    //{
        //display_PLLs();
    //}

    add_typed_string(_CS_MACHINE, "BeagleBone");

    /*
     * Load bootstrap executables in the image file system and Initialise
     * various syspage pointers. This must be the _last_ initialisation done
     * before transferring control to the next program.
     */
    init_system_private();

    /*
     * This is handy for debugging a new version of the startup program.
     * Commenting this line out will save a great deal of code.
     */
    print_syspage();

    return 0;
}




#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn/product/branches/6.6.0/trunk/hardware/startup/boards/ti-am335x/beaglebone/main.c $ $Rev: 746391 $")
#endif
