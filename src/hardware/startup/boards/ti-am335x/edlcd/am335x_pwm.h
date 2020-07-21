/*
 * am335x_pwm.h
 *
 *  Created on: 2015-12-1
 *      Author: QQ
 */
#include <arm/am335x.h>
#ifndef AM335X_PWM_H_
#define AM335X_PWM_H_

/* PWM Base address  (from memory map)*/
#define AM335X_ECAP0_BASE           0x48300100
#define AM335X_ECAP1_BASE           0x48302100
#define AM335X_ECAP2_BASE           0x48304100
#define AM335X_ECAP_SIZE            0x00001000

#define AM335X_EPWM0_BASE           0x48300200
#define AM335X_EPWM1_BASE           0x48302200
#define AM335X_EPWM2_BASE           0x48304200
#define AM335X_EPWM_SIZE            0x00001000

//control module
#define pwmss_ctrl             (AM335X_CTRL_BASE+0x0664)
#define pwmss_ctrl_offset              0x0664
#define pwmss0_tbclken_dis             0x00<<0
#define pwmss1_tbclken_dis             0x00<<1
#define pwmss2_tbclken_dis             0x00<<2
#define pwmss0_tbclken_en            0x01<<0
#define pwmss1_tbclken_en            0x01<<1
#define pwmss2_tbclken_en            0x01<<2

// TBCTL (Time-Base Control)
// = = = = = = = = = = = = = = = = = = = = = = = = = =
// TBCNT MODE bits
#define TB_COUNT_UP 0x0
#define TB_COUNT_DOWN 0x1
#define TB_COUNT_UPDOWN 0x2
#define TB_FREEZE 0x3
// PHSEN bit
#define TB_DISABLE 0x0
#define TB_ENABLE 0x1
// PRDLD bit
#define TB_SHADOW 0x0
#define TB_IMMEDIATE 0x1
// SYNCOSEL bits
#define TB_SYNC_IN 0x0
#define TB_CTR_ZERO 0x1
#define TB_CTR_CMPB 0x2
#define TB_SYNC_DISABLE 0x3
// HSPCLKDIV and CLKDIV bits
#define TB_DIV1 0x0
#define TB_DIV2 0x1
#define TB_DIV4 0x2
#define TB_DIV32 0x5
// PHSDIR bit
#define TB_UP 0x1
#define TB_DOWN 0x0
// CMPCTL (Compare Control)
// = = = = = = = = = = = = = = = = = = = = = = = = = =
// LOADAMODE and LOADBMODE bits
#define CC_CTR_ZERO 0x0
#define CC_CTR_PRD 0x1
#define CC_CTR_ZERO_PRD 0x2
#define CC_LD_DISABLE 0x3
// SHDWAMODE and SHDWBMODE bits
#define CC_SHADOW 0x0
#define CC_IMMEDIATE 0x1
// AQCTLA and AQCTLB (Action-qualifier Control)
// = = = = = = = = = = = = = = = = = = = = = = = = = =
// ZRO, PRD, CAU, CAD, CBU, CBD bits
#define AQ_NO_ACTION 0x0
#define AQ_CLEAR 0x1
#define AQ_SET 0x2
#define AQ_TOGGLE 0x3
// DBCTL (Dead-Band Control)
// = = = = = = = = = = = = = = = = = = = = = = = = = =
// MODE bits
#define DB_DISABLE 0x0
#define DBA_ENABLE 0x1
#define DBB_ENABLE 0x2
#define DB_FULL_ENABLE 0x3
// POLSEL bits
#define DB_ACTV_HI 0x0
#define DB_ACTV_LOC 0x1
#define DB_ACTV_HIC 0x2
#define DB_ACTV_LO 0x3
// PCCTL (chopper control)
// = = = = = = = = = = = = = = = = = = = = = = = = = =
// CHPEN bit
#define CHP_ENABLE 0x0
#define CHP_DISABLE 0x1
// CHPFREQ bits
#define CHP_DIV1 0x0
#define CHP_DIV2 0x1
#define CHP_DIV3 0x2
#define CHP_DIV4 0x3
#define CHP_DIV5 0x4
#define CHP_DIV6 0x5
#define CHP_DIV7 0x6
#define CHP_DIV8 0x7
// CHPDUTY bits
#define CHP1_8TH 0x0
#define CHP2_8TH 0x1
#define CHP3_8TH 0x2
#define CHP4_8TH 0x3
#define CHP5_8TH 0x4
#define CHP6_8TH 0x5
#define CHP7_8TH 0x6
// TZSEL (Trip-zone Select)
// = = = = = = = = = = = = = = = = = = = = = = = = = =
// CBCn and OSHTn bits
#define TZ_ENABLE 0x0
#define TZ_DISABLE 0x1
// TZCTL (Trip-zone Control)
// = = = = = = = = = = = = = = = = = = = = = = = = = =
// TZA and TZB bits
#define TZ_HIZ 0x0
#define TZ_FORCE_HI 0x1
#define TZ_FORCE_LO 0x2
#define TZA_DISABLE 0x3
// ETSEL (Event-trigger Select)
// = = = = = = = = = = = = = = = = = = = = = = = = = =
// INTSEL, SOCASEL, SOCBSEL bits
#define ET_CTR_ZERO 0x1
#define ET_CTR_PRD 0x2
#define ET_CTRU_CMPA 0x4
#define ET_CTRD_CMPA 0x5
#define ET_CTRU_CMPB 0x6
#define ET_CTRD_CMPB 0x7
// ETPS (Event-trigger Prescale)
// = = = = = = = = = = = = = = = = = = = = = = = = = =
// INTPRD, SOCAPRD, SOCBPRD bits
#define ET_DISABLE 0x0
#define ET_1ST 0x1
#define ET_2ND 0x2
#define ET_3RD 0x3
// = = = = = = = = = = = = = = = = = = = = = = = = = =
// ECCTL1 ( ECAP Control Reg 1)
//==========================
// CAPxPOL bits
#define EC_RISING 0x0
#define EC_FALLING 0x1
// CTRRSTx bits
#define EC_ABS_MODE 0x0
#define EC_DELTA_MODE 0x1
// PRESCALE bits
#define EC_BYPASS 0x0
#define EC_DIV1 0x0
#define EC_DIV2 0x1
#define EC_DIV4 0x2
#define EC_DIV6 0x3
#define EC_DIV8 0x4
#define EC_DIV10 0x5
// ECCTL2 ( ECAP Control Reg 2)
//==========================
// CONT/ONESHOT bit
#define EC_CONTINUOUS 0x0
#define EC_ONESHOT 0x1
// STOPVALUE bit
#define EC_EVENT1 0x0
#define EC_EVENT2 0x1
#define EC_EVENT3 0x2
#define EC_EVENT4 0x3
// RE-ARM bit
#define EC_ARM 0x1
// TSCTRSTOP bit
#define EC_FREEZE 0x0
#define EC_RUN 0x1
// SYNCO_SEL bit
#define EC_SYNCIN 0x0
#define EC_CTR_PRD 0x1
#define EC_SYNCO_DIS 0x2
// CAP/APWM mode bit
#define EC_CAP_MODE 0x0
#define EC_APWM_MODE 0x1
// APWMPOL bit
#define EC_ACTV_HI 0x0
#define EC_ACTV_LO 0x1
// Generic
#define EC_DISABLE 0x0
#define EC_ENABLE 0x1
#define EC_FORCE 0x1
// = = = = = = = = = = = = = = = = = = = = = = = = = =

#define BACKLIGHT_FREQUNCY         50000//50kHZ
#define BACKLIGHT_PERIOD           20000//ns

#define AM335X_SYSCLKOUT           100000000 //100MHZ
#define AM335X_SYSCLKOUT_DIV32     3125000//     100000000/32

#define AM335X_TBCLK               AM335X_SYSCLKOUT_DIV32


// = = = = = = = = = = = = = = = = = = = = = = = = = =
// EPWM
#define TBCTL_OFFSET               0x00
#define TBCTL_CTRMODE              TB_COUNT_UP
#define TBCTL_PHSEN                TB_DISABLE<<2
#define TBCTL_PRDLD                TB_SHADOW<<3
#define TBCTL_SYNCOSEL             TB_SYNC_DISABLE<<4
#define TBCTL_HSPCLKDIV            TB_DIV1<<7
#define TBCTL_CLKDIV               TB_DIV1<<10

#define TBSTS_OFFSET               0x02
#define TBPHSHR_OFFSET             0x04
#define TBPHS_OFFSET               0x06
#define TBCNT_OFFSET               0x08
#define TBPRD_OFFSET               0x0A

#define CMPCTL_OFFSET              0x0E
#define CMPCTL_SHDWAMODE           CC_SHADOW<<4
#define CMPCTL_SHDWBMODE           CC_SHADOW<<6
#define CMPCTL_LOADBMODE           CC_CTR_ZERO<<2//(0x01<<2)//CTR=PRD
#define CMPCTL_LOADAMODE           CC_CTR_ZERO//(0x01)//CTR=PRD


#define CMPAHR_OFFSET              0x10
#define CMPA_OFFSET                0x12

#define AQCTLA_OFFSET              0x16
#define AQCTLA_CAU                 AQ_CLEAR<<4//(0x01<<4)
#define AQCTLA_ZRO                 AQ_SET//(0x02)

#define DBCTL_OFFSET               0x1E
#define DBCTL_OUT_MODE             DB_FULL_ENABLE
#define DBCTL_POSEL                DB_ACTV_HI<<2
#define DBCTL_IN_MODE              DB_FULL_ENABLE<<4

#define DBRED_OFFSET               0x20
#define DBRED_DEL                  35

#define ETSEL_OFFSET               0x32
#define ETSEL_INTEN                0x01<<3
#define ETSEL_INTSEL               0x02

#define ETPS_OFFSET                0x34
#define ETPS_INTPRD                ET_1ST

#define PWMx_CMPA          (0xFFFF)
#define PWMx_TBPRD           (0xFFFF)

// = = = = = = = = = = = = = = = = = = = = = = = = = =
//ECAP
#define TSCTR_OFFSET               0x00
#define ECTRPHS_OFFSET             0x04
#define CAP1_OFFSET                0x08
#define CAP2_OFFSET                0x0C

#define ECCTL2_OFFSET              0x2A
#define ECCTL2_CAP_APWM            EC_APWM_MODE<<9
#define ECCTL2_APWMPOL             EC_ACTV_HI<<10
#define ECCTL2_SYNCO_SEL           EC_SYNCO_DIS<<6
#define ECCTL2_SYNCI_EN            EC_DISABLE<<5
#define ECCTL2_TSCTRSTOP           EC_RUN<<4

#define ECCTL2_TSCTRSTOP_RUN       EC_RUN<<4
#define ECCTL2_TSCTRSTOP_FREEZE    EC_FREEZE<<4

  //pwmss_ctrl_reg
#define PWMSS_CTRL_OFFSET             	0x0664
#define PWMSSCTRL_SIZE             		0x4
#define PWMSSCTRL_BASE             		(AM335X_CTRL_BASE+PWMSS_CTRL_OFFSET)
#define PWMSS0_TBCLK_DIS            	0x00<<0
#define PWMSS1_TBCLK_DIS            	0x00<<1
#define PWMSS2_TBCLK_DIS            	0x00<<2
#define PWMSS0_TBCLK_EN              	0x01<<0
#define PWMSS1_TBCLK_EN              	0x01<<1
#define PWMSS2_TBCLK_EN              	0x01<<2


#endif /* AM335X_PWM_H_ */
