/*************************************************************************
 * University of Alberta
 * AFE4490 peripheral definition file
 *************************************************************************
 * Compiler: IAR C Compiler for 8051
 * Target platform: CC2540/1
 *************************************************************************
 * Revision 1.0.0.0 2017/07/20
 * Acknowledgements: Andrew Tiberius Jowsey
 *************************************************************************/

#ifndef pulse_AFE_h
#define pulse_AFE_h

#include "hal_types.h"

/*********************************************************************
 * CONSTANTS
 */

// Basic Constants
#define AFE4490_MSG_BYTES				3

// AFE Pinouts
#define AFE4490_RESET_PIN 	0x10    // Pin 15 - P0.4 0001 0000
#define AFE4490_CS_PIN 		0x20    // Pin 14 - P0.5 0010 0000
#define AFE4490_READY_PIN 	0x40    // Pin 13 - P0.6 0100 0000
#define AFE4490_PWRDWN_PIN 	0x80    // Pin 12 - P0.7 1000 0000
   
// CONTROL0 (0x00) Register
#define AFE4490_SW_RESET                0x08
#define AFE4490_DIAG_EN                 0x04
#define AFE4490_TIMECNT_RESET           0x02
#define AFE4490_SPI_READ_EN             0x01
#define AFE4490_CONTROL0_DEFAULT        AFE4490_SPI_READ_EN

// LEDCNTRL Register (LED Control Register)
#define AFE4490_LED_CURRENT_RANGE_1	0x0 << 16
#define AFE4490_LED_CURRENT_RANGE_2	0x1 << 16
#define AFE4490_LED_CURRENT_RANGE_3	0x2 << 16
#define AFE4490_LED_CURRENT_RANGE_4	0x3 << 16

// LEDCNTRL Register (LED Control Register 2)
#define AFE4490_PDN_AFE			0x1
#define AFE4490_PDN_RX			0x1 << 1
#define AFE4490_PDN_TX			0x1 << 2
#define AFE4490_TX_REF_0V75		0x0 << 17
#define AFE4490_TX_REF_0V5		0x1 << 17
#define AFE4490_TX_REF_1V		0x2 << 17

// TIAGAIN and TIAAMBGAIN Registers ( Transimpedance Amplifier Gain Setting )
#define AFE4490_LED_TIAGAIN_RF_10K      0x5
#define AFE4490_LED_TIAGAIN_RF_25K      0x4
#define AFE4490_LED_TIAGAIN_RF_50K      0x3
#define AFE4490_LED_TIAGAIN_RF_100K     0x2
#define AFE4490_LED_TIAGAIN_RF_250K     0x1
#define AFE4490_LED_TIAGAIN_RF_500K     0x0

#define AFE4490_LED_TIAGAIN_CF_5PF	0x0 << 3
#define AFE4490_LED_TIAGAIN_CF_10PF	0x1 << 3
#define AFE4490_LED_TIAGAIN_CF_20PF	0x2 << 3
#define AFE4490_LED_TIAGAIN_CF_30PF	0x4 << 3
#define AFE4490_LED_TIAGAIN_CF_55PF	0x8 << 3
#define AFE4490_LED_TIAGAIN_CF_155PF	0x10 << 3

#define AFE4490_TIAGAIN_STG2GAIN1_0DB	0x0 << 8
#define AFE4490_TIAGAIN_STG2GAIN1_3DB5	0x1 << 8
#define AFE4490_TIAGAIN_STG2GAIN1_6DB	0x2 << 8
#define AFE4490_TIAGAIN_STG2GAIN1_9DB5	0x3 << 8
#define AFE4490_TIAGAIN_STG2GAIN1_12DB	0x4 << 8

#define AFE4490_TIAGAIN_STAGE2EN1		0x1 << 14	// Enable Stage 2 for LED 1
#define AFE4490_CONTROL1_ENSEPGAIN 		0x1 << 15
#define AFE4490_CONTROL2_FLTR_500HZ		0x0 << 15
#define AFE4490_CONTROL2_FLTR_1000HZ	0x1 << 15

// CONTROL2 Register (0x23)
#define AFE4490_CONTROL2_AFE_PWRUP		0x0
#define AFE4490_CONTROL2_AFE_PWRDWN		0x1

/*********************************************************************
 * Timing Modules Values: 500 Hz Sampling
 */
#define AFE4490_LED2STC_500HZ			0x1770
#define AFE4490_LED2ENDC_500HZ			0x1F3F
#define AFE4490_LED2LEDSTC_500HZ		0x1770
#define AFE4490_LED2LEDENDC_500HZ		0x1F3F
#define AFE4490_LED1STC_500HZ			0x7D0
#define AFE4490_LED1ENDC_500HZ			0xF9F
#define AFE4490_LED1LEDSTC_500HZ		0x7D0
#define AFE4490_LED1LEDENDC_500HZ		0xF9F
#define AFE4490_LED2CONVST_500HZ		0x6
#define AFE4490_LED2CONVEND_500HZ		0x7CF
#define AFE4490_LED1CONVST_500HZ		0xFA6
#define AFE4490_LED1CONVEND_500HZ		0x176F

#define AFE4490_LADCRSTSTCT0_500HZ	0x0
#define AFE4490_LADCRSTENDCT0_500HZ	0x5
#define AFE4490_LADCRSTSTCT1_500HZ	0x7D0
#define AFE4490_LADCRSTENDCT1_500HZ	0x7D5
#define AFE4490_LADCRSTSTCT2_500HZ	0xFA0
#define AFE4490_LADCRSTENDCT2_500HZ	0xFA5
#define AFE4490_LADCRSTSTCT3_500HZ	0x1770
#define AFE4490_LADCRSTENDCT3_500HZ	0x1775

#define AFE4490_PRPCOUNT_500HZ	0x1F3F

   
#define AFE4490_LEDCNTRL_500HZ  0x012636

/*********************************************************************
 * REGISTER ADDRESSES
 */

#define AFE4490_RA_CONTROL0		0x00
#define AFE4490_RA_LED2STC		0x01
#define AFE4490_RA_LED2ENDC		0x02
#define AFE4490_RA_LED2LEDSTC		0x03
#define AFE4490_RA_LED2LEDENDC		0x04
#define AFE4490_RA_LED1STC		0x07
#define AFE4490_RA_LED1ENDC		0x08
#define AFE4490_RA_LED1LEDSTC		0x09
#define AFE4490_RA_LED1LEDENDC		0x0A
#define AFE4490_RA_LED2CONVST		0x0D
#define AFE4490_RA_LED2CONVEND		0x0E
#define AFE4490_RA_LED1CONVST		0x11
#define AFE4490_RA_LED1CONVEND		0x12

#define AFE4490_RA_LADCRSTSTCT0		0x15
#define AFE4490_RA_LADCRSTENDCT0	0x16
#define AFE4490_RA_LADCRSTSTCT1		0x17
#define AFE4490_RA_LADCRSTENDCT1	0x18
#define AFE4490_RA_LADCRSTSTCT2		0x19
#define AFE4490_RA_LADCRSTENDCT2	0x1A
#define AFE4490_RA_LADCRSTSTCT3		0x1B
#define AFE4490_RA_LADCRSTENDCT3	0x1C

// Sets the device pulse repetition peroid count
#define AFE4490_RA_PRPCOUNT		0x1D	// Default: 0x1F3F

#define AFE4490_RA_CONTROL1		0x1E
#define AFE4490_RA_TIAGAIN		0x20
#define AFE4490_RA_TIAAMBGAIN	0x21
#define AFE4490_RA_LEDCNTRL		0x22
#define AFE4490_RA_CONTROL2		0x23

// Contains the digital value of the latest samples converted by the ADC.
#define AFE4490_RA_LED2VAL 		0x2A 
#define AFE4490_RA_ALED2VAL 		0x2B
#define AFE4490_RA_LED1VAL 		0x2C
#define AFE4490_RA_ALED1VAL 		0x2D
#define AFE4490_RA_LED2_ALED2VAL 	0x2E
#define AFE4490_RA_LED1_ALED1VAL 	0x2F
   
 /*********************************************************************
 * MACROS
 */
 
#define CS_AFE_ENABLE()         (P0_5 = 0)
#define CS_AFE_DISABLE()        (P0_5 = 1) 

/*********************************************************************
 * FUNCTION PROTOTYPES
 */
void AFEPwrUp();
void AFEInit();
void AFEPwrDownAFE();
void AFEReadSample(uint32* RedSample, uint32* IRSample);
void AFEWriteEnable();
void AFEWriteDisable();
void AFERead(uint8 address, uint8* data);
void AFEWrite(uint8 address, uint32 data);
void AFEStartSampling();
void AFEStopSampling();
void AFEConfigureForLongTerm();
#endif