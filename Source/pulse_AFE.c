/*************************************************************************
 * University of Alberta
 * Analog Front End Drivers for AFE4490
 *************************************************************************
 * Compiler: IAR C Compiler for 8051
 * Target platform: CC2540/1
 *************************************************************************
 * Revision 1.0.0.0 2017/07/27
 * Andrew Tiberius Jowsey
 *************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "simpleBLEPeripheral.h"
#include "OnBoard.h"
#include "pulse_spi.h"
#include "internalFlash.h"
#include "pulse_AFE.h"
#include <stdio.h>
#include <string.h>
   
/*********************************************************************
 * @fn      AFEInit
 *
 * @brief   Configure the AFE4490 to be ready for longterm Sampling
 *
 * @param   none
 *
 * @return  none
 */
void AFEInit()
{
  // Initialize SPI registers for CC2541
  InitSPI();
  
  // Get system ready for 500Hz
  AFEConfigureForLongTerm();
  
  // Powerdown until 5 min cycle begins
  AFEStopSampling();
}

/*********************************************************************
 * @fn      AFEPwrDown
 *
 * @brief   Powers down the AFE4490. There are other alternatives like
 *			just powering down the Tx and Rx but we will be using this
 *			between 5 min cycles. All registers are available through
 *			the AFE's Control 2 Register
 *
 * @param   none
 *
 * @return  none
 */
void AFEPwrDownAFE()
{
  // Set bit D0 to 1. Which powers down the AFE.
  AFEWrite(AFE4490_RA_CONTROL2,AFE4490_CONTROL2_AFE_PWRDWN);
}

/*********************************************************************
 * @fn      AFEPwrUp
 *
 * @brief   Powers up the AFE4490. Deselects the powerdown bit in
 *          CONTROL2 register.
 *
 * @param   none
 *
 * @return  none
 */
void AFEPwrUp()
{
  // Set bit D0 to 0. Which powers up the AFE.
  AFEWrite(AFE4490_RA_CONTROL2,AFE4490_CONTROL2_AFE_PWRUP);
}

/*********************************************************************
 * @fn      AFEReadSample
 *
 * @brief   Read the sample registers and save to paramters.
 *
 * @param   none
 *
 * @return  none
 */
void AFEReadSample(uint32* RedSample, uint32* IRSample)
{
  uint8 tempRedData[4];			// Red LED = LED2
  uint8 tempIrData[4];			// Infrared LED = LED1
  AFERead(AFE4490_RA_LED2VAL,tempRedData);
  AFERead(AFE4490_RA_LED1VAL,tempIrData);

  memcpy(RedSample,tempRedData, 3);
  memcpy(IRSample,tempIrData, 3);
}

/*********************************************************************
 * @fn      AFEWriteEnable
 *
 * @brief   Sets the write enable mode on the AFE4490 should always
 *          get called prior to a write operation.
 *
 * @param   none
 *
 * @return  none
 */
void AFEWriteEnable()
{
  CS_AFE_ENABLE();
  // Address, Data[2], Date[1], Data[0] (D23 - D0)
  uint8 SPIBuf[4] = {AFE4490_RA_CONTROL0,0x00,0x00,0x00};
  SPITx(4,0,SPIBuf); // All bytes discarded
  CS_AFE_DISABLE();
};

/*********************************************************************
 * @fn      AFEWriteDisable
 *
 * @brief   Sets the write disable mode on the AFE4490should always
 *          get called after a write operation.
 *
 * @param   none
 *
 * @return  none
 */
void AFEWriteDisable()
{
  CS_AFE_ENABLE();
  // Address, Data[2], Date[1], Data[0] (D23 - D0)
  uint8 SPIBuf[4] = {AFE4490_RA_CONTROL0,0x00,0x00,AFE4490_SPI_READ_EN};
  SPITx(4,0,SPIBuf); // All bytes discarded
  CS_AFE_DISABLE();
}
  
/*********************************************************************
 * @fn      AFERead
 *
 * @brief   Reads a specified number of bytes at a specified address inside
 *          the AFE4490.
 *
 * @param   bytes - number of bytes to read
 *          threeByteAddress - Address of the first byte to read
 *          data - Pointer to the container which will hold the read data
 *
 * @return  none
 */
void AFERead(uint8 address, uint8* data)
{
  AFEWriteDisable();

  CS_AFE_ENABLE();
  data[0]= address;
  data[1]=0xFF;
  data[2]=0xFF;
  data[3]=0xFF;
  SPITx(4,3,data);
  waitUs(3);
  CS_AFE_DISABLE();
}

/*********************************************************************
 * @fn      AFEWrite
 *
 * @brief   Writes a specified number of bytes to the AFE4490.
 *
 * @param   bytes - number of bytes to write
 *          threeByteAddress - Address of the first byte to write
 *          data - Pointer to the container which holds the data to write
 *
 * @return  none
 */
void AFEWrite(uint8 address, uint32 data)
{
  uint8 message[3];
  message[0] = data >> 16;
  message[1] = data >> 8;
  message[2] = data;

  AFEWriteEnable();
  CS_AFE_ENABLE();
  SPI2Tx( address, message ); // All bytes discarded
  waitUs(3);
  CS_AFE_DISABLE();
}

/*********************************************************************
 * @fn      AFEStartSampling
 *
 * @brief   Function is called to initialize internal timers for AFE
 *			sampling. Called at the begining of every longterm cycle.
 *
 * @param   bytes - number of bytes to write
 *          threeByteAddress - Address of the first byte to write
 *          data - Pointer to the container which holds the data to write
 *
 * @return  none
 */
void AFEStartSampling()
{
  // Turn on AFE
  AFEPwrUp();

  // Clear Port 0 Intherrupt flags.
  P0IF = 0;
  P0IFG = 0x00;
  
  // Interrupt enable on pin P0.5.
  P0IEN |= BV(6);  
  // Enable CPU Interrupt for Port 0 (IEN1.P0IE = 1).
  P0IE = 1;
  // Enable Global Interrupt by setting the (IEN0.EA = 1).
  EA = 1;
}
/*********************************************************************
 * @fn      AFEStopSampling
 *
 * @brief   Function is called to initialize internal timers for AFE
 *	    	sampling.
 *
 * @param   bytes - number of bytes to write
 *          threeByteAddress - Address of the first byte to write
 *          data - Pointer to the container which holds the data to write
 *
 * @return  none
 */
void AFEStopSampling()
{
  // Powerdown AFE until next longterm sampling period
  AFEPwrDownAFE();
  
  // Interrupt disable on pin P0.6.
  P0IEN &= ~BV(6);  
}

/**********************************************************************
 * @fn          AFEConfigureForLongTerm
 *
 * @brief       Configure AFE register to be ready for long term sampling
 */
static void AFEConfigureForLongTerm( void )
{
  // Basic 500 Hz Configuration (Refer to EVM default in GUI)
  AFEWrite(AFE4490_RA_LED2STC,AFE4490_LED2STC_500HZ);
  AFEWrite(AFE4490_RA_LED2ENDC,AFE4490_LED2ENDC_500HZ);
  AFEWrite(AFE4490_RA_LED2LEDSTC,AFE4490_LED2LEDSTC_500HZ);
  AFEWrite(AFE4490_RA_LED2LEDENDC,AFE4490_LED2LEDENDC_500HZ);
  AFEWrite(AFE4490_RA_LED1STC,AFE4490_LED1STC_500HZ);
  AFEWrite(AFE4490_RA_LED1ENDC,AFE4490_LED1ENDC_500HZ);
  AFEWrite(AFE4490_RA_LED1LEDSTC,AFE4490_LED1LEDSTC_500HZ);
  AFEWrite(AFE4490_RA_LED1LEDENDC,AFE4490_LED1LEDENDC_500HZ);
  AFEWrite(AFE4490_RA_LED2CONVST,AFE4490_LED2CONVST_500HZ);
  AFEWrite(AFE4490_RA_LED2CONVEND,AFE4490_LED2CONVEND_500HZ);
  AFEWrite(AFE4490_RA_LED1CONVST,AFE4490_LED1CONVST_500HZ);
  AFEWrite(AFE4490_RA_LED1CONVEND,AFE4490_LED1CONVEND_500HZ);
  AFEWrite(AFE4490_RA_LADCRSTSTCT0,AFE4490_LADCRSTSTCT0_500HZ);
  AFEWrite(AFE4490_RA_LADCRSTENDCT0,AFE4490_LADCRSTENDCT0_500HZ);
  AFEWrite(AFE4490_RA_LADCRSTSTCT1,AFE4490_LADCRSTSTCT1_500HZ);
  AFEWrite(AFE4490_RA_LADCRSTENDCT1,AFE4490_LADCRSTENDCT1_500HZ);
  AFEWrite(AFE4490_RA_LADCRSTSTCT2,AFE4490_LADCRSTSTCT2_500HZ);
  AFEWrite(AFE4490_RA_LADCRSTENDCT2,AFE4490_LADCRSTENDCT2_500HZ);
  AFEWrite(AFE4490_RA_LADCRSTSTCT3,AFE4490_LADCRSTSTCT3_500HZ);
  AFEWrite(AFE4490_RA_LADCRSTENDCT3,AFE4490_LADCRSTENDCT3_500HZ);
  AFEWrite(AFE4490_RA_PRPCOUNT,AFE4490_PRPCOUNT_500HZ);
  AFEWrite(AFE4490_RA_LEDCNTRL, AFE4490_LEDCNTRL_500HZ);
  
  uint8 tempIrData[4];			// Infrared LED = LED1
  AFERead(AFE4490_RA_LED1CONVST,tempIrData);    // Should be FA6

  // Enabled the internal timer and 0 averaging
  AFEWrite(AFE4490_RA_CONTROL1,0x101);
}