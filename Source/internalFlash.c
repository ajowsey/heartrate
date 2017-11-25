/*************************************************************************
 * University of Alberta
 * CC2540/1 Internal flash implementation file
 *************************************************************************
 * Compiler: IAR C Compiler for 8051
 * Target platform: CC2540/1
 *************************************************************************
 * Revision 1.0.0.0 2014/01/30
 * Jacob Ortt
 * Acknowledgements: Fraaz Kamal
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
#include "hal_adc.h"
#include "hal_flash.h"
#include "simpleBLEPeripheral.h"
#include <stdio.h>
#include "internalFlash.h"
#include <stdio.h>
#include <string.h>

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint32 samplePage(uint32 sampleIndex);

/*********************************************************************
 * VARIBLES
 */
static uint32 LEDLastPage = FIRST_SAMPLE_BUFFER_PAGE - 1;
static uint32 LEDStartPage = FIRST_SAMPLE_BUFFER_PAGE;
static uint16 LEDPageCount = 0;

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*****************************************************************
 * @fn          intFlash_WriteSampling
 *
 * @brief       will write the sampling rate value in the internal flash
 *
 * @param       samplingRate - the sampling rate value that is to be written
 */
void intFlash_WriteSampling( uint8 samplingRate )
{
  HalFlashErase(SAMPLING_RATE_PAGE); //erase old data
  
  uint16 address = SAMPLING_RATE_ADDRESS/4;
  uint8* rateBuf = &samplingRate;
  
  HalFlashWrite(address, rateBuf, 1); //write to flash
}

/*****************************************************************
 * @fn          intFlash_ReadSampling
 *
 * @brief       will read the sampling rate value from the internal flash
 *
 * @return      samplingRate - the previous sampling rate stored in flash
 */
uint8 intFlash_ReadSampling( void )
{
  uint8 samplingRate;
  HalFlashRead(SAMPLING_RATE_PAGE, 0, &samplingRate, 1);
  
  return samplingRate;
}

/*****************************************************************
 * @fn          intFlash_WriteSampling
 *
 * @brief       will write the sampling rate value in the internal flash
 *
 * @param       samplingRate - the sampling rate value that is to be written
 */
void intFlash_WriteDeviceName( uint8* deviceName, bool newValue )
{
  HalFlashErase(SENSOR_ID_PAGE); //erase old data
  uint16 address = SENSOR_ID_ADDRESS/4;
  HalFlashWrite(address, deviceName, DEVICE_NAME_LENGTH); //write to flash
}

/*****************************************************************
 * @fn          intFlash_ReadDeviceName
 *
 * @brief       will read the device name value from the internal flash
 *
 * @return      deviceName - the previous name rate stored in flash
 */
void intFlash_ReadDeviceName( uint8* deviceName)
{
  LED_ENABLE();
  
  uint8 pResult[DEVICE_NAME_LENGTH];
  HalFlashRead(SENSOR_ID_PAGE, 0, pResult, DEVICE_NAME_LENGTH);
  VOID osal_memcpy(deviceName, pResult, DEVICE_NAME_LENGTH);
  
  LED_DISABLE();
}

void intFlash_WriteGyro( uint8* gyroCalibration )
{
  HalFlashErase(GYRO_PAGE); // erase old data
  uint16 address = GYRO_ADDRESS/4;
  HalFlashWrite(address, gyroCalibration, 2);
}

uint8 *intFlash_ReadGyro( void )
{
  uint8 _data[6] = {0, 0, 0, 0, 0, 0};
  uint8 *ptr = _data;
  
  HalFlashRead(GYRO_PAGE, 0, _data, 6);
  
  return ptr;
}

void intFlash_WriteAccel( uint8* accelCalibration )
{
  HalFlashErase(ACCEL_PAGE); // erase old data
  uint16 address = ACCEL_ADDRESS/4;
  HalFlashWrite(address, accelCalibration, 2);
}

uint8 *intFlash_ReadAccel( void )
{
  uint8 _data[6] = {0, 0, 0, 0, 0, 0};
  uint8 *ptr = _data;
  
  HalFlashRead(ACCEL_PAGE, 0, _data, 6);
  
  return ptr;
}

void intFlash_WriteSleepWake( uint8* sleepWake )
{
  HalFlashErase(SLEEPWAKE_PAGE); // Erase old data
  uint16 address = SLEEPWAKE_ADDRESS/4;
  HalFlashWrite(address, sleepWake, 1);
}

uint8 *intFlash_ReadSleepWake( void )
{
  uint8 _data[4] = {0, 0, 0, 0};
  uint8 *ptr = _data;
  
  HalFlashRead(SLEEPWAKE_PAGE, 0, _data, 4);
  
  return ptr;
}

void intFlash_WriteLEDSample( uint32 redSample, uint32 irSample, uint16 sampleCount )
{
  uint8 tempBuffer[4];
  uint16 sampleOffset = (sampleCount*8)%2048;
  uint32 tempAddress;
  
  LEDLastPage = samplePage(sampleCount);
  tempAddress = (LEDLastPage * 2048 + sampleOffset)/4;
  
  // Everytime we enter the start of a page, erase old data
  if( sampleOffset == 0 )
  {
    HalFlashErase(samplePage(sampleCount)); 
    LEDPageCount++;
  }
  
  // Write Red Sample (Even indexes)
  memcpy(tempBuffer, &redSample,sizeof(redSample));
  HalFlashWrite(tempAddress++, tempBuffer, 1);

  // Write Infrared Sample (Odd indexes)
  memcpy(tempBuffer, &irSample, sizeof(irSample));
  HalFlashWrite(tempAddress, tempBuffer, 1);
}

// Offset needs to be od
void intFlash_ReadLEDSample( uint32* redSampleReturn, uint32* irSampleReturn, uint16 sampleIndex )
{
  // Read IR and Red Sample
  uint8 _data[8] = {0, 0, 0, 0, 0, 0, 0, 0};  
  HalFlashRead(samplePage(sampleIndex), (sampleIndex*8)%2048, _data, 8);
  
  memcpy(redSampleReturn, _data, 4);
  memcpy(irSampleReturn, &_data[4], 4);
}

uint32 samplePage(uint32 sampleIndex)
{
  uint32 tempAddress = LEDStartPage + ((sampleIndex*8)/2048);
  
  // Prevent Page Overflow
  if(tempAddress > LAST_SAMPLE_BUFFER_PAGE)
  {
    tempAddress -= ( LAST_SAMPLE_BUFFER_PAGE - FIRST_SAMPLE_BUFFER_PAGE + 1);
  }
  return tempAddress;
}

// Start 5 mins log in next Page
// Dont need for first log
void intFlash_LEDStartNewLog()
{
  LEDStartPage = LEDLastPage + 1;
  LEDPageCount = 0;
  
  // Prevent Page Overflow
  if(LEDStartPage > LAST_SAMPLE_BUFFER_PAGE)
  {
    LEDStartPage -= ( LAST_SAMPLE_BUFFER_PAGE - FIRST_SAMPLE_BUFFER_PAGE + 1);
  }
}
