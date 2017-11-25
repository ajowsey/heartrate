/*************************************************************************
 * University of Alberta
 * SPI bus implementation file
 *************************************************************************
 * Compiler: IAR C Compiler for 8051
 * Target platform: CC2540/1
 *************************************************************************
 * Revision 1.0.0.0 2014/01/30
 * Fraaz Kamal
 * Acknowledgements: Tyler Charlton, Eric Chalmers, Chris Woloschuk, Jacob Ortt
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
#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_lcd.h"
#include "pulse_spi.h"


/*********************************************************************
 * @fn      InitSPI
 *
 * @brief   Initializes the CC2540/1's SPI bus.
 *
 * @param   none
 *
 * @return  none
 */
void InitSPI(void)
{
  
  U0BAUD =        Baud_M ;  
  U0GCR =         ADC_Polarity | Baud_E;    
  U0CSR =         U0CsrDefault;   // set to 0: SPI mode, receiver UART disable, SPI master for micontroller
}
/*******************************************************************************
SPITx

This function deals with both transmitting and receiving using SPI.  BytesTx is
the number of bytes that will be sent out along the SPI line and BytesRx is the 
number of bytes that are expected to be read.

BytesDiscarded is the number of bytes that are to be sent out that do not require
a value to be read back (ex: memory command or address).

Then to receive bytes back, dummy values in Payload are transmitted out and the
received values are then saved back into Payload.

Example:  We are trying to read two bytes of data from memory location 0x01 so,

BytesTx = 5, Bytes Rx = 2, Payload = {0x00,0x00,0x01,0x00,0x00}

So, BytesDiscarded = 3, meaning these bytes will be sent to the SPI buffer, but
what comes back does not matter.  In this case, the first 3 bytes is the memory
location we are trying to read from.  Then, two more dummy bytes of value 0x00
are sent to the SPI buffer and the values returned to the buffer from the memory
are saved in Payload[n-3].

*******************************************************************************/

/*********************************************************************
 * @fn      SPITx
 *
 * @brief   See the above comment block. 
 *
 * @param   BytesTx - Bytes to transmit. See above.
 * @param   BytesRx - Bytes to receive. See above.
 * @param   Payload - Container of bytes to transmit and/or receive.
 *
 * @return  none
 */
void SPITx(uint16 BytesTx, uint16 BytesRx, uint8* Payload)
{
  uint16 n=0;
  uint16 BytesDiscarded = BytesTx-BytesRx;
  
  // transmit the first set of bytes (recieved bytes discarded)
  for (n=0; n<(BytesDiscarded); n++)
  {
    SPIBuffer = Payload[n];
    while((U0CSR&0x01)){}
    SPIBuffer;
  }
  // transmit the rest of the bytes (recieved bytes saved in payload buffer)
  for (n=n; n<BytesTx; n++)
  {
    SPIBuffer = Payload[n];
    while((U0CSR&0x01)){}
    Payload[n-BytesDiscarded]=SPIBuffer;
  }

  return;
}//Ends SPITx

/*********************************************************************
 * @fn      SPI1Tx
 *
 * @brief   If you just have to transmit one byte and you don't care
 *          about the response, this is more efficient than the above function
 *          and doesn't require pointer manipulation.
 *
 * @param   payload - Byte to send over the SPI bus. 
 *
 * @return  none
 */
void SPI1Tx(uint8 payload)
{
  SPIBuffer = payload;
  while((U0CSR&0x01)){}
  SPIBuffer;
}

/*********************************************************************
 * @fn      SPI2Tx
 *
 * @brief   If you just have to transmit one byte and you don't care
 *          about the response, this is more efficient than the above function
 *          and doesn't require pointer manipulation.
 *
 * @param   payload - Byte to send over the SPI bus. 
 *
 * @return  none
 */
void SPI2Tx(uint8 Address, uint8* Data)
{
  SPIBuffer = Address;
  for (uint16 n=0; n<3; n++)
  {
    while(U0CSR&0x01){}
   SPIBuffer = Data[n];
  }
}