/*************************************************************************
 * University of Alberta
 * SPI bus definition file
 *************************************************************************
 * Compiler: IAR C Compiler for 8051
 * Target platform: CC2540/1
 *************************************************************************
 * Revision 1.0.0.0 2014/01/30
 * Fraaz Kamal
 * Acknowledgements: Tyler Charlton, Eric Chalmers, Chris Woloschuk, Jacob Ortt
 *************************************************************************/

#ifndef module_spi_h
#define module_spi_h

/*********************************************************************
 * DEFINES
 */

#define Baud_M  0x80 //34             // Defines the speed of the SPI bus. See
#define Baud_E  0x10 //13            // datasheet page 154 for more info.
#define ADC_Polarity  0x20        // Polarity and clock phase for the AD7689 A/D converter (CPOL = CPHA = 0 when not using the busy indicator)
//#define SPICFG        U0GCR     // Polarity and clock phase
#define SPIBuffer     U0DBUF    // generated interrupt when data is ready

#define SCLK          P1_3      // SPI clock control

//#define U0BaudDefault 163        
#define U0CsrDefault  0         //

/*********************************************************************
 * FUNCTION PROTOTYPES
   */
     
void InitSPI(void);
void SPITx(uint16 BytesTx, uint16 BytesRx, uint8* Payload);
void SPI1Tx(uint8 payload);
void SPI2Tx(uint8 Address, uint8* Data);

/*********************************************************************
* VARIABLES
*/
static uint8 SPIData[16];

#endif