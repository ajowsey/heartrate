/*************************************************************************
 * University of Alberta
 * CC2540/1 Internal flash definition file
 *************************************************************************
 * Compiler: IAR C Compiler for 8051
 * Target platform: CC2540/1
 *************************************************************************
 * Revision 1.0.0.0 2014/01/30
 * Jacob Ortt
 * Acknowledgements: Fraaz Kamal
 *************************************************************************/

#ifndef INTERNALFLASH_H
#define INTERNALFLASH_H

// Here come's senior Drew with some fresh pages
// 20 Pages for Sample Buffer

#define FIRST_SAMPLE_BUFFER_PAGE        0x46 // d70
#define LAST_SAMPLE_BUFFER_PAGE         0x5A // d90
#define FIRST_SAMPLE_BUFFER_ADDRESS     0X23000
#define LAST_SAMPLE_BUFFER_ADDRESS      0X23800

// Storage for total samples
#define FIRST_TOTAL_SAMPLE_PAGE         0x5B    // d91
#define LAST_TOTAL_SAMPLE_PAGE          0x63    // d99
#define FIRST_TOTAL_BUFFER_ADDRESS      0x2D800
#define LAST_TOTAL_BUFFER_ADDRESS       0x31800

////////////////////////////////////////////////////////

#define SAMPLING_RATE_ADDRESS           0x32000
#define SAMPLING_RATE_PAGE              0x64    // d100

#define ACCEL_PAGE                      0x78    // d120
#define ACCEL_ADDRESS                   0x3C000
   
#define GYRO_PAGE                       0x79    // d121
#define GYRO_ADDRESS                    0x3C800
   
#define SLEEPWAKE_PAGE                  0x7A    // d122 
#define SLEEPWAKE_ADDRESS               0x3D000

#define SENSOR_ID_PAGE                  0x7B    // d123
#define SENSOR_ID_ADDRESS               0x3D800 // 123 * 2048 = d251904
#define DEVICE_NAME_LENGTH              20


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
extern void intFlash_WriteSampling( uint8 samplingRate );

/*****************************************************************
 * @fn          intFlash_ReadSampling
 *
 * @brief       will read the sampling rate value from the internal flash
 */
extern uint8 intFlash_ReadSampling( void );

/*****************************************************************
 * @fn          intFlash_WriteDeviceName
 *
 * @brief       will write the device name in the internal flash
 *
 * @param       samplingRate - the sampling rate value that is to be written
 */
extern void intFlash_WriteDeviceName( uint8 * deviceName, bool newValue );

/*****************************************************************
 * @fn          intFlash_ReadDeviceName
 *
 * @brief       will read the device namefrom the internal flash
 */
extern void intFlash_ReadDeviceName( uint8* ); 

extern void intFlash_WriteGyro( uint8* gyroCalibration );

extern uint8 *intFlash_ReadGyro( void );

extern void intFlash_WriteAccel( uint8* accelCalibration );

extern uint8 *intFlash_ReadAccel( void );

extern void intFlash_WriteSleepWake( uint8* sleepWake );

extern void intFlash_WriteLEDSample( uint32 redSample, uint32 irSample, uint16 sampleCount );

extern void intFlash_ReadLEDSample( uint32* redSampleReturn, uint32* irSampleReturn, uint16 sampleIndex );

extern uint8 *intFlash_ReadSleepWake( void );

extern void intFlash_LEDStartNewLog( void );

#endif