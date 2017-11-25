/*************************************************************************
 * University of Alberta
 * Blood Oxygen Saturation, Heart Rate, Temperature, and IMU Sampling Hub
 *************************************************************************
 * Compiler: IAR C Compiler for 8051
 * Target platform: CC2540/1
 *************************************************************************
 * Revision 1.0.0.0 2017/07/20
 * Acknowledgements: Andrew Tiberius Jowsey
 *************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "sampleTestDataLongDec.h"
#include "simpleBLEPeripheral.h"
#include "pulse_sampling.h"
#include "pulse_AFE.h"
#include "OnBoard.h"
#include "hal_types.h"
#include "internalFlash.h"
#include "filters.h"
#include <string.h> /* memset */


/*********************************************************************
 * CONSTANTS
 */

#define OFFLINE_PROCESSING			TRUE

// General Sampling
#define SAMPLE_BUFFER_LENGTH_SEC	        4
#define DOWNSAMPLE_RATE                         16
#define SAMPLE_FREQEUNCY_HZ			( 500 / DOWNSAMPLE_RATE )// 31.25
#define SAMPLE_BYTE_LNGTH			4
#define SAMPLE_BUFFER_SMP_LGNTH 		( SAMPLE_BUFFER_LENGTH_SEC * SAMPLE_FREQEUNCY_HZ)

// Moving Average - not used
#define SAMPLE_MEAN_WINDOW_SHIFT                5        
#define SAMPLE_MEAN_WINDOW                      BV(SAMPLE_MEAN_WINDOW_SHIFT)

// SpO2
#define SPO2_BUFFER_TIME_S      3   // 3 Seconds worth of Windows
#define RMS_WINDOW              35  // With 500/16 Samples Hz, This is 
                                    // a single heartbeat period at 55bpm    
#define SPO2_WND_CNT            ( SPO2_BUFFER_TIME_S * SAMPLE_FREQEUNCY_HZ )           
#define SPO2_BUFFER_LENGTH      ( RMS_WINDOW * SPO2_WND_CNT ) // Probably wont use..      
#define SPO2_INTER      100
#define SPO2_SLOPE      28

static double redRMSBuffer = 0;
static double irRMSBuffer = 0;
static uint8  rmsCount = 0;
static uint8  rmsCalcCount = 0;
static uint32 ratio[SPO2_WND_CNT];
static uint8 SpO2;
static uint32 averageR = 0;

// HR
#define MAX_PEAK        20
#define DELTA_MIN       220
#define HR_BUFFER_LENGTH 79     // 37*3
#define HR_OFFSET       1000
static uint16 hr_buffer[HR_BUFFER_LENGTH]; // Array of arrays
static uint16 emi_peaks[MAX_PEAK];
static uint16 absorp_peaks[MAX_PEAK];
static uint8 emi_count = 0;
static uint8 absorp_count = 0;
static uint16 hrSampleCount =0;
static uint8 heartRate = 0;

// ISR
static uint16 isrCount = 0;
static uint16 saveSampleCount = 0;

// Filters
static double highPassOutput[2];
static double lowPassOutput[2];

/*********************************************************************
 * VARIABLES
 */
static void pulseSamplingHeartRate(double* filteredData);
static void pulseSamplingSpO2(double *highPassData, double *lowPassData );
static void pulseSamplingFIR( const double *coeffs, uint16 filterLength, double *outputs );
static uint32 pulseSamplingSQRT(uint32 a_nInput);
static uint8 pulseSamplingPeakDetect(
        uint16*         data, /* the data */ 
        uint16          data_count, /* row count of data */ 
        uint16*         emi_peaks, /* emission peaks will be put here */ 
        uint8*          num_emi_peaks, /* number of emission peaks found */
        uint8           max_emi_peaks, /* maximum number of emission peaks */ 
        uint16*         absop_peaks, /* absorption peaks will be put here */ 
        uint8*          num_absop_peaks, /* number of absorption peaks found */
        uint8           max_absop_peaks, /* maximum number of absorption peaks */
        uint8           delta, /* delta used for distinguishing peaks */
        uint8           emi_first /* should we search emission peak first of
                                     absorption peak first? */
        );

/*********************************************************************
 * @fn      pulseSamplingInit
 *
 * @brief   Initialize...ever heard of it?
 *
 * @param   none
 *
 * @return  none
 */
void pulseSamplingInit()
{
  // AFE Init
  AFEInit();
  
  // For testing the internal flash function
  // pulseSamplingSaveTestData();
 
  // Temperature Init
  
  // MPU Init
}

/*********************************************************************
 * @fn      pulseSamplingLongTerm
 *
 * @brief   This function is called by the long term event every 5 minutes
 *			or so. This function will be used intiate a 500 Hz interrupt-driven
 			sampling routine with the AFE4490. At the begining of every cycle,
 			the sample buffer must be reset.
 *
 * @param   none
 *
 * @return  none
 */
void pulseSamplingLongTerm()
{
  isrCount = 0;
  
  // Reset buffer location
  intFlash_LEDStartNewLog();
    
  // Turn on LED Sampling
  AFEStartSampling();
}

/*********************************************************************
 * @fn      pulseSamplingSaveTestData
 *
 * @brief   Fill the internal flash with test data   
 *
 * @param   none
 *
 * @return  none
 */
void pulseSamplingSaveTestData()
{
  uint32 testReadRed;
  uint32 testReadIR;

  // Write Sample Data into Internal Flash
  for (uint16 n = 0; n<(sizeof(redTestSamples)/4); n++)
  {
    intFlash_WriteLEDSample((uint32)redTestSamples[n],(uint32)irTestSamples[n],n);
    intFlash_ReadLEDSample( &testReadRed, &testReadIR, n );
  }

  // 1999 is the last sample
  intFlash_ReadLEDSample( &testReadRed, &testReadIR, 1999 );
}

/*********************************************************************
 * @fn      pulseSamplingHeartRate
 *
 * @brief   When the sample buffer is full, this function will be
 *			used to determine heart rate. Once a reasonable heart 
 *			rate has been calculated, the latest value is stored
 *			at internal flash and the corresponding GATT characteristic
 *			is updated.
 *
 * @param	none
 *
 * @return  1 byte heart rate measurement to be added to total sample
 */
void pulseSamplingHeartRate( double *newData)
{
 uint32 tempAverages;
 uint16* hr_peaks;     // For after peaks are detected
 uint8  hr_count = 0;   // For after peaks are detected

 
 // Only save Red Samples since we only need one signal for HR
 hr_buffer[hrSampleCount++] += (uint16)(HR_OFFSET+newData[0]);
 
 // Buffer is full. Look for heart rate now
 if( hrSampleCount == HR_BUFFER_LENGTH )
 {
  pulseSamplingPeakDetect(hr_buffer,hrSampleCount,
                          emi_peaks, &emi_count, MAX_PEAK,
                          absorp_peaks, &absorp_count, MAX_PEAK,
                          DELTA_MIN, 1);
  
  // Find the one that allows the most
 // average heart rate caluclations
 if( absorp_count> emi_count)
 {
   hr_count = absorp_count;
   hr_peaks = absorp_peaks;
 }
 else
 {
   hr_count = emi_count;
   hr_peaks = emi_peaks;
 }
 
  // Need to find average distance between peaks
  if( hr_count > 1)
  {
    tempAverages = 0;
    for( uint8 n = 0; n < hr_count - 1; n++)
    {
      tempAverages += hr_peaks[n+1] - hr_peaks[n];
    }
    tempAverages = tempAverages/(hr_count-1);
    tempAverages = 60 * ( 500 / DECIMATION_FACTOR ) / tempAverages;
    heartRate = (uint8)tempAverages;
  }
 }
}

/*********************************************************************
 * @fn      pulseSamplingSpO2
 *
 * @brief   
 *
 * @param   none
 *
 * @return  1 byte heart rate measurement to be added to total sample
 */
void pulseSamplingSpO2(double *highPassData, double *lowPassData )
{ 
// Normalize and Convert the data streams
redRMSBuffer += ( ( (uint32)( highPassData[0] / lowPassData[0] ) )^2 )/RMS_WINDOW;
irRMSBuffer += ( ( (uint32)( highPassData[1] / lowPassData[1] ) )^2 )/RMS_WINDOW;
rmsCount++;

if( rmsCount >= RMS_WINDOW )
  {
  rmsCount = 0; // Reset the counter
  ratio[rmsCalcCount] = pulseSamplingSQRT((uint32)(redRMSBuffer)) / pulseSamplingSQRT((uint32)(irRMSBuffer));
  averageR += ratio[rmsCalcCount++];
  
  // Clear values for the next iteration
  redRMSBuffer = 0;
  irRMSBuffer = 0;
  }

// Take an average R value
if( rmsCalcCount >= SPO2_WND_CNT )
  {
    SpO2 = (uint8)( SPO2_INTER - ( averageR/SPO2_WND_CNT )*SPO2_SLOPE );
  }
}

/***********************************************************************************
* @fn          pulseSamplingAFEIsr
*
* @brief       Port 0 Interrupt Service Routine, which executes when the AFE4490's
*              ADC_RDY pin creates a rising edge on P0.5. We will read the sample
*              and reset the interrupt.
*
* @param       void
*
* @return      void */
void pulseSamplingAFEIsr()
{
  uint32 RedSample;
  uint32 IRSample;
  
  if((isrCount++ % DECIMATION_FACTOR) == 0)
  {    
   // Read a sample
    AFEReadSample(&RedSample, &IRSample);
    
    if(saveSampleCount<sizeof(redTestSamples)/sizeof(uint32))
      {
      /////////////////////////
      //  Fake Measurements  //
      // Change param to isrCount-1 if using non dec samples
      RedSample = redTestSamples[saveSampleCount];
      IRSample = irTestSamples[saveSampleCount];
      /////////////////////////
      
      // Save samples
      intFlash_WriteLEDSample(RedSample, IRSample,saveSampleCount++);

      // We know enough samples are saved to do FIR
      if(saveSampleCount >= MAX_FILTER_LENGTH)
      {
        // FIR
        pulseSamplingFIR(hpfilter,HP_FILTER_LENGTH,highPassOutput);        
        pulseSamplingFIR(lpfilter,LP_FILTER_LENGTH,lowPassOutput);
        
        // Post Processing                            
        pulseSamplingHeartRate(highPassOutput);
        pulseSamplingSpO2(highPassOutput,lowPassOutput);
      }
    }
  }
} 


/***********************************************************************************
* @fn           pulseSamplingFIR
*
* @brief        https://sestevenson.wordpress.com/implementation-of-fir-filtering-in-c-part-1/ */
void pulseSamplingFIR( const double *coeffs, uint16 filterLength, double *outputs )
{
    const double        *coeffp; // pointer to coefficients
    uint32              tempSavedSample[2];
    
    coeffp = coeffs;
    memset( outputs, 0, sizeof( outputs ) );

    for ( uint16 k = 0; k < filterLength; k++ ) {
        intFlash_ReadLEDSample(&tempSavedSample[0],&tempSavedSample[1], saveSampleCount-k-1);
        for ( uint8 i = 0; i < 2; i++ )
        {
          outputs[i] += (*coeffp) * tempSavedSample[i];
        }
        coeffp++;
    }
}

/***********************************************************************************
* @fn           pulseSamplingPeakDetect
*
* @brief        https://github.com/xuphys/peakdetect */

uint8 pulseSamplingPeakDetect(
        uint16*          data, /* data */ 
        uint16          data_count, /* row count of data */ 
        uint16*         emi_peaks, /* emission peaks will be put here */ 
        uint8*          num_emi_peaks, /* number of emission peaks found */
        uint8           max_emi_peaks, /* maximum number of emission peaks */ 
        uint16*         absop_peaks, /* absorption peaks will be put here */ 
        uint8*          num_absop_peaks, /* number of absorption peaks found */
        uint8           max_absop_peaks, /* maximum number of absorption peaks
                                            */ 
        uint8           delta, /* delta used for distinguishing peaks */
        uint8           emi_first /* should we search emission peak first of
                                     absorption peak first? */
        )
{
  uint16        i;
  uint16        mx;
  uint16        mn;
  uint16        mx_pos = 0;
  uint16        mn_pos = 0;
  uint8         is_detecting_emi = emi_first;

  mx = data[0];
  mn = data[0];

  *num_emi_peaks = 0;
  *num_absop_peaks = 0;

  for(i = 1; i < data_count; ++i)
  {
      if(data[i] > mx)
      {
          mx_pos = i;
          mx = data[i];
      }
      if(data[i] < mn)
      {
          mn_pos = i;
          mn = data[i];
      }

      if(is_detecting_emi &&
              data[i] < mx - delta)
      {
          if(*num_emi_peaks >= max_emi_peaks) /* not enough spaces */
              return 1;

          emi_peaks[*num_emi_peaks] = mx_pos;
          ++ (*num_emi_peaks);

          is_detecting_emi = 0;

          i = mx_pos - 1;

          mn = data[mx_pos];
          mn_pos = mx_pos;
      }
      else if((!is_detecting_emi) &&
              data[i] > mn + delta)
      {
          if(*num_absop_peaks >= max_absop_peaks)
              return 2;

          absop_peaks[*num_absop_peaks] = mn_pos;
          ++ (*num_absop_peaks);

          is_detecting_emi = 1;
          
          i = mn_pos - 1;

          mx = data[mn_pos];
          mx_pos = mn_pos;
      }
  }

  return 0;
}
                                              
/***********************************************************************************
* @fn           pulseSamplingSQRT
*
* @brief        https://gist.github.com/foxtrotbravao/2782571

* Calculate the square root of the argument using the iterative Babylonian method.
* Details of the algorithm are online at Wikipedia, I tweaked it to output an integer answer.*/
                                           
uint32 pulseSamplingSQRT(uint32 a_nInput)
{
    uint32 op  = a_nInput;
    uint32 res = 0;
    uint32 one = 1uL << 30; // The second-to-top bit is set: use 1u << 14 for uint16_t type; use 1uL<<30 for uint32_t type
    a_nInput = 16;


    // "one" starts at the highest power of four <= than the argument.
    while (one > op)
    {
        one >>= 2;
    }

    while (one != 0)
    {
        if (op >= res + one)
        {
            op = op - (res + one);
            res = res +  2 * one;
        }
        res >>= 1;
        one >>= 2;
    }

    /* Do arithmetic rounding to nearest integer */
    if (op > res)
    {
        res++;
    }

    return res;
}