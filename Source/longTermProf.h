
#ifndef LONGTERM_H
#define LONGTERM_H

#ifdef __cplusplus
extern "C"
{
#endif

// Long Term UUIDs
#define LONG_TERM_SERVICE_UUID             0x1814
#define LONG_TERM_MODE_UUID                0x2B03
#define SAMPLING_RATE_UUID                 0x2B04
#define DATE_AND_TIME_UUID                 0x2B05
  
// Modes values
#define COMMUNICATION_MODE                 0x00
#define LONG_TERM_MODE                     0x01
  
#define DATE_AND_TIME_LENGTH               6
  
// Profile Parameters
#define SAMPLING_RATE_PARAM                0
#define DATE_AND_TIME_PARAM                1
  
typedef NULL_OK void (*longTermCB_t)( void );

typedef struct
{
  longTermCB_t          pfnStartLongTerm;
} longTermProfileCBs_t;

/*******************************************************************
 * @fn      LongTermProf_AddService
 *
 * @brief   Initializes the Long Term Mode Service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
********************************************************************/
extern bStatus_t LongTermProf_AddService( void );

/********************************************************************
 * @fn          longTermProf_RegisterAppCBs
 *
 * @brief       this function will register the call-back functions
 *              coming from the main application
 */
extern bStatus_t longTermProf_RegisterAppCBs(longTermProfileCBs_t *appCallbacks);

/*********************************************************************
 * @fn      longTermProf_SetParameter
 *
 * @brief   Should be used at the start of the program to set the rate of
 *          sampling rate based on the value that is saved in the internal
 *          memory of the chip.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
extern bStatus_t longTermProf_SetParameter( uint8 param, uint8 len, void *value );

/*********************************************************************
 * @fn      longTermProf_GetParameter
 *
 * @brief   Is used to get the current sampling rate
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t longTermProf_GetParameter( uint8 param, void *value );

#ifdef __cplusplus
}
#endif

#endif /* LONGTERM_H */