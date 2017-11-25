/****************************************************************
longTermProf.c
****************************************************************/

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "hal_adc.h"
#include "OSAL_Clock.h"

#include "longTermProf.h"

/*****************************************************************
LOCAL VARIABLES
*****************************************************************/
static longTermProfileCBs_t *braceMonitor_LongTermProfileCBs = NULL;

/*****************************************************************
GLOBAL VARIABLES
*****************************************************************/
// Long Term Service
CONST uint8 longTermServUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(LONG_TERM_SERVICE_UUID), HI_UINT16(LONG_TERM_SERVICE_UUID)
};

// Long Term Mode
CONST uint8 longTermModeUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(LONG_TERM_MODE_UUID), HI_UINT16(LONG_TERM_MODE_UUID)
};

// Sampling Rate
CONST uint8 samplingRateUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(SAMPLING_RATE_UUID), HI_UINT16(SAMPLING_RATE_UUID)
};

// Data and Time
CONST uint8 dateAndTimeUUID[ATT_BT_UUID_SIZE] =
{
  LO_UINT16(DATE_AND_TIME_UUID), HI_UINT16(DATE_AND_TIME_UUID)
};

/*****************************************************************
Profile Attributes
*****************************************************************/
// Long Term service attribute
static CONST gattAttrType_t longTermService = { ATT_BT_UUID_SIZE, longTermServUUID };

// Long Term Mode attribute
static uint8 longTermProps = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8 longTermMode = COMMUNICATION_MODE;
static uint8 longTermUserDesp[20] = "Set Long Term Mode\0";

// Sampling Rate attribute
static uint8 samplingRateProps = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8 samplingRate = 10; //default is 10 seconds
static uint8 samplingRateDesp[15] = "Sampling Rate\0";

// Data and Time attribute
static uint8 dateAndTimeProps = GATT_PROP_READ | GATT_PROP_WRITE;
static uint8 dateAndTime[DATE_AND_TIME_LENGTH] = {0, 0, 0, 0, 0, 0};
static uint8 dateAndTimeDesp[15] = "Date and Time\0";

/*****************************************************************
Profile Attributes - Table
*****************************************************************/

static gattAttribute_t longTermProfAttrTbl[] =
{
  // Long Term Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8 *)&longTermService
  },
    
    // Long Term Mode Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &longTermProps 
    },

      // Long Term Mode
      { 
        { ATT_BT_UUID_SIZE, longTermModeUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        &longTermMode
      },

      // Long Term Mode User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        longTermUserDesp
      },
        // Sampling Rate Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &samplingRateProps 
    },

      // Sampling Rate
      { 
        { ATT_BT_UUID_SIZE, samplingRateUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        &samplingRate
      },

      // Sampling Rate User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        samplingRateDesp
      },
    //Date and Time Declaration 
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &dateAndTimeProps
    },
      //Date and Time
      {
        { ATT_BT_UUID_SIZE, dateAndTimeUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        dateAndTime
      },
      //Date and Time User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        dateAndTimeDesp
      }
};

extern uint8 intFlash_ReadSampling( void );
extern void intFlash_WriteSampling( uint8 samplingRate );

/****************************************************************
LOCAL FUNCTIONS
****************************************************************/
static uint8 longTermProf_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t longTermProf_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );

/****************************************************************
PROFILE CALLBACKS
****************************************************************/
CONST gattServiceCBs_t longTermProfCBs =
{
  longTermProf_ReadAttrCB,
  longTermProf_WriteAttrCB,
  NULL
};

/*****************************************************************
PUBLIC FUNCTIONS
*****************************************************************/

/*******************************************************************
 * @fn      LongTermProf_AddService
 *
 * @brief   Initializes the Long Term Mode Service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
********************************************************************/
bStatus_t LongTermProf_AddService( void )
{
  return GATTServApp_RegisterService( longTermProfAttrTbl,
                                     GATT_NUM_ATTRS(longTermProfAttrTbl),
                                     &longTermProfCBs);
}

/********************************************************************
 * @fn          longTermProf_RegisterAppCBs
 *
 * @brief       this function will register the call-back functions
 *              coming from the main application
 */
bStatus_t longTermProf_RegisterAppCBs(longTermProfileCBs_t *appCallbacks)
{
  if ( appCallbacks )
  {
    braceMonitor_LongTermProfileCBs = appCallbacks;
    return SUCCESS;
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

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
bStatus_t longTermProf_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
  case SAMPLING_RATE_PARAM:
    if ( len == sizeof ( uint8 ) )
    {
      samplingRate = *((uint8*)value);
    }
  case DATE_AND_TIME_PARAM:
    if ( len == DATE_AND_TIME_LENGTH )
    {
      osal_memcpy( dateAndTime, value, DATE_AND_TIME_LENGTH );
    }
  }
  return ( ret );
}

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
bStatus_t longTermProf_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
  case SAMPLING_RATE_PARAM:
    *((uint8*)value) = samplingRate;
    break;
  case DATE_AND_TIME_PARAM:
    VOID osal_memcpy(value, dateAndTime, DATE_AND_TIME_LENGTH );
    break;
  default:
    ret = INVALIDPARAMETER;
    break;
  }
  return ( ret );
}


/*********************************************************************
 * @fn          simpleBattReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 *
 * @return      Success or Failure
 */
static uint8 longTermProf_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
{
  bStatus_t status = SUCCESS;
  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
  UTCTime timeInSecs = osal_getClock();
  UTCTimeStruct tm;
  
  switch (uuid)
  {
  case LONG_TERM_MODE_UUID:
    *pLen = 1;
    pValue[0] = *pAttr->pValue;
    break;
  case SAMPLING_RATE_UUID:
    samplingRate = intFlash_ReadSampling();
    *pLen = 1;
    pValue[0] = *pAttr->pValue;
    break;
  case DATE_AND_TIME_UUID:
    // We need to get the time from the OSAL clock and convert it into a form that the GATT table can accept
    osal_ConvertUTCTime( &tm, timeInSecs );
    dateAndTime[5] = tm.seconds;
    dateAndTime[4] = tm.minutes;
    dateAndTime[3] = tm.hour;
    dateAndTime[2] = tm.day + 1; // The osal clock API starts the days at zero, so everything shifts forward by one.
    dateAndTime[1] = tm.month + 1; // The osal clock API starts the months at zero, so everything shifts forward by one.
    dateAndTime[0] = tm.year;
    *pLen = DATE_AND_TIME_LENGTH;
    VOID osal_memcpy( pValue, pAttr->pValue, DATE_AND_TIME_LENGTH );
    break;
  default:
    *pLen = 0;
    status = ATT_ERR_ATTR_NOT_FOUND;
    break;
  }
  return ( status );
}

/*********************************************************************
 * @fn      longTermProf_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   complete - whether this is the last packet
 * @param   oper - whether to validate and/or write attribute value  
 *
 * @return  Success or Failure
 */
static bStatus_t longTermProf_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset )
{
  bStatus_t status = SUCCESS;
  uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
  
  switch (uuid)
  {
  case LONG_TERM_MODE_UUID:
    if ( offset == 0 )
    {
      if ( len != 1 )
      {
        status = ATT_ERR_INVALID_VALUE_SIZE;
      }
    }
    else
    {
      status = ATT_ERR_ATTR_NOT_LONG;
    }   
    
    if ( status == SUCCESS )
    {
      uint8 *pCurValue = (uint8 *)pAttr->pValue;        
      *pCurValue = pValue[0];
      
      if(pValue[0] == 1)
      {
        braceMonitor_LongTermProfileCBs->pfnStartLongTerm();
      }
    }
    break;
  case SAMPLING_RATE_UUID:
    if ( offset == 0 )
    {
      if ( len != 1 )
      {
        status = ATT_ERR_INVALID_VALUE_SIZE;
      }
    }
    else
    {
      status = ATT_ERR_ATTR_NOT_LONG;
    }   
    
    if ( status == SUCCESS )
    {
      uint8 *pCurValue = (uint8 *)pAttr->pValue;        
      *pCurValue = pValue[0];
      intFlash_WriteSampling(samplingRate);
    }
    break;
  case DATE_AND_TIME_UUID:
    if ( offset == 0 )
    {
      if ( len != DATE_AND_TIME_LENGTH )
      {
        status = ATT_ERR_INVALID_VALUE_SIZE;
      }
    }
    else
    {
      status = ATT_ERR_ATTR_NOT_LONG;
    }
    
    if ( status == SUCCESS )
    {
      uint8 *pCurValue = (uint8 *)pAttr->pValue;
      VOID osal_memcpy( pCurValue, pValue, DATE_AND_TIME_LENGTH );
      
      // Update the OSAL's clock to reflect the time set by the GATT client.
      // First we need to put the struct together so we can perform the conversion
      UTCTimeStruct tm;
      tm.seconds = dateAndTime[5];
      tm.minutes = dateAndTime[4];
      tm.hour = dateAndTime[3];
      tm.day = dateAndTime[2] - 1; // The osal clock API starts the days at zero, so everything shifts back by one
      tm.month = dateAndTime[1] - 1; // The osal clock API starts the months at zero, so everything shifts back by one
      tm.year = dateAndTime[0]+2000;
      
      // Now we can actually set the OSAL clock.
      UTCTime timeInSecs = osal_ConvertUTCSecs(&tm);
      osal_setClock(timeInSecs);
    }
    break;
  default:
    status = ATT_ERR_ATTR_NOT_FOUND;
    break;
  }
  return ( status );
}

