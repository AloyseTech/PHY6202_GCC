/**************************************************************************************************
 
  Phyplus Microelectronics Limited confidential and proprietary. 
  All rights reserved.

  IMPORTANT: All rights of this software belong to Phyplus Microelectronics 
  Limited ("Phyplus"). Your use of this Software is limited to those 
  specific rights granted under  the terms of the business contract, the 
  confidential agreement, the non-disclosure agreement and any other forms 
  of agreements as a customer or a partner of Phyplus. You may not use this 
  Software unless you agree to abide by the terms of these agreements. 
  You acknowledge that the Software may not be modified, copied, 
  distributed or disclosed unless embedded on a Phyplus Bluetooth Low Energy 
  (BLE) integrated circuit, either as a product or is integrated into your 
  products.  Other than for the aforementioned purposes, you may not use, 
  reproduce, copy, prepare derivative works of, modify, distribute, perform, 
  display or sell this Software and/or its documentation for any purposes.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED AS IS WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  PHYPLUS OR ITS SUBSIDIARIES BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
  
**************************************************************************************************/

/**************************************************************************************************
  Filename:       simpleBLEPeripheral.c
  Revised:        
  Revision:       

  Description:    This file contains the Simple BLE Peripheral sample application
                  

**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "rf_phy_driver.h"
#include "global_config.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
//#include "OnBoard.h"
#include "gatt.h"
#include "hci.h"
#include "log.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleIBeaconProfile_ota.h"

#include "peripheral.h"
#include "gapbondmgr.h"
#include "flash.h"
#include "simpleBLEPeripheral.h"
#include "ll.h"
#include "clock.h"
#include "common.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

// How often to perform periodic event
#define SBP_PERIODIC_EVT_PERIOD                   5000

#define DEVINFO_SYSTEM_ID_LEN             8
#define DEVINFO_SYSTEM_ID                 0
 

#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     24//32//80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     800//48//800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY         0//0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT          500//1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         TRUE

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL         6

#define INVALID_CONNHANDLE                    0xFFFF

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15


/*********************************************************************
 * TYPEDEFS
 */


/*********************************************************************
 * GLOBAL VARIABLES
 */
 typedef struct
 {
	 uint8_t scan_mac[B_ADDR_LEN];
	 int8_t rssi;
 }Scan_Info;

/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern volatile uint8_t g_current_advType;
extern uint16 g_conn_param_foff;
extern uint8 g_conn_param_carrSens;
extern uint32 counter_tracking;

extern uint32 g_counter_traking_avg;
extern uint32 g_counter_traking_cnt;
extern uint32_t  g_TIM2_IRQ_TIM3_CurrCount;
extern uint32_t  g_TIM2_IRQ_to_Sleep_DeltTick;
extern uint32_t  g_osal_tick_trim;
extern uint32_t  g_TIM2_IRQ_PendingTick;
extern uint32_t  g_TIM2_wakeup_delay;
/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 simpleBLEPeripheral_TaskID;   // Task ID for internal task/event processing

static gaprole_States_t gapProfileState = GAPROLE_INIT;
static  uint8_t notifyBuf[20]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20};
static uint16 notifyInterval = 0;
static uint8 notifyPktNum = 0;
static uint8 connEvtEndNotify =0;
static uint16 notifyCnt = 0;


#if (DBG_RTC_TEST==1)
static uint32 testRtcCnt=0;
static uint32 testRtcCntLast=0;
static uint32 testRtcCntHigh=0;
#endif
	
Scan_Info g_scan_info;


// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
    // complete name
    0x12,   // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    0x50,   // 'P'
    0x68,   // 'h'
    0x79,   // 'y'
    0x70,   // 'p'
    0x6c,   // 'l'
    0x75,   // 'u'
    0x73,   // 's'
	0x20,	// ''
	0x2d,	// '-'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
    0x46,   // 'F'
  

    // connection interval range
    0x05,   // length of this data
    GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
    LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
    HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),
    LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
    HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),

    // Tx power level
    0x02,   // length of this data
    GAP_ADTYPE_POWER_LEVEL,
    0       // 0dBm
};


// advert data for iBeacon
static uint8 advertData[] =
{	
    0x02,   // length of this data
    GAP_ADTYPE_FLAGS,
    DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
    0x1A, // length of this data including the data type byte
    GAP_ADTYPE_MANUFACTURER_SPECIFIC, // manufacturer specific adv data type
    0x4c, // Company ID - Fixed
    0x00, // Company ID - Fixed
    0x02, // Data Type - Fixed
    0x15, // Data Length - Fixed
    0xFD, // UUID  
    0xA5, // UUID 
    0x06, // UUID
    0x93, // UUID
    0xA4, // UUID
    0xE2, // UUID
    0x4F, // UUID
    0xB1, // UUID
    0xAF, // UUID
    0xCF, // UUID
    0xC6, // UUID
    0xEB, // UUID
    0x07, // UUID
    0x64, // UUID
    0x78, // UUID
    0x25, // UUID
    0x27, // Major
    0x74, // Major
    0x6b,//0x04, // Minor
    0xed,//0xb0, // Minor
    0xc5 // Power - The 2's complement of the calibrated Tx Power
};


static uint8 otaAdvIntv         = 100;      //unit is 10ms
static uint8 otaConnIntvMax     = DEFAULT_DESIRED_MIN_CONN_INTERVAL>>2;        //unit is 5ms
static uint8 otaConnIntvMin     = DEFAULT_DESIRED_MAX_CONN_INTERVAL>>2;        //uiit is 5ms
static uint8 otaConnIntvLatency = DEFAULT_DESIRED_SLAVE_LATENCY;        //
static uint8 otaConnTimeOut     = DEFAULT_DESIRED_CONN_TIMEOUT/100;        //unit is second
static uint8 sysClockChangeFlg  = 0;

// GAP GATT Attributes
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "PhyPlus -FFFFFF ";

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void simpleProfileChangeCB( uint8 paramID );
static void updateAdvData(void);
static void peripheralStateReadRssiCB( int8 rssi  );
char *bdAddr2Str( uint8 *pAddr );
static uint8_t simpleBLEPeripheral_ScanRequestFilterCBack(void);
static uint8_t simpleBLEPeripheral_MasterRssiScanCBack(void);
/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
    peripheralStateNotificationCB,  // Profile State Change Callbacks
    peripheralStateReadRssiCB       // When a valid RSSI is read from controller (not used by application)
};


// Simple GATT Profile Callbacks
static simpleProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
{
    simpleProfileChangeCB    // Charactersitic value change callback
};
/*********************************************************************
 * PUBLIC FUNCTIONS
 */



/*********************************************************************
 * @fn      SimpleBLEPeripheral_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLEPeripheral_Init( uint8 task_id )
{
    simpleBLEPeripheral_TaskID = task_id;

    //LL_PLUS_LoadMACFromFlash(0x5070);
    pplus_LoadMACFromChipMAddr();
	LL_PLUS_SetScanRequestFilterCB(simpleBLEPeripheral_ScanRequestFilterCBack);
	//LL_PLUS_SetScanRequestFilterCB(simpleBLEPeripheral_MasterRssiScanCBack);
    
    // Setup the GAP
    VOID GAP_SetParamValue( TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL );
  
    // Setup the GAP Peripheral Role Profile
    {
        // device starts advertising upon initialization
        uint8 initial_advertising_enable = TRUE;

        uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
        uint8 advChnMap = GAP_ADVCHAN_37 | GAP_ADVCHAN_38 | GAP_ADVCHAN_39; 
        
        // By setting this to zero, the device will go into the waiting state after
        // being discoverable for 30.72 second, and will not being advertising again
        // until the enabler is set back to TRUE
        uint16 gapRole_AdvertOffTime = 0;
    
        uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
        uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
        uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
        uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;
		
        uint8 peerPublicAddr[] = {
			0x01,
			0x02,
			0x03,
			0x04,
			0x05,
			0x06
		};


        uint8 advType =g_current_advType;// LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT;//LL_ADV_SCANNABLE_UNDIRECTED_EVT;//LL_ADV_CONNECTABLE_LDC_DIRECTED_EVT;//;    // it seems a  bug to set GAP_ADTYPE_ADV_NONCONN_IND = 0x03
        GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType );
        
        GAPRole_SetParameter(GAPROLE_ADV_DIRECT_ADDR, sizeof(peerPublicAddr), peerPublicAddr);
        // set adv channel map
        GAPRole_SetParameter(GAPROLE_ADV_CHANNEL_MAP, sizeof(uint8), &advChnMap);        

        // Set the GAP Role Parameters
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
        GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

        GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
        GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

        GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
        GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
        GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
        GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
        GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
    }

    // Set the GAP Characteristics
    GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

    // Set advertising interval
    {
        uint16 advInt = 800;//1600;   // actual time = advInt * 625us

        GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
        GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
        GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
    }
	
    // Initialize GATT attributes
    GGS_AddService( GATT_ALL_SERVICES );            // GAP
    GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
    DevInfo_AddService();                           // Device Information Service
    SimpleProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile

    // Setup the SimpleProfile Characteristic Values
    {
        uint8  uuid_setting[IBEACON_UUID_LEN] = {
          0xFD,
          0xA5,
          0x06,
          0x93,
          0xA4,
          0xE2,
          0x4F,
          0xB1,
          0xAF,
          0xCF,
          0xC6,
          0xEB,
          0x07,
          0x64,
          0x78,
          0x25
        };
        uint16 major = 0x2774;
        uint16 minor = 0x6bed;
        uint8 power = 0x0f;
        uint8 reset = 0x0;
	
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR1, IBEACON_UUID_LEN, uuid_setting);
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR2, sizeof ( uint16 ), &major );
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR3, sizeof ( uint16 ), &minor );
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR4, sizeof ( uint8 ), &power );
        SimpleProfile_SetParameter( SIMPLEPROFILE_CHAR5, sizeof ( uint8 ), &reset );
    }

    // Register callback with SimpleGATTprofile
    VOID SimpleProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );

    // Setup a delayed profile startup
    osal_set_event( simpleBLEPeripheral_TaskID, SBP_START_DEVICE_EVT );

#if(DBG_RTC_TEST==1)
    osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_RTC_TEST_EVT, 1000 );
#endif
    
    // start a 60second timer for enter non conn state
    //VOID osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_ENTER_NOCONN_EVT, 60 * 1000);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLEPeripheral_ProcessEvent( uint8 task_id, uint16 events )
{
    VOID task_id; // OSAL required parameter that isn't used in this function

    if ( events & SYS_EVENT_MSG )
    {
        uint8 *pMsg;

        if ( (pMsg = osal_msg_receive( simpleBLEPeripheral_TaskID )) != NULL )
        {
            simpleBLEPeripheral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

            // Release the OSAL message
            VOID osal_msg_deallocate( pMsg );
        }

        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }

    if ( events & SBP_START_DEVICE_EVT )
    {
        // Start the Device
        VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );

        // Start Bond Manager
        //  VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );

        // Set timer for first periodic event
        osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, SBP_PERIODIC_EVT_PERIOD );

        return ( events ^ SBP_START_DEVICE_EVT );
    }
    // enable adv
    if ( events & SBP_RESET_ADV_EVT )
    {
        uint8 initial_advertising_enable = TRUE;
		
        GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );	
	  
        return ( events ^ SBP_RESET_ADV_EVT );
    }  
		
		if (events & SBP_SCAN_RSP_EVT )
		{
			//LOG("SBP_SCAN_RSP_EVT address:%02X,%02X,%02X,%02X,%02X,%02X; rssi:%d\n",g_scan_info.scan_mac[0],\
			g_scan_info.scan_mac[1],\
			g_scan_info.scan_mac[2],\
			g_scan_info.scan_mac[3],\
			g_scan_info.scan_mac[4],\
			g_scan_info.scan_mac[5],\
			g_scan_info.rssi);
			return ( events ^ SBP_SCAN_RSP_EVT );
		}

        // notifity
    if ( events & SBP_PERIODIC_EVT )
    {
//        uint8_t perReportFlg;
//        perStats_t perStats;
//        if(notifyCnt>perReportIntv+LL_MAX_NUM_DATA_CHAN+1)
//        {
//            notifyCnt=0;
//        }
        
//        if(notifyCnt==0)
//        {
//            perReportFlg=0;
//            LL_PLUS_PerStatsReset();
//            perRxNumTotal=0;
//            perRxCrcErrTotal=0;
//            perTxNumTotal=0;
//            perTxAckTotal=0;
//            perRxToCntTotal=0;
//            perConnEvtTotal=0;
//
////            perReportIntv = notifyPktNum*(perReportTime*1000+((notifyInterval)>>1))/(notifyInterval);
//            
//            notifyBuf[0]=HI_UINT16(notifyCnt);
//            notifyBuf[1]=LO_UINT16(notifyCnt);
//        }
        

        for(int i=0;i<notifyPktNum;i++)
        {
#if 0        
            if(     (notifyCnt > perReportIntv) 
                &&  (notifyCnt <= perReportIntv+LL_MAX_NUM_DATA_CHAN+1) )
            {  
                perReportFlg=0x80;
                notifyBuf[0]=perReportFlg|HI_UINT16(notifyCnt-perReportIntv-1);
                notifyBuf[1]=LO_UINT16(notifyCnt-perReportIntv-1);

                if(notifyCnt>perReportIntv+LL_MAX_NUM_DATA_CHAN)
                {
                    perStats.rxNumPkts=perRxNumTotal;
                    perStats.rxNumCrcErr=perRxCrcErrTotal;
                    perStats.txNumRetry=perTxNumTotal;
                    perStats.TxNumAck=perTxAckTotal;
                    perStats.rxToCnt=perRxToCntTotal;
                    perStats.connEvtCnt=perConnEvtTotal;
                    
                }
                else
                {
                    LL_PLUS_PerStasReadByChn(notifyCnt-perReportIntv-1,&perStats);
                    perRxNumTotal+=perStats.rxNumPkts;
                    perRxCrcErrTotal+=perStats.rxNumCrcErr;
                    perTxNumTotal+=perStats.txNumRetry;
                    perTxAckTotal+=perStats.TxNumAck;
                    perRxToCntTotal+=perStats.rxToCnt;
                    perConnEvtTotal+=perStats.connEvtCnt;


                }

                notifyBuf[2]=HI_UINT16(perStats.connEvtCnt);
                notifyBuf[3]=LO_UINT16(perStats.connEvtCnt);
                notifyBuf[4]=HI_UINT16(perStats.rxNumPkts);
                notifyBuf[5]=LO_UINT16(perStats.rxNumPkts);
                notifyBuf[6]=HI_UINT16(perStats.rxNumCrcErr);
                notifyBuf[7]=LO_UINT16(perStats.rxNumCrcErr);
                notifyBuf[8]=HI_UINT16(perStats.rxToCnt);
                notifyBuf[9]=LO_UINT16(perStats.rxToCnt);
                notifyBuf[10]=HI_UINT16(perStats.TxNumAck);
                notifyBuf[11]=LO_UINT16(perStats.TxNumAck);
                notifyBuf[12]=HI_UINT16(perStats.txNumRetry);
                notifyBuf[13]=LO_UINT16(perStats.txNumRetry);

                if(notifyCnt-perReportIntv-1==0)
                    LOG("\n[===PER STATS===]\n");
                
                LOG("%02x %04x %04x %04x %04x %04x %04x\n",notifyCnt-perReportIntv-1,
                                                                perStats.connEvtCnt,
                                                                perStats.rxNumPkts,
                                                                perStats.rxNumCrcErr,
                                                                perStats.rxToCnt,
                                                                perStats.TxNumAck,
                                                                perStats.txNumRetry);
           }
            else
            {
                notifyBuf[0]=HI_UINT16(notifyCnt);
                notifyBuf[1]=LO_UINT16(notifyCnt);
            }
#endif

            notifyBuf[0]=HI_UINT16(notifyCnt);
            notifyBuf[1]=LO_UINT16(notifyCnt);

            if(i==0)
            {
                uint16 connIntv;
                uint16 connlatency;
                GAPRole_GetParameter(GAPROLE_CONN_INTERVAL,&connIntv);
                GAPRole_GetParameter(GAPROLE_CONN_LATENCY, &connlatency);
                
                notifyBuf[14]=HI_UINT16(connIntv);
                notifyBuf[15]=LO_UINT16(connIntv);
                notifyBuf[16]=LO_UINT16(connlatency);
                //HCI_ReadRssiCmd(0);//update rssi;
                int8 rssi;
                LL_ReadRssi(0,&rssi);
                notifyBuf[17]=rssi;
                notifyBuf[18]=0xff&((g_conn_param_foff-512)>>2);//4KHz resolution
                notifyBuf[19]=g_conn_param_carrSens;
            }

            uint8 status= simpleProfile_Notify(SIMPLEPROFILE_CHAR6,20,notifyBuf);
            
            if(SUCCESS==status)
            {             
                //LOG("[NOTF_TX] %02x %4x\n",status,notifyCnt);
                notifyCnt++;
            }
            else
            {
                //LOG("[NOTF_TX ERR] %02x %4x\n",status,notifyCnt);
                break;
            }
            
        }

      
        
        
        if(notifyInterval>0 )
        {
            if(connEvtEndNotify==0)
                osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, notifyInterval );
            
        }
        else
        {
            osal_stop_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT );
            notifyCnt=0;
        }


        
        return ( events ^ SBP_PERIODIC_EVT );
    } 
#if (DBG_RTC_TEST==1)

    if ( events & SBP_RTC_TEST_EVT )
    {
        uint32 dTime = (getMcuPrecisionCount());//uint 10ms

        testRtcCnt  =  clock_time_rtc();

        if(testRtcCntLast>testRtcCnt)
            testRtcCntHigh++;
            
        testRtcCntLast  =  clock_time_rtc();

        uint16_t msec;
        uint8_t  sec;
        uint8_t  min;
        uint8_t  hour;

        hour = (dTime/(3600*1600));
        min  = ((dTime-hour*(3600*1600))/(60*1600));
        sec  = (dTime-hour*(3600*1600)-min*(60*1600))/(1600);
        msec = (10*(dTime-hour*(3600*1600)-min*(60*1600)-sec*1600))>>4;


        uint32_t rtc_time_sec = (testRtcCntHigh<<9)+(testRtcCnt>>15);
        uint16_t rtc_msec     = ((testRtcCnt&0x7fff)*1000)>>15;
        uint8_t  rtc_sec;
        uint8_t  rtc_min;
        uint8_t  rtc_hour;

        rtc_hour = rtc_time_sec/(3600);
        rtc_min  = (rtc_time_sec-(3600*rtc_hour))/60;
        rtc_sec  = rtc_time_sec-(3600*rtc_hour)-rtc_min*60;
        

        

        LOG("%03d:%02d:%02d:%03d %03d:%02d:%02d:%03d %3d %08d %4d %6d %6d %8d %3d %d %d\n",
            hour,min,sec,msec,
            rtc_hour,rtc_min,rtc_sec,rtc_msec,sec*1000+msec-(rtc_sec*1000+rtc_msec),
            dTime,counter_tracking,1000000-(g_TIM2_IRQ_TIM3_CurrCount>>2),g_TIM2_IRQ_to_Sleep_DeltTick>>2,testRtcCnt,g_TIM2_IRQ_PendingTick,625-(g_TIM2_wakeup_delay>>2),g_osal_tick_trim);

        osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_RTC_TEST_EVT, 100 );
            
        return  ( events ^ SBP_RTC_TEST_EVT );
    }
#endif
    // Discard unknown events
    return 0;
}

/*********************************************************************
 * @fn      simpleBLEPeripheral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLEPeripheral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
    switch ( pMsg->event )
    {  
    default:
        // do nothing
        break;
    }
}
/*********************************************************************
 * @fn      peripheralStateReadRssiCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateReadRssiCB( int8  rssi )
{
//    notifyBuf[15]++;
//    notifyBuf[16]=rssi;
//    notifyBuf[17]=HI_UINT16(g_conn_param_foff);
//    notifyBuf[18]=LO_UINT16(g_conn_param_foff);;
//    notifyBuf[19]=g_conn_param_carrSens;
}

/*********************************************************************
 * @fn      peripheralStateNotificationCB
 *
 * @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
    switch ( newState )
    {
        case GAPROLE_STARTED:
        {
            uint8 ownAddress[B_ADDR_LEN];
            uint8 str_addr[14]={0}; 
            uint8 systemId[DEVINFO_SYSTEM_ID_LEN];
            uint8 initial_advertising_enable = TRUE;//true
        
            GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);
        
            // use 6 bytes of device address for 8 bytes of system ID value
            systemId[0] = ownAddress[0];
            systemId[1] = ownAddress[1];
            systemId[2] = ownAddress[2];
        
            // set middle bytes to zero
            systemId[4] = 0x00;
            systemId[3] = 0x00;
        
            // shift three bytes up
            systemId[7] = ownAddress[5];
            systemId[6] = ownAddress[4];
            systemId[5] = ownAddress[3];
        
            DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);


            osal_memcpy(&str_addr[0],bdAddr2Str(ownAddress),14);
            osal_memcpy(&scanRspData[11],&str_addr[6],8);
            osal_memcpy(&attDeviceName[9],&str_addr[6],8);
        

            GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
            // Set the GAP Characteristics
            GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

            GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
        }
            break;
        
        case GAPROLE_ADVERTISING:
        {
            osal_stop_timerEx(simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT);
            notifyCnt=0;
            notifyInterval = 0;
            LL_PLUS_PerStatsReset();

            uint32_t rcCalCode = (*(volatile uint32_t *) 0x4000f018 & 0x7f)>>1;
            
            LOG("[RTC CNT]%02x %d %d %d\n",rcCalCode,counter_tracking,g_counter_traking_cnt,g_counter_traking_avg);
            if(sysClockChangeFlg)
            {
                LOG("[NVIC RST]\n");
                sysClockChangeFlg=0;
                NVIC_SystemReset();             
            }
         }   
            break;
        
        case GAPROLE_CONNECTED:
            HCI_PPLUS_ConnEventDoneNoticeCmd(simpleBLEPeripheral_TaskID, NULL);
            
            break;
        
        case GAPROLE_CONNECTED_ADV:
            break;      
        case GAPROLE_WAITING:
            break;
        
        case GAPROLE_WAITING_AFTER_TIMEOUT:
            break;
        
        case GAPROLE_ERROR:
            break;
        
        default:
            break;        
    }  
    gapProfileState = newState;

    LOG("[GAP ROLE %d]\n",newState);
     
    VOID gapProfileState;     
}


/*********************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void simpleProfileChangeCB( uint8 paramID )
{
  uint8 newValue[20];
    
  switch( paramID )
  {
    case SIMPLEPROFILE_CHAR5:
      SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR5, newValue );
      LOG("[WRT_ATT] %02x \n",newValue[0]);
      //===============================================================================
      // 0xff reset to connectable adv
      if (newValue[0] == 0xff)
      {

        
         otaAdvIntv = newValue[1];
         LOG("[AdvIntv CONNECT] %04d\n",otaAdvIntv*10);
          // option:
          // 1. reset
          // 2. reset advertisement
          g_current_advType = LL_ADV_CONNECTABLE_UNDIRECTED_EVT;
          pGlobal_config[ADV_CHANNEL_INTERVAL] = 1400;//6250;
          VOID updateAdvData();		  
      }
      else if(newValue[0]==0xfe)
      {

        
         otaAdvIntv = newValue[1];
         LOG("[AdvIntv NONCONN] %04d\n",otaAdvIntv*10);
          // option:
          // 1. reset
          // 2. reset advertisement
          g_current_advType = LL_ADV_NONCONNECTABLE_UNDIRECTED_EVT;
          pGlobal_config[ADV_CHANNEL_INTERVAL] = 600;//6250;
          VOID updateAdvData();		  
      }
      //===============================================================================
      // check per stats
      else if(newValue[0]==0xfd)
      {

        check_PerStatsProcess();	  
      }
      //===============================================================================
      // system clock change
      else if(newValue[0]==0xfc)
      {
      
#if (SYS_CLK_TEST_UCDS)       
           if(      ( newValue[1] == SYS_CLK_DLL_48M || newValue[1]==SYS_CLK_XTAL_16M ) 
                &&  (newValue[2] == CLK_32K_XTAL || newValue[2]== CLK_32K_RCOSC) )
           {
                  
                LOG("[SYS CLK CHG] %d %d\n",newValue[2], newValue[1]);
                flash_erase_ucds(SYS_CLK_TEST_UCDS);

                flash_write_ucds(SYS_CLK_TEST_UCDS, (newValue[2]<<8) | newValue[1] );
                sysClockChangeFlg = 1;
           }
           else
           {
                LOG("[SYS CLK CHG ERR] %d %d\n",newValue[2], newValue[1]);
           }
#else
            LOG("[SYS CLK CHG NO SUPPORT]\n");
#endif
       
      }
      //===============================================================================
      // [0x00 a1 a2 ] : enable notifiy , notifiy intv is a1
      else if(newValue[0]== 0x00 )
      {
        //enable 8 packet per connection interval
        HCI_PPLUS_ExtendTRXCmd(1);
        
        connEvtEndNotify = (newValue[1]&0x80)>>7;
        notifyInterval   = (newValue[1]&0x7f)*5;
        notifyPktNum     = newValue[2]; 
        
        uint16 connIntv;
        GAPRole_GetParameter(GAPROLE_CONN_INTERVAL,&connIntv);
        connIntv = ((connIntv<<2)+connIntv)>>2;//*1.25

        if(connEvtEndNotify>0)
            notifyInterval = connIntv;

        if(notifyInterval>0)
        {
            if(notifyInterval<connIntv)
            {
                notifyPktNum=(notifyInterval<<3)/connIntv;
            }

            if(notifyPktNum==0)
            {
                notifyPktNum=1;
                notifyInterval = (connIntv+1)>>2;
            }

            if(connEvtEndNotify>0)
            {
                HCI_PPLUS_ConnEventDoneNoticeCmd(simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT);
            }
            else
            {
                HCI_PPLUS_ConnEventDoneNoticeCmd(simpleBLEPeripheral_TaskID, NULL);
                osal_start_timerEx( simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT, notifyInterval );
            }
        }
        else
        {
            osal_stop_timerEx(simpleBLEPeripheral_TaskID, SBP_PERIODIC_EVT);
            notifyCnt=0;
        }

        if(connEvtEndNotify)
        {
            LOG("[Notf_Conn] p%d ni%d ci%d\n",notifyPktNum,notifyInterval,connIntv);
        }
        else
        {
            LOG("[Notify   ] p%d ni%d ci%d\n",notifyPktNum,notifyInterval,connIntv);
        }
        
      }
      
      //===============================================================================
      // [0x01 a1 a2 a3 a4 ] : cont config ,a1 
      else if(newValue[0]==0x01)
      {
        otaConnIntvMin      = newValue[1];
        otaConnIntvMax      = newValue[2];
        otaConnIntvLatency  = newValue[3];
        otaConnTimeOut      = newValue[4];

        uint16 desired_min_interval = (otaConnIntvMin<<2)<6 ? 6:(otaConnIntvMin<<2);
        uint16 desired_max_interval = (otaConnIntvMax<<2)<6 ? 6:(otaConnIntvMax<<2);
        uint16 desired_slave_latency = otaConnIntvLatency;
        uint16 desired_conn_timeout = otaConnTimeOut*100;
        uint8 updateConnParams = true;

        LOG("[ConnPara] %04d %04d L%02d T%02d\n",desired_min_interval,
                                                 desired_max_interval,
                                                 desired_slave_latency,
                                                 desired_conn_timeout);

        GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
        GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
        GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
        GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );

        GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_REQ, sizeof( uint8 ), &updateConnParams );
      }
      

      break;

    default:
      // not process other attribute change
      break;
  }
}


/*********************************************************************
 * @fn      updateAdvData
 *
 * @brief   update adv data and change the adv type
 *
 * @param   none
 *
 * @return  none
 */
static void updateAdvData(void)
{
    uint8  new_uuid[IBEACON_UUID_LEN];
    uint16  major;
    uint16  minor;
    uint8   power;
    
    // 1. get the new setting from GATT attributes
    SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR1, new_uuid );
    SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR2, &major );
    SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR3, &minor );
    SimpleProfile_GetParameter( SIMPLEPROFILE_CHAR4, &power );	  
    
    // 2. update adv data storage
    //set UUID
    VOID osal_memcpy(&advertData[9], new_uuid, IBEACON_UUID_LEN);
    // set major
    advertData[25] = LO_UINT16( major );
    advertData[26] = HI_UINT16( major );	  
    // set minor	  
    advertData[27] = LO_UINT16( minor );
    advertData[28] = HI_UINT16( minor );	
    // set power
    advertData[29] = power;
	
    // 3. disconnect all connection
    GAPRole_TerminateConnection();
		
    // 4. close advert
    uint8 initial_advertising_enable = FALSE;		
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );		
        
    // 5. update adv data
    // 5.1 update adv type
    uint8 advType = g_current_advType;    
    GAPRole_SetParameter( GAPROLE_ADV_EVENT_TYPE, sizeof( uint8 ), &advType );	  

    uint16 advInt = otaAdvIntv<<4;
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );

    // 5.2 update advert broadcast
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );	

    // 5.3 set TxPower
    g_rfPhyTxPower = power;
    rf_phy_set_txPower(power);

    // 6. set reset advertisement event, note that GAP/LL will process close adv event in advance
    osal_start_timerEx(simpleBLEPeripheral_TaskID, SBP_RESET_ADV_EVT,5000);    
}

/*********************************************************************
* @fn      bdAddr2Str
*
* @brief   Convert Bluetooth address to string. Only needed when
*          LCD display is used.
*
* @return  none
*/
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;
  
  *pStr++ = '0';
  *pStr++ = 'x';
  
  // Start from end of addr
  pAddr += B_ADDR_LEN;
  
  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }
  
  *pStr = 0;
  
  return str;
}


uint8_t simpleBLEPeripheral_ScanRequestFilterCBack(void)
{
	uint8_t scanReq[32];
    uint8_t scanAddr[6];
    uint8_t scanReqLen;
    scanReqLen=LL_PLUS_GetScanRequestExtendData(scanReq);
    if(scanReqLen>0 )
    {
        if(scanReq[0]%2==0)
        {
            LL_PLUS_SetScanRsqDataByIndex(sizeof ( scanRspData )-1, scanReq[0]);
            return 1;
        }
        else
        {
            return 0;
        }
        
    }
    else
    {
        LL_PLUS_SetScanRsqDataByIndex(sizeof ( scanRspData )-1, 0);
        LL_PLUS_GetScanerAddr(scanAddr);
        return 1;
    }
}

/*********************************************************************
* @fn      simpleBLEPeripheral_MasterRssiScanCBack 
*
* @brief   get master scan addr and rssi
*
* @return  none

*/
uint8_t simpleBLEPeripheral_MasterRssiScanCBack(void)
{
	LL_PLUS_GetScanerAddr(g_scan_info.scan_mac);
	g_scan_info.rssi = LL_PLUS_GetCurrentRSSI();
	osal_set_event(simpleBLEPeripheral_TaskID,SBP_SCAN_RSP_EVT);
	return 1;
}

/*********************************************************************
*********************************************************************/
