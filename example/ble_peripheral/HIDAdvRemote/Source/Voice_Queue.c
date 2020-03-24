#include "Voice_Queue.h"
#include "osal.h"
#include "log.h"
#include "hidkbd.h"
Queue VoiceQueue;

uint8 Voicebuf[VOICE_QUEUE_MAX_LENGTH*VOICE_REPORT_FRAME_SIZE]= {0,1,23};

#if VOICE_MTU_SIZE_FIXED_20_BYTES

uint8 VoiceSend_SubIndex=0;


#endif



/**
 * @brief init voice queue
 * @param uint8 * voice_buf--Buf For queue
 * @return none
 */
void InitQueue(void)
{
    VoiceQueue.queuesize = VOICE_QUEUE_MAX_LENGTH;
    VoiceQueue.SendIdx   = 0;
    VoiceQueue.StoreIdx  = 0;
    VoiceQueue.VoicePackageSN = 0;
    //VoiceQueue.StoreCnt  = 0;
    VoiceQueue.VoiceFlg  = TRUE;
    VoiceQueue.VoiceSendFlg = FALSE;
    VoiceQueue.ReleaseSendFlg = FALSE;
    VoiceQueue.ReleaseTimerFlg = FALSE;
    VoiceQueue.VoiceQueue = (uint8 *)Voicebuf;
}



/**
 * @brief in voice queue
 * @param buffer - buffer data to be stored
 * @return none
 */
void InQueue(uint8 *buffer)
{
    uint32 Tail;
#if VOICE_MTU_SIZE_FIXED_20_BYTES
    Tail = (VoiceQueue.StoreIdx + 1) % VoiceQueue.queuesize;

    if (Tail == VoiceQueue.SendIdx)
    {
        LOG("Voice Queue is full\n\r");
    }
    else
    {

        osal_memcpy(&VoiceQueue.VoiceQueue[VoiceQueue.StoreIdx * VOICE_REPORT_FRAME_SIZE],buffer, VOICE_REPORT_FRAME_SIZE);
        VoiceQueue.StoreIdx = Tail;
        VoiceQueue.VoicePackageSN++;
        //LOG("Voice InQueue\n\r");
    }



#else


#endif
}

/**
 * @brief out voice queue
 * @param uint8 * send_buf
 * @return none
 */
uint8 OutQueue(uint8 * send_buf)
{
    if (VoiceQueue.StoreIdx == VoiceQueue.SendIdx)
    {
        LOG("Voice Queue is empty\n\r");
#if VOICE_FROM_FLASH
        VoiceQueue.SendIdx=0; //teddy add for test，set data transf continue
#endif
        return 1;
    }
    else
    {
        //SendData
        //ProfileAPI_SendData(gHIDServiceId, GATT_SRV_HID_VENDOR_INPUT_INDEX_2, VoiceQueue.VoiceQueue + VoiceQueue.SendIdx * VOICE_REPORT_FRAME_SIZE, VOICE_REPORT_FRAME_SIZE);
        osal_memcpy(send_buf, VoiceQueue.VoiceQueue+VoiceQueue.SendIdx * VOICE_REPORT_FRAME_SIZE, VOICE_REPORT_FRAME_SIZE);
        VoiceQueue.SendIdx = (VoiceQueue.SendIdx + 1) % VoiceQueue.queuesize;
        return 0;
    }
}

/**
 * @brief check if voice queue is full
 * @param none
 * @return true or false
 */
uint8 IsQueueFull(void)
{
    if ((VoiceQueue.StoreIdx + 1) % VoiceQueue.queuesize == VoiceQueue.SendIdx)
    {
        //DBG_BUFFER(MODULE_DRIVERTASK, LEVEL_INFO, "Voice Queue is full.",0);
        return 1;
    }
    else
    {
        return 0;
    }
}
/**
 * @brief check if voice queue is empty
 * @param none
 * @return true or false
 */
uint8 IsQueueEmpty(void)
{
    if (VoiceQueue.StoreIdx == VoiceQueue.SendIdx)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
