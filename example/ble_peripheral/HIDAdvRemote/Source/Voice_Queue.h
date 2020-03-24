#ifndef VOICE_QUEUE_H
#define VOICE_QUEUE_H

#include "types.h"

#define VOICE_QUEUE_MAX_LENGTH 100 //300

#define VOICE_MTU_SIZE_FIXED_20_BYTES 1
#define VOICE_REPORT_FRAME_SIZE 100

#define VOICE_BLESEND_FRAME_SIZE 20




typedef struct queue
{
    uint8 VoiceFlg;
    uint8 VoiceSendFlg; //voice buffered to send
    uint8 ReleaseSendFlg; //release key buffered to send
    uint8 ReleaseTimerFlg;//timer to process abnormal flg.

    uint32 VoicePackageSN;
    uint32 queuesize;
    uint32 SendIdx;    //head
    uint32 StoreIdx;   //tail
    uint8 *VoiceQueue;
} Queue;

extern Queue VoiceQueue;

#if VOICE_MTU_SIZE_FIXED_20_BYTES 

extern uint8 VoiceSend_SubIndex;


#endif


extern void  InitQueue(void);
extern void InQueue(uint8 *buffer);
extern uint8 OutQueue(uint8 * send_buf);
extern uint8 IsQueueFull(void);
extern uint8 IsQueueEmpty(void);




#endif
