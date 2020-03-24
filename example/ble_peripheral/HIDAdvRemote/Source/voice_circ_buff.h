//*****************************************************************************
// voice_circ_buff.h
//

//*****************************************************************************

#ifndef __CIRCULAR_BUFFER_API_H__
#define __CIRCULAR_BUFFER_API_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif
  
/****************************************************************************/
/*				TYPEDEFS										*/
/****************************************************************************/
typedef struct CircularBuffer
{   
	  unsigned char  postFlag;
    unsigned short TransThreshold_size;
    unsigned char *pucReadPtr;
    unsigned char *pucWritePtr;
    unsigned char *pucBufferStartPtr;
    unsigned long ulBufferSize;
    unsigned char *pucBufferEndPtr;
}tCircularBuffer;

#ifndef TRUE
#define TRUE                    1
#endif

#ifndef FALSE
#define FALSE                   0
#endif

//*****************************************************************************
//
// Define a boolean type, and values for true and false.
//
//*****************************************************************************
typedef unsigned int tboolean;

/****************************************************************************/
/*		        FUNCTION PROTOTYPES							*/
/****************************************************************************/
extern tCircularBuffer *InitCircularBuffer(tCircularBuffer*pTempBuff, unsigned char *userbuf,unsigned long ulBufferSize,unsigned short threshold_size);
extern tCircularBuffer* CreateCircularBuffer(unsigned long ulBufferSize);
extern void DestroyCircularBuffer(tCircularBuffer *pCircularBuffer);
extern unsigned char* GetReadPtr(tCircularBuffer *pCircularBuffer);
extern unsigned char* GetWritePtr(tCircularBuffer *pCircularBuffer);
extern long FillBuffer(tCircularBuffer *pCircularBuffer,
                       unsigned char *pucBuffer, unsigned int uiBufferSize);
extern void UpdateReadPtr(tCircularBuffer *pBuffer, unsigned int uiDataSize);
extern void UpdateWritePtr(tCircularBuffer *pCircularBuffer,
                           unsigned int uiPacketSize);
extern long ReadBuffer(tCircularBuffer *pCircularBuffer,
                       unsigned char *pucBuffer, unsigned int uiDataSize);
extern long FillZeroes(tCircularBuffer *pCircularBuffer,
					   unsigned int uiPacketSize);
extern unsigned int GetBufferSize(tCircularBuffer *pCircularBuffer);
extern unsigned int GetBufferEmptySize(tCircularBuffer *pCircularBuffer);
extern tboolean IsBufferEmpty(tCircularBuffer *pCircularbuffer);
extern tboolean IsBufferSizeFilled(tCircularBuffer *pCircularBuffer,
                             unsigned long ulThresholdHigh);
extern tboolean IsBufferVacant(tCircularBuffer *pCircularBuffer,
                               unsigned long ulThresholdLow);

#ifdef __cplusplus
}
#endif

#endif // __CIRCULAR_BUFFER_API_H__

