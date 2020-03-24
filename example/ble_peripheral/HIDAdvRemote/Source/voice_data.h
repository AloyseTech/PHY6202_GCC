#ifndef VOICE_DATA_H
#define VOICE_DATA_H

#include "types.h"

//#define VOICE_RAW_LEN  32640

#define VOICE_RAW_LEN  12288


#define VOICE_RAW_FRAM_LEN 384

extern uint16 voiceRaw_index;
extern uint16 header_index;  


/******************************************************************************************************
*************************************今天的天气*******************************************************
******************************************************************************************************/

extern const unsigned char Voice_data[VOICE_RAW_LEN];
#endif 
