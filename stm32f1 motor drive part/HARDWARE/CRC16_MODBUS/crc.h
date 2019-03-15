#ifndef _CRC_H
#define _CRC_H

#include "sys.h"

unsigned short ModBusCRC (unsigned char *ptr,unsigned char size);
uint16_t CRC16(uint8_t *puchMsg, uint16_t usDataLen);


#endif
