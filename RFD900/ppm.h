/*
 * ppm.h
 *
 *  Created on: 21/02/2016
 *      Author: kentm
 */

#ifndef PPM_H_
#define PPM_H_
#include <stdbool.h>

#define MAX_PPM_LEN 16																													// number of ppm channels
#define DMA_BUFF_LEN (MAX_PPM_LEN+1)																						// one extra for start frame

typedef enum{
	PPMModeIn=0,
	PPMModeOut,
	PPMModeLast,
	PPMModeSizeSet = 0xff
} PPMMode_t;

bool PPMWrite(uint8_t *Data, uint16_t Len);																			// write one complete PPM stream to port
bool ReadPPM(uint8_t *Data, uint16_t* Len,uint16_t max_len);										// read any complete incoming stream from PPM port
bool InitPPM(PPMMode_t Mode);																										// Initialise PPM to read or write
bool PPMRecordDefault(void);																										// record the default signal to send now
void PPM_Read_Defaults(uint16_t *Data);																					// read out the desired default values + length

#endif /* PPM_H_ */
