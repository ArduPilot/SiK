/*
 * printfl.h
 *
 *  Created on: 20/11/2015
 *      Author: kentm
 */

#ifndef PRINTFL_H_
#define PRINTFL_H_
#include <stdarg.h>

void	vprintfl(const char * fmt, va_list ap);
#define	vprintf(_fmt, _ap)	vprintfl(_fmt, _ap)		///< avoid fighting with the library vprintf() prototype

/// Alternate printf implementation
void	printfl(const char *fmt, ...);
#define printf(_fmt, args...)	printfl(_fmt, ##args)		///< avoid fighting with the library printf() prototype

/// start a capture of printf data
void printf_start_capture(uint8_t *buf, uint8_t size);

/// end printf capture, returning number of bytes that have been captured
uint8_t printf_end_capture(void);


#endif /* PRINTFL_H_ */
