#ifndef _OLSR_DEBUG_H_
#define _OLSR_DEBUG_H_

#include "olsr_types.h"

#define NUMBER_OF_PRINTS 8

void olsr_buffered_print(const char *, size_t);
void olsr_buffered_printf(size_t, const char *, ...) __attribute__ ((format(printf, 2, 3)));
void olsr_buffered_memdump(const char*, size_t, const void*, size_t);
void olsr_print_buffered_strings(void);

#endif /* _OLSR_DEBUG_H_ */
