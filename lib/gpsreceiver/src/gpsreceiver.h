#ifndef _GPSREC_GPSRECEIVER_H_
#define _GPSREC_GPSRECEIVER_H_

#include <stdbool.h>

/*
 * Global
 */

/** The long plugin name */
#define GPSREC_PLUGIN_NAME "OLSRD GPS Receiver"


/*
 *  Interface
 */

bool init_gpsrec(void);
void close_gpsrec(void);
void gpsrec_error(bool, const char *format, ...) __attribute__ ((format(printf, 2, 3)));

#endif /* _GPSREC_GPSRECEIVER_H_ */
