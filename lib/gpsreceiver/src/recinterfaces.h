#ifndef _GPSREC_NETWORKINTERFACES_H
#define _GPSREC_NETWORKINTERFACES_H

/* Plugin includes */

/* OLSR includes */
#include "olsr_types.h"
#include "interfaces.h"
#include "scheduler.h"

/* System includes */
#include <stdbool.h>
#include <net/if.h>

bool create_receive_interface(socket_handler_func handler);
void close_receive_interfaces(void);


#endif /* _GPSREC_NETWORKINTERFACES_H */
