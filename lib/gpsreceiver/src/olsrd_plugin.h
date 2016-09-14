#ifndef _GPSREC_OLSRD_PLUGIN_H_
#define _GPSREC_OLSRD_PLUGIN_H_

/* Plugin includes */
#include "configuration.h"

/* OLSRD includes */
#include "../../../src/olsrd_plugin.h"
#include "olsrd_plugin.h"

/* System includes */
#include <stddef.h>

/** The interface version supported by the plugin */
#define GPSREC_INTERFACE_VERSION 5

int olsrd_plugin_init(void);
int olsrd_plugin_interface_version(void);

/**
 The plugin parameter configuration, containing the parameter names, pointers
 to their setters, and an optional data pointer that is given to the setter
 when it is called.
 */

static const struct olsrd_plugin_parameters plugin_parameters[] = {
  { .name = GPSREC_GPS_IF_NAME, .set_plugin_parameter = &add_gps_interface, .data = NULL},
  { .name = GPSREC_GPS_ADDR_NAME, .set_plugin_parameter = &set_gps_addr, .data = NULL},
  { .name = GPSREC_GPS_PORT_NAME, .set_plugin_parameter = &set_gps_port, .data = NULL}
};

#endif /* _GPSREC_OLSRD_PLUGIN_H_ */
