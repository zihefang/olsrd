#ifndef _GPSREC_CONFIGURATION_H_
#define _GPSREC_CONFIGURATION_H_

/* Plugin includes */

/* OLSR includes */
#include "../../../src/olsrd_plugin.h"
#include "olsr_types.h"

/* System includes */
#include <stdbool.h>
#include <stddef.h>

/** The name of the receive GPS interfaces plugin parameter */
#define GPSREC_GPS_IF_NAME "GpsInterface"

bool is_gps_interface(const char *if_name);
int add_gps_interface(const char *value, void *, set_plugin_parameter_addon);
unsigned int get_gps_interfaces_count(void);
unsigned char * get_gps_interface_name(unsigned int index);

/** The name of the receive multicast address plugin parameter */
#define GPSREC_GPS_ADDR_NAME "GpsMulticastAddr"
#define GPSREC_GPS_ADDR_4_DEFAULT "224.0.0.224"
#define GPSREC_GPS_ADDR_6_DEFAULT "FF02:0:0:0:0:0:0:1"

union olsr_sockaddr * get_gps_addr(void);
int set_gps_addr(const char *value, void *data, set_plugin_parameter_addon addon);

/** The name of the receive multicast port plugin parameter */
#define GPSREC_GPS_PORT_NAME "GpsPort"
#define GPSREC_GPS_PORT_DEFAULT 2240

uint16_t get_gps_port(void);
int set_gps_port(const char *value, void *data, set_plugin_parameter_addon addon);

/*
 * Check Functions
 */

unsigned int check_config(void);

#endif /* _GPSREC_CONFIGURATION_H_ */
