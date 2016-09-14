#include "configuration.h"

/* Plugin includes */
#include "gpsreceiver.h"

/* OLSR includes */
#include "olsrd_plugin.h"
#include "olsr_types.h"
#include "olsr_protocol.h"

/* System includes */
#include <assert.h>
#include <unistd.h>
#include <limits.h>
#include <stdbool.h>
#include <stddef.h>
#include <net/if.h>
#include <arpa/inet.h>

#ifdef __ANDROID_API__
typedef __in_port_t in_port_t;
#endif /* __ANDROID_API__ */

/*
 * Helper functions
 */

/**
 Get the port in an OLSR socket address

 @param addr
 A pointer to OLSR socket address
 @param default_port
 The default port (in host byte order), returned when addr is NULL
 @return
 The port (in network byte order)
 */
static inline in_port_t
get_olsr_sockaddr_port(union olsr_sockaddr * addr, in_port_t default_port) {
  if (!addr) {
    return htons(default_port);
  }
  if (addr->in.sa_family == AF_INET) {
    return addr->in4.sin_port;
  } else {
    return addr->in6.sin6_port;
  }
}

/**
 Set the port in an OLSR socket address

 @param addr
 A pointer to OLSR socket address
 @param port
 The port (in network byte order)
 */
static inline void
set_olsr_sockaddr_port(union olsr_sockaddr * addr, in_port_t port) {
  if (!addr) {
    return;
  }
  if (addr->in.sa_family == AF_INET) {
    addr->in4.sin_port = port;
  } else {
    addr->in6.sin6_port = port;
  }
}

/**
 Determine whether an IP address (v4 or v6) is a multicast address.

 @param addr
 An IP address (v4 or v6)

 @return
 - true when the address is a multicast address
 - false otherwise
 */
static inline bool
is_multicast(union olsr_sockaddr *addr) {
  assert(addr != NULL);
  return addr->in.sa_family == AF_INET ?
    IN_MULTICAST(ntohl(addr->in4.sin_addr.s_addr)) :
    IN6_IS_ADDR_MULTICAST(&addr->in6.sin6_addr);
}

static bool
read_uint16(const char * parameter_name, const char * str, uint16_t * dst) {
  char * end = NULL;
  unsigned long value;

  assert(parameter_name != NULL);
  assert(str != NULL);
  assert(dst != NULL);

  errno = 0;
  value = strtoul(str, &end, 10);

  if (!((end != str) && (*str != '\0') && (*end == '\0'))) {
    /* invalid conversion */
    gpsrec_error(false, "Value of parameter %s (%s) could not be converted to a number", parameter_name, str);
    return false;
  }

  if (value > 65535) {
    gpsrec_error(false, "Value of parameter %s (%lu) is outside of valid range 0-65535", parameter_name, value);
    return false;
  }

  *dst = (uint16_t) value;
  return true;
}

static bool
read_ip_addr(const char * parameter_name, const char * str, in_port_t default_port,
    union olsr_sockaddr * dst, bool * is_dst_set) {
  union olsr_sockaddr ip;
  int conversion;

  assert(parameter_name != NULL);
  assert(str != NULL);
  assert(dst != NULL);
  assert(is_dst_set != NULL);

  /* try IPv4 first */
  memset(&ip, 0, sizeof(ip));
  ip.in.sa_family = AF_INET;
  conversion = inet_pton(ip.in.sa_family, str, &ip.in4.sin_addr);

  /* now try IPv6 if IPv4 conversion was not successful */
  if (conversion != 1) {
    memset(&ip, 0, sizeof(ip));
    ip.in.sa_family = AF_INET6;
    conversion = inet_pton(ip.in.sa_family, str, &ip.in6.sin6_addr);
  }

  if (conversion != 1) {
    gpsrec_error((conversion == -1) ? true : false,
    "Value of parameter %s (%s) is not an IP address", parameter_name, str);
    return false;
  }

  if (!*is_dst_set) {
    set_olsr_sockaddr_port(&ip, htons(default_port));
  } else {
    set_olsr_sockaddr_port(&ip, get_olsr_sockaddr_port(dst, default_port));
  }

  *dst = ip;
  *is_dst_set = true;
  return true;
}



/*
 * Note:
 * Setters must return true when an error is detected, false otherwise
 */

#define MAX_INTERFACES 32

static unsigned char gps_interfaces[MAX_INTERFACES][IFNAMSIZ + 1];
static unsigned int gps_interfaces_count = 0;

/**
 @param if_name
 The interface name to check

 @return
 - true when the given interface name is configured as a receive non-OLSR
 interface
 - false otherwise
 */
bool
is_gps_interface(const char *if_name) {
  unsigned int i;
  assert (if_name != NULL);

  for (i = 0; i < gps_interfaces_count; i++) {
    if (strncmp((char *) &gps_interfaces[i][0], if_name, IFNAMSIZ + 1) == 0) {
      return true;
    }
  }
  return false;
}

int
add_gps_interface(const char *value, void *data __attribute__ ((unused)),
    set_plugin_parameter_addon addon __attribute__ ((unused))) {
  size_t valueLength;

  if (gps_interfaces_count >= MAX_INTERFACES) {
    gpsrec_error(false, "Can't configure more than %u receive interfaces", MAX_INTERFACES);
    return true;
  }

  assert (value != NULL);

  valueLength = strlen(value);
  if (valueLength > IFNAMSIZ) {
    gpsrec_error(false, "Value of parameter %s (%s) is too long,"
      " maximum length is %u, current length is %lu",
        GPSREC_GPS_IF_NAME, value, IFNAMSIZ, (long unsigned int)valueLength);
    return true;
  }

  if (!is_gps_interface(value)) {
    strcpy((char *) &gps_interfaces[gps_interfaces_count][0],
        value);
    gps_interfaces_count++;
  }

  return false;
}

/**
 * @return the number of configured non-olsr receive interfaces
 */
unsigned int
get_gps_interfaces_count(void) {
  return gps_interfaces_count;
}

/**
 * @param idx the index of the configured non-olsr receive interface
 * @return the index-th interface name
 */
unsigned char *
get_gps_interface_name(unsigned int idx) {
  return &gps_interfaces[idx][0];
}


/*
 * gps_addr + gps_port
 */

/** The rx multicast address */
static union olsr_sockaddr gps_addr;

/** True when the rx multicast address is set */
static bool is_gps_addr_set = false;

/**
 @return
 The receive multicast address (in network byte order). Sets both the address
 and the port to their default values when the address was not yet set.
 */
union olsr_sockaddr*
get_gps_addr(void) {
  if (!is_gps_addr_set) {
    set_gps_addr((olsr_cnf->ip_version == AF_INET) ? GPSREC_GPS_ADDR_4_DEFAULT : GPSREC_GPS_ADDR_6_DEFAULT,
        NULL, ((set_plugin_parameter_addon) {.pc = NULL}));
  }
  return &gps_addr;
}

int
set_gps_addr(const char *value, void *data __attribute__ ((unused)), set_plugin_parameter_addon addon __attribute__ ((unused))) {
  static const char * value_name = GPSREC_GPS_ADDR_NAME;

  if (!read_ip_addr(value_name, value, GPSREC_GPS_PORT_DEFAULT, &gps_addr, &is_gps_addr_set)) {
      return true;
  }

  if (!is_multicast(&gps_addr)) {
    gpsrec_error(false, "Value of parameter %s (%s) is not a multicast address",
        value_name, value);
    return true;
  }

  return false;
}

/**
 @return
 The receive multicast port (in network byte order)
 */
uint16_t
get_gps_port(void) {
  return get_olsr_sockaddr_port(get_gps_addr(), GPSREC_GPS_PORT_DEFAULT);
}

int
set_gps_port(const char *value, void *data __attribute__ ((unused)), set_plugin_parameter_addon addon __attribute__ ((unused))) {
  static const char * value_name = GPSREC_GPS_PORT_NAME;
  uint16_t gps_port_new;

  if (!read_uint16(value_name, value, &gps_port_new)) {
    return true;
  }

  if (gps_port_new < 1) {
    gpsrec_error(false, "Value of parameter %s (%u) is outside of valid range 1-65535", value_name, gps_port_new);
    return true;
  }

  set_olsr_sockaddr_port(get_gps_addr(), htons((in_port_t) gps_port_new));

  return false;
}



/*
 * Check Functions
 */

/**
 Check the configuration for consistency and validity.

 @return
 - true when the configuration is consistent and valid
 - false otherwise
 */
unsigned int
check_config(void) {
  int retval = true;

  if (gps_interfaces_count == 0) {
    gpsrec_error(false, "No receive non-OLSR interfaces configured");
    retval = false;
  }

  return retval;
}
