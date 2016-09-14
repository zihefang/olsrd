#include "gpsreceiver.h"

/* Plugin includes */
#include "recinterfaces.h"
#include "configuration.h"

/* NMEA lib includes */
#include "nmea/parser.h"
#include "nmea/info.h"
#include "nmea/context.h"
#include "nmea/sentence.h"

/* OLSRD includes */
#include "olsr.h"
#include "debug.h"
#include "olsr_protocol.h"
#include "olsr_types.h"
#include "ipcalc.h"
#include "net_olsr.h"
#include "parser.h"
#include "log.h"
#include "lq_plugin_gps_lite.h"

/* System includes */
#include <time.h>

/** The size of the buffer in which the received NMEA string is stored */
#define BUFFER_SIZE_NMEA 2048

/** The NMEA string parser */
static nmeaPARSER nmea_parser;

/** local NMEA info */
static nmeaINFO nmea_info;


/** When false, use olsr_printf in gpsrec_error, otherwise use olsr_syslog */
static bool gpsrec_error_use_syslog = false;

/**
 Report a plugin error.

 @param use_errno
 when true then errno is used in the error message; the error reason is also
 reported.
 @param format
 a pointer to the format string
 @param ...
 arguments to the format string
 */
void
gpsrec_error(bool use_errno, const char *format, ...) {
  char str[256];
  const char *colon;
  const char *str_err;

  if ((format == NULL) || (*format == '\0')) {
    str[0] = '\0';
    colon = "";
    if (!use_errno) {
      str_err = "Unknown error";
    } else {
      str_err = strerror(errno);
    }
  } else {
    va_list arglist;

    va_start(arglist, format);
    vsnprintf(str, sizeof(str), format, arglist);
    va_end(arglist);

    if (use_errno) {
      colon = ": ";
      str_err = strerror(errno);
    } else {
      colon = "";
      str_err = "";
    }
  }

  if (!gpsrec_error_use_syslog)
    olsr_printf(0, "%s: %s%s%s\n", GPSREC_PLUGIN_NAME, str, colon, str_err);
  else
    olsr_syslog(OLSR_LOG_ERR, "%s: %s%s%s\n", GPSREC_PLUGIN_NAME, str, colon, str_err);
}

static void
gpsrec_nmea_error(const char *str, int str_size __attribute__((unused))) {
  gpsrec_error(false, "NMEA library error: %s", str);
}


/**
 Called by OLSR core when a packet for the plugin is received from the non-OLSR
 network. 

 @param fd
 the socket file descriptor on which the packet is received
 @param data
 a pointer to the network interface structure on which the packet was received
 @param flags
 unused
 */
static void
packet_received_from_gps(int fd, void *data __attribute__ ((unused)), unsigned int flags __attribute__ ((unused))) {
  unsigned char buff[BUFFER_SIZE_NMEA];
  ssize_t count;
  union olsr_sockaddr sender;
  socklen_t sender_size = sizeof(sender);

  if (fd < 0)
    return;

  /* Receive the captured Ethernet frame */
  memset(&sender, 0, sender_size);
  errno = 0;
  count = recvfrom(fd, &buff[0], (sizeof(buff) - 1), 0,
      (struct sockaddr *)&sender, &sender_size);
  if (count < 0) {
    gpsrec_error(true, "Receive error in %s, ignoring message.", __func__);
    return;
  }

  /* make sure the string is null-terminated */
  buff[count] = '\0';

  /* we have the received string in the buff now */

  /*static const char * rxBufferPrefix = "$GP";
  static const size_t rxBufferPrefixLength = 3;*/

  /* do not process when the message does not start with $GP */
  if (count < 3 || strncmp((char *) buff, "$GP", 3) != 0) {
    return;
  }

  nmea_parse(&nmea_parser, (char *) buff, count, &nmea_info);
  if (nmea_info.smask == GPNON) {
    return;
  }
  nmea_INFO_sanitise(&nmea_info);
  nmea_INFO_unit_conversion(&nmea_info);
  lq_gps_lite_update_position((float) nmea_info.lat, (float) nmea_info.lon, (int16_t) nmea_info.elv);
}


/**
 Initialise the plugin: check the configuration, initialise the NMEA parser,
 create network interface sockets, hookup the plugin to OLSR and setup data
 that can be setup in advance.

 @return
 - false upon failure
 - true otherwise
 */
bool
init_gpsrec(void) {
  if (!check_config()) {
    gpsrec_error(false, "Invalid configuration");
    goto error;
  }

  if (!nmea_parser_init(&nmea_parser)) {
    gpsrec_error(false, "Could not initialise NMEA parser");
    return false;
  }
  nmea_context_set_error_func(&gpsrec_nmea_error);

  /*
   * Creates receive and transmit sockets and register the receive sockets
   * with the OLSR stack
   */
  if (!create_receive_interface(&packet_received_from_gps)) {
    gpsrec_error(false, "Could not create require network interfaces");
    goto error;
  }

  /* switch to syslog logging, load was succesful */
  gpsrec_error_use_syslog = !olsr_cnf->no_fork;

  return true;

  error: close_gpsrec();
  return false;
}

/**
 Stop the plugin: shut down all created network interface sockets and destroy
 the NMEA parser.
 */
void
close_gpsrec(void) {
  nmea_parser_destroy(&nmea_parser);
  close_receive_interfaces();
}
