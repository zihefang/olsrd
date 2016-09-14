#include "recinterfaces.h"

/* Plugin includes */
#include "gpsreceiver.h"
#include "configuration.h"

/* OLSRD includes */
#include "olsr.h"
#include "interfaces.h"

/* System includes */
#include <assert.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>

/** A list of non_olsr_interface objects, used for non-OLSR interfaces */
struct non_olsr_interface {
    int socket_fd;
    char name[IFNAMSIZ + 1];
    socket_handler_func handler;
    struct non_olsr_interface* next;
};

/**
 Get the IPv4 address of an interface

 @param if_name
 the name of the interface
 @param ifr
 the buffer in which to write the IPv4 address

 @return
 - the pointer to the IPv4 address (inside ifr)
 - NULL on failure
 */
static struct in_addr *
get_ipv4_addr(const char * if_name, struct ifreq *ifr) {
  int fd;
  int cpySize;

  assert(if_name != NULL);
  assert(strlen(if_name) <= IFNAMSIZ);
  assert(ifr != NULL);

  fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) {
    gpsrec_error(true, "%s@%u: socket error: %s", __FILE__, __LINE__, strerror(errno));
    return NULL;
  }

  ifr->ifr_addr.sa_family = AF_INET;
  memset(ifr->ifr_name, 0, sizeof(ifr->ifr_name));
  cpySize = (strlen(if_name) < sizeof(ifr->ifr_name)) ? strlen(if_name)
      : sizeof(ifr->ifr_name);
  strncpy(ifr->ifr_name, if_name, cpySize);

  errno = 0;
  if (ioctl(fd, SIOCGIFADDR, ifr) < 0) {
    gpsrec_error(true, "%s@%u: ioctl(SIOCGIFADDR) error", __FILE__, __LINE__);
    close(fd);
    return NULL;
  }

  close(fd);

  return &((struct sockaddr_in *)(void *) &ifr->ifr_addr)->sin_addr;
}


/** The list of network interface objects, receiving GPS NMEA sentences */
static struct non_olsr_interface* non_olsr_interfaces_head = NULL;

/** Pointer to the last network interface object, receiving GPS NMEA sentences */
static struct non_olsr_interface* last_non_olsr_interface = NULL;

/**
 Create a receive socket for a network interface

 @param network_interface
 The network interface object. This function expects it to be filled with all
 information, except for the socket descriptor.
 @param handler
 The function that handles reception of data on the network interface
 @param multicast_addr
 The receive multicast address

 @return
 - the socket descriptor (>= 0)
 - -1 if an error occurred
 */
static int
create_receive_socket(const char * if_name, union olsr_sockaddr * multicast_addr) {
  int ip_family;
  int ip_protocol;
  int ip_multicast_loop;
  int ip_add_membership;

  union olsr_sockaddr address;
  void * addr;
  size_t add_size;

  int socket_fd = -1;
  int socket_reuse = 1;
  int multicast_loop = 1;

  assert(strncmp(&if_name[0], "", IFNAMSIZ+1) != 0);

  memset(&address, 0, sizeof(address));
  if (multicast_addr->in.sa_family == AF_INET) {
    assert(multicast_addr->in4.sin_addr.s_addr != INADDR_ANY);

    ip_family = AF_INET;
    ip_protocol = IPPROTO_IP;
    ip_multicast_loop = IP_MULTICAST_LOOP;
    ip_add_membership = IP_ADD_MEMBERSHIP;

    address.in4.sin_family = ip_family;
    address.in4.sin_addr.s_addr = INADDR_ANY;
    address.in4.sin_port = get_gps_port();
    addr = &address.in4;
    add_size = sizeof(struct sockaddr_in);
  } else {
    assert(multicast_addr->in6.sin6_addr.s6_addr != in6addr_any.s6_addr);

    ip_family = AF_INET6;
    ip_protocol = IPPROTO_IPV6;
    ip_multicast_loop = IPV6_MULTICAST_LOOP;
    ip_add_membership = IPV6_ADD_MEMBERSHIP;

    address.in6.sin6_family = ip_family;
    address.in6.sin6_addr = in6addr_any;
    address.in6.sin6_port = get_gps_port();
    addr = &address.in6;
    add_size = sizeof(struct sockaddr_in6);
  }

  /* Create a datagram socket on which to receive. */
  errno = 0;
  socket_fd = socket(ip_family, SOCK_DGRAM, 0);
  if (socket_fd < 0) {
    gpsrec_error(true, "Could not create a receive socket for interface %s", if_name);
    goto bail;
  }

  /* Enable SO_REUSEADDR to allow multiple applications to receive the same
   * multicast messages */
  errno = 0;
  if (setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, &socket_reuse,
      sizeof(socket_reuse)) < 0) {
    gpsrec_error(true, "Could not set the reuse flag on the receive socket for"
      " interface %s", if_name);
    goto bail;
  }

  /* Bind to the proper port number with the IP address INADDR_ANY
   * (INADDR_ANY is really required here, do not change it) */
  errno = 0;
  if (bind(socket_fd, addr, add_size) < 0) {
    gpsrec_error(true, "Could not bind the receive socket for interface"
      " %s to port %u", if_name, ntohs(get_gps_port()));
    goto bail;
  }

  /* Enable multicast local loopback */
  errno = 0;
  if (setsockopt(socket_fd, ip_protocol, ip_multicast_loop, &multicast_loop,
      sizeof(multicast_loop)) < 0) {
    gpsrec_error(true, "Could not enable multicast loopback on the"
      " receive socket for interface %s", if_name);
    goto bail;
  }

  /* Join the multicast group on the local interface. Note that this
   * ADD_MEMBERSHIP option must be called for each local interface over
   * which the multicast datagrams are to be received. */
  if (ip_family == AF_INET) {
    struct ip_mreq mc_settings;

    struct ifreq ifr;
    struct in_addr * if_addr = get_ipv4_addr(if_name, &ifr);
    if (!if_addr) {
      gpsrec_error(true, "Could not get interface address of %s", if_name);
      goto bail;
    }

    (void) memset(&mc_settings, 0, sizeof(mc_settings));
    mc_settings.imr_multiaddr = multicast_addr->in4.sin_addr;
    mc_settings.imr_interface = *if_addr;
    errno = 0;
    if (setsockopt(socket_fd, ip_protocol, ip_add_membership,
        &mc_settings, sizeof(mc_settings)) < 0) {
      gpsrec_error(true, "Could not subscribe interface %s to the configured"
        " multicast group", if_name);
      goto bail;
    }
  } else {
    struct ipv6_mreq mc6_settings;
    (void) memset(&mc6_settings, 0, sizeof(mc6_settings));
    mc6_settings.ipv6mr_multiaddr = multicast_addr->in6.sin6_addr;
    mc6_settings.ipv6mr_interface = if_nametoindex(if_name);
    errno = 0;
    if (setsockopt(socket_fd, ip_protocol, ip_add_membership,
        &mc6_settings, sizeof(mc6_settings)) < 0) {
      gpsrec_error(true, "Could not subscribe interface %s to the configured"
        " multicast group", if_name);
      goto bail;
    }
  }

  return socket_fd;

  bail: if (socket_fd >= 0) {
    close(socket_fd);
  }
  return -1;

}

/**
 Create a receive interface and add it to the list of receive network interface
 objects

 @param handler
 the function that handles reception of data on the network interface

 @return
 - true on success
 - false on failure
 */
bool
create_receive_interface(socket_handler_func handler) {
  union olsr_sockaddr* multicast_addr = get_gps_addr();
  unsigned int count = get_gps_interfaces_count();

  unsigned char* if_name;
  struct non_olsr_interface* network_interface;
  int socket_fd;
  int failed_interfaces = 0;

  while (count--) {
    if_name = get_gps_interface_name(count);
    if(if_name == NULL)
    {
      failed_interfaces++;
      gpsrec_error(true, "Could not get name of interface #%d", count);
      continue;
    }
    network_interface = olsr_malloc(sizeof(struct non_olsr_interface), "non_olsr_interface (GPSREC)");
    if(network_interface == NULL)
    {
      failed_interfaces++;
      continue;
    }
    memcpy(network_interface->name, (char*) if_name, sizeof(network_interface->name));
    network_interface->name[IFNAMSIZ] = '\0';
    network_interface->handler = NULL;
    network_interface->next = NULL;

    /* network_interface needs to be filled in when calling create_receive_socket */
    socket_fd = create_receive_socket(network_interface->name, multicast_addr);
    if (socket_fd < 0) {
      failed_interfaces++;
      free(network_interface);
      continue;
    }
    network_interface->socket_fd = socket_fd;
    network_interface->handler = handler;
    add_olsr_socket(socket_fd, handler, NULL, NULL, SP_PR_READ);  

    /* Add new object to the end of the global list. */
    if (non_olsr_interfaces_head == NULL) {
      non_olsr_interfaces_head = network_interface;
      last_non_olsr_interface = network_interface;
    } else {
      last_non_olsr_interface->next = network_interface;
      last_non_olsr_interface = network_interface;
    }
  }

  if(failed_interfaces == 0)
    return true;
  gpsrec_error(true, "Could not create %d receive interfaces among the %d required", failed_interfaces, get_gps_interfaces_count());
  return false;
}


/**
 Close and cleanup the network interfaces in the given list

 @param network_interface
 the list of network interface to close and clean up
 */
void
close_receive_interfaces(void) {
  struct non_olsr_interface* next_iface = non_olsr_interfaces_head;
  struct non_olsr_interface* iterated_iface;

  while (next_iface != NULL) {
    iterated_iface = next_iface;
    if (iterated_iface->socket_fd >= 0) {
      if (iterated_iface->handler)
        remove_olsr_socket(iterated_iface->socket_fd,
            iterated_iface->handler, NULL);
      close(iterated_iface->socket_fd);
      iterated_iface->socket_fd = -1;
    }
    next_iface = iterated_iface->next;
    iterated_iface->next = NULL;
    free(iterated_iface);
  }
  non_olsr_interfaces_head = NULL;
}
