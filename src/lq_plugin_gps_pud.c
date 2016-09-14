
#include "tc_set.h"
#include "link_set.h"
#include "olsr_spf.h"
#include "lq_packet.h"
#include "packet.h"
#include "olsr.h"
#include "lq_plugin_gps_pud.h"
#include "scheduler.h"
#include "mid_set.h"
#include "common/avl.h"
#include <string.h>
#include <math.h>

static void lq_initialize_gps_pud(void);
static olsr_linkcost lq_calc_cost_gps_pud(void *lq);
static void lq_packet_loss_worker_gps_pud(struct link_entry *link, void *lq, bool lost);
static void lq_memorize_foreign_hello_gps_pud(void *local, void *foreign);
static int lq_serialize_hello_lq_pair_gps_pud(unsigned char *buff, void *lq);
static void lq_deserialize_hello_lq_pair_gps_pud(const uint8_t ** curr, void *lq);
static int lq_serialize_tc_lq_pair_gps_pud(unsigned char *buff, void *lq);
static void lq_deserialize_tc_lq_pair_gps_pud(const uint8_t ** curr, void *lq);
static void lq_copy_link2tc_gps_pud(void *target, void *source);
static void lq_clear_gps_pud(void *target);
static const char *lq_print_gps_pud(void *ptr, char separator, struct lqtextbuffer *buffer);
static const char *lq_print_cost_gps_pud(olsr_linkcost cost, struct lqtextbuffer *buffer);


/* Default lq plugin settings */
struct lq_handler lq_etx_gps_pud_handler = {
  &lq_initialize_gps_pud,

  &lq_calc_cost_gps_pud,
  &lq_calc_cost_gps_pud,

  &lq_packet_loss_worker_gps_pud,
  &lq_memorize_foreign_hello_gps_pud,
  &lq_copy_link2tc_gps_pud,
  &lq_copy_link2tc_gps_pud,

  &lq_clear_gps_pud,
  &lq_clear_gps_pud,
  &lq_clear_gps_pud,

  &lq_serialize_hello_lq_pair_gps_pud,
  &lq_serialize_tc_lq_pair_gps_pud,
  &lq_deserialize_hello_lq_pair_gps_pud,
  &lq_deserialize_tc_lq_pair_gps_pud,

  &olsr_default_serialize_global_lq,
  &olsr_default_serialize_global_lq,
  &olsr_default_deserialize_global_lq,
  &olsr_default_deserialize_global_lq,

  &lq_print_gps_pud,
  &lq_print_gps_pud,
  &lq_print_cost_gps_pud,

  sizeof(struct lq_gps_pud),
  sizeof(struct lq_gps_pud),
  sizeof(struct lq_gps_pud),
  4,
  4
};


/**
 * maths functions
 */

#define DEG2RAD 0.017453292 // = Pi/180
#define EARTH_RADIUS 6378000 // in meters

/**
 * Distance along the earth, seen as a sphere,
 * using the haversine formula
 * (http://en.wikipedia.org/wiki/Law_of_haversines)
 * d = 2 R arcsin(sqrt(sin²(dlat/2) + cos(lat1)cos(lat2)sin²(dlon/2)))
 * R is the earth radius ; we thus do not take into account the elevation.
 * (we could try to add it to the earth radius, but it would be
 * really negligible)
 */
/*static double
distance_earth(const struct lq_gps_record *a, const struct lq_gps_record *b) {
  double dlat = (b->lat - a->lat)*DEG2RAD,
    dlon = (b->lon - a->lon)*DEG2RAD;

  double x = pow(sin(dlat/2.0), 2) + cos(a->lat*DEG2RAD)*cos(b->lat*DEG2RAD)*pow(sin(dlon/2.0), 2);
  // asin(sqrt(x)) = atan(sqrt(x)/sqrt(1-x))
  //               = atan2(sqrt(x), sqrt(1-x))
  return EARTH_RADIUS*2*atan2(sqrt(x), sqrt(1 - x));
}*/

/**
 * Line of sight distance from a to b.
 * This distance takes into account the elevation, but
 * assume that a and b are of sight distance from each other.
 * It is the only one that makes sense as far as wireless
 * waves are considered...
 */
static double
distance_lineofsight(const struct lq_gps_record *a, const struct lq_gps_record *b) {
  double lata = a->lat*DEG2RAD, latb = b->lat*DEG2RAD;
  double lona = a->lon*DEG2RAD, lonb = b->lon*DEG2RAD;
  double clata = cos(lata), clatb = cos(latb);
  double ra = EARTH_RADIUS + a->elv;
  double rb = EARTH_RADIUS + b->elv;

  double x = rb*clatb*sin(lonb) - ra*clata*sin(lona);
  double y = rb*clatb*cos(lonb) - ra*clata*cos(lona);
  double z = rb*sin(latb) - ra*sin(lata);
  return sqrt(x*x+y*y+z*z);
}


/**
 * Position info of every other node, kept
 * in a tree, indexed by IP addresses.
 */
static struct avl_tree node_position_tree;

/**
 * Our own gps record, for easier access.
 */
static struct lq_gps_record local_record =
  { 0, 0.0, 0.0, 0, 0, 0, 0.0 };

/**
 * Logs for distance
 */
static FILE* file_dlog = NULL;

/**
 * Computes the distance between the local position
 * and the given lq_node_position, if the time of
 * the given lq_gps_record is bigger than the last
 * distance update time by at least 1 second.
 * Copies the lq_gps_record in the lq_node_position before
 * computing the distance if update_record is set to true.
 */
static void
lq_update_distances(struct lq_node_position *pos)
{
  struct ipaddr_str buf;

  uint32_t newest_time = pos->gps_record.time > local_record.time ?
    pos->gps_record.time : local_record.time;

  if(pos->last_update_time + 1 > newest_time)
    return;

  pos->delta_t = (float) (newest_time - pos->last_update_time);
  pos->last_update_time = newest_time;  
  pos->delta_d = pos->distance;    
  pos->distance = (float) distance_lineofsight(&local_record, &pos->gps_record);
  pos->delta_d = pos->distance - pos->delta_d;

  //if(file_dlog != NULL)
  //  fprintf(file_dlog, "%s, %d, %f, %f, %d, %f, %f, %d\n", olsr_ip_to_string(&buf, &pos->addr), newest_time, (double) local_record.lat, (double) local_record.lon, local_record.elv, (double) pos->gps_record.lat, (double) pos->gps_record.lon, pos->gps_record.elv);

  //olsr_buffered_printf(39 + sizeof(buf) + 2*10, "Updating distance with %s (%.1f) (%.2f)", olsr_ip_to_string(&buf, &pos->addr), (double) pos->delta_t, (double) pos->delta_d);
}

/**
 * Adds a lq_gps_record to the tree with the given addr
 * as key, or updates the current record if the addr
 * is already present.
 */
void 
lq_update_gps_record_pud(const union olsr_ip_addr *addr, const struct lq_gps_record *new_record) {
  const union olsr_ip_addr *main_addr;
  struct lq_node_position *position;
  struct ipaddr_str buf;

  /* find the main address */
  main_addr = mid_lookup_main_addr(addr);
  if(main_addr == NULL)
    main_addr = addr;

  /* find the lq_node_position corresponding to the address,
   * create it if it does not exist */
  position = (struct lq_node_position *) avl_find(&node_position_tree, main_addr);
  if(position == NULL) {
    /* do not add oneself (could happen if PUD is badly configured) */
    if(ipequal(&olsr_cnf->main_addr, main_addr))
      return;
    position = olsr_malloc(sizeof(struct lq_node_position), "New GPS info node");
    if(position == NULL)
      return;
    memset(position, 0, sizeof(struct lq_node_position));
    memcpy(&position->addr, main_addr, olsr_cnf->ip_version == AF_INET ? sizeof(position->addr.v4) : sizeof(position->addr.v6));
    position->node.key = &position->addr;
    avl_insert(&node_position_tree, &position->node, false);
  }
  else
  {
    /* if it already exists, ensure that the new is strictly newer */
    if(new_record->time < position->gps_record.time + 1)
      return;
  }

  memcpy(&position->gps_record, new_record, sizeof(struct lq_gps_record));
  lq_update_distances(position);
}

/**
 * Updates our own gps record
 */
void
lq_update_local_record_pud(const struct lq_gps_record *record) {
  struct lq_node_position *pos;

  if(record->time < local_record.time + 1)
    return;

  memcpy(&local_record, record, sizeof(struct lq_gps_record));
  OLSR_FOR_ALL_GPS_RECORD(pos) {
    lq_update_distances(pos);
  } OLSR_FOR_ALL_GPS_RECORD_END(pos)
}


/**
 * Prints all the currently known gps records, for
 * every node
 */
static int
lq_print_node_position_tree(int a __attribute__((unused)), int b __attribute__((unused)), int c __attribute__((unused)))
{
  struct lq_node_position *pos;
  struct ipaddr_str buf;

  OLSR_PRINTF(1, "\n--- %s -------------------------------------- GPS INFORMATION\n", olsr_wallclock_string());
  OLSR_PRINTF(1, "     IP address      Lat     Long    Alt distance  delta_d  delta_t  lq_mult\n");
  OLSR_PRINTF(1, "           self  %7.4f  %7.3f  %4d\n",
      local_record.lat, local_record.lon, local_record.elv);
  OLSR_FOR_ALL_GPS_RECORD(pos) {
    OLSR_PRINTF(1, "%15s  %7.4f  %7.3f  %4d  %7.2f  %7.2f  %7.1f  %7.3f\n",
      olsr_ip_to_string(&buf, &pos->addr), pos->gps_record.lat,
      pos->gps_record.lon, pos->gps_record.elv,
      (double) pos->distance,
      (double) pos->delta_d, (double) pos->delta_t,
      (double) pos->lq_multiplier);
  } OLSR_FOR_ALL_GPS_RECORD_END(pos)
  return 0;
}


static void
lq_initialize_gps_pud(void)
{
  /* open distances logs */
  struct ipaddr_str buf;
  char name[20 + INET6_ADDRSTRLEN];
  sprintf(name, "/tmp/dlog_node%s.csv", olsr_ip_to_string(&buf, &olsr_cnf->main_addr));
  file_dlog = fopen(name, "w");

  avl_init(&node_position_tree, olsr_cnf->ip_version == AF_INET ? &avl_comp_ipv4 : &avl_comp_ipv6);
  if(olsr_cnf->debug_level > 0)
    register_pcf(&lq_print_node_position_tree);
}

/* The LQ multiplier formula, should not
 * be in a separated function, but this way
 * is a bit easier to perform measurements
 */
static inline float
lq_calc_dist_multiplier(struct lq_node_position *pos)
{
  if(pos == NULL)
    return 1.0f;
  if(pos->last_update_time != pos->last_computed_time) {
    pos->lq_multiplier =
       olsr_cnf->lq_alpha*expf(pos->delta_d/pos->delta_t*olsr_cnf->lq_beta);
    pos->last_computed_time = pos->last_update_time;
  }
  return pos->lq_multiplier;
}

static olsr_linkcost
lq_calc_cost_gps_pud(void *ptr)
{
  const struct lq_gps_pud *lq = ptr;
  olsr_linkcost cost;

  if (lq->lq < (float)MINIMAL_USEFUL_LQ || lq->nlq < (float)MINIMAL_USEFUL_LQ) {
    return LINK_COST_BROKEN;
  }

  cost = (olsr_linkcost) (1.0f / (lq->lq * lq->nlq)
    * lq_calc_dist_multiplier((struct lq_node_position *) avl_find(&node_position_tree, &lq->addr))
    * (float)LQ_PLUGIN_LC_MULTIPLIER);

  if (cost > LINK_COST_BROKEN)
    return LINK_COST_BROKEN;
  if (cost == 0) {
    return 1;
  }
  return cost;
}

static int
lq_serialize_hello_lq_pair_gps_pud(unsigned char *buff, void *ptr)
{
  struct lq_gps_pud *lq = ptr;
  int s = 0, t;

  buff[0] = (unsigned char)(lq->lq * 255);
  buff[1] = (unsigned char)(lq->nlq * 255);

  if(olsr_cnf->ip_version == AF_INET) {
    s = sizeof(struct in_addr);
    memcpy(&buff[2], &olsr_cnf->main_addr.v4.s_addr, s);
  } else {
    s = sizeof(struct in6_addr);
    memcpy(&buff[2], &olsr_cnf->main_addr.v6.s6_addr, s);
  }
  t = (2 + s) % 4;
  if(t == 0)
    return 2 + s;
  memset(&buff[2 + s], 0, 4 - t);
  return 6 + s - t;
}

static void
lq_deserialize_hello_lq_pair_gps_pud(const uint8_t ** curr, void *ptr)
{
  struct lq_gps_pud *lq = ptr;
  uint8_t lq_value, nlq_value;
  int s, t;
  //struct ipaddr_str sbuf;

  pkt_get_u8(curr, &lq_value);
  pkt_get_u8(curr, &nlq_value);

  if(olsr_cnf->ip_version == AF_INET) {
    s = sizeof(struct in_addr);
    memcpy(&lq->addr.v4, *curr, s);
  } else {
    s = sizeof(struct in6_addr);
    memcpy(&lq->addr.v6, *curr, s);
  }
  *curr += s;
  t = (2 + s) % 4;
  if(t != 0)
    *curr += 4 - t;

  lq->lq = (float)lq_value / 255.0f;
  lq->nlq = (float)nlq_value / 255.0f;

  //olsr_buffered_printf(53 + INET6_ADDRSTRLEN, "[HLLQ] lq = %2.2f ; nlq = %2.2f ; addr = %s", (double) lq->lq, (double) lq->nlq, olsr_ip_to_string(&sbuf, &lq->addr));
}

static int
lq_serialize_tc_lq_pair_gps_pud(unsigned char *buff, void *ptr)
{
  struct lq_gps_pud *lq = ptr;
  int s = 0, t;

  buff[0] = (unsigned char)(lq->lq * 255);
  buff[1] = (unsigned char)(lq->nlq * 255);

  if(olsr_cnf->ip_version == AF_INET) {
    s = sizeof(struct in_addr);
    memcpy(&buff[2], &olsr_cnf->main_addr.v4.s_addr, s);
  } else {
    s = sizeof(struct in6_addr);
    memcpy(&buff[2], &olsr_cnf->main_addr.v6.s6_addr, s);
  }
  t = (2 + s) % 4;
  if(t == 0)
    return 2 + s;
  memset(&buff[2 + s], 0, 4 - t);
  return 6 + s - t;
}

static void
lq_deserialize_tc_lq_pair_gps_pud(const uint8_t ** curr, void *ptr)
{
  struct lq_gps_pud *lq = ptr;
  uint8_t lq_value, nlq_value;
  int s, t;

  pkt_get_u8(curr, &lq_value);
  pkt_get_u8(curr, &nlq_value);

  if(olsr_cnf->ip_version == AF_INET) {
    s = sizeof(struct in_addr);
    memcpy(&lq->addr.v4, *curr, s);
  } else {
    s = sizeof(struct in6_addr);
    memcpy(&lq->addr.v6, *curr, s);
  }
  *curr += s;
  t = (2 + s) % 4;
  if(t != 0)
    *curr += 4 - t;

  lq->lq = (float)lq_value / 255.0f;
  lq->nlq = (float)nlq_value / 255.0f;
}

static void
lq_packet_loss_worker_gps_pud(struct link_entry *link, void *ptr, bool lost)
{
  struct lq_gps_pud *tlq = ptr;
  float alpha = olsr_cnf->lq_aging;

  if (tlq->quickstart < LQ_QUICKSTART_STEPS) {
    alpha = LQ_QUICKSTART_AGING;        /* fast enough to get the LQ value within 6 Hellos up to 0.9 */
    tlq->quickstart++;
  }
  // exponential moving average
  tlq->lq *= (1 - alpha);
  if (lost == 0) {
    tlq->lq += (alpha * link->loss_link_multiplier / 65536);
  }
  link->linkcost = lq_calc_cost_gps_pud(ptr);
  olsr_relevant_linkcost_change();
}

static void
lq_memorize_foreign_hello_gps_pud(void *ptrLocal, void *ptrForeign)
{
  struct lq_gps_pud *local = ptrLocal;
  struct lq_gps_pud *foreign = ptrForeign;
  struct ipaddr_str sbuf;

  if (foreign) {
    local->nlq = foreign->lq;
    memcpy(&local->addr, &foreign->addr, sizeof(union olsr_ip_addr));
  } else {
    local->nlq = 0;
  }
}

static void
lq_copy_link2tc_gps_pud(void *target, void *source)
{
  memcpy(target, source, sizeof(struct lq_gps_pud));
}

static void
lq_clear_gps_pud(void *target)
{
  memset(target, 0, sizeof(struct lq_gps_pud));
}

static const char *
lq_print_gps_pud(void *ptr, char separator, struct lqtextbuffer *buffer)
{
  struct lq_gps_pud *lq = ptr;
  //float m = lq_calc_dist_multiplier(lq);
  //snprintf(buffer->buf, sizeof(struct lqtextbuffer), "%2.2f%c%2.2f%c%2.2f", (double)lq->lq, separator, (double)lq->nlq, separator, (double) m);
  snprintf(buffer->buf, sizeof(struct lqtextbuffer), "%2.2f%c%2.2f", (double)lq->lq, separator, (double)lq->nlq);
  return buffer->buf;
}

static const char *
lq_print_cost_gps_pud(olsr_linkcost cost, struct lqtextbuffer *buffer)
{
  snprintf(buffer->buf, sizeof(struct lqtextbuffer), " %2.3f", (double)(((float)cost) / (float)LQ_PLUGIN_LC_MULTIPLIER));

  return buffer->buf;
}
