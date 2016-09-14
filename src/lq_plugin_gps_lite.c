
#include "tc_set.h"
#include "link_set.h"
#include "olsr_spf.h"
#include "lq_packet.h"
#include "packet.h"
#include "olsr.h"
#include "debug.h"
#include "lq_plugin_gps_lite.h"
#include "scheduler.h"
#include "mid_set.h"
#include "common/avl.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

static void lq_initialize_gps_lite(void);
static olsr_linkcost lq_calc_cost_gps_lite(void *lq);
static void lq_packet_loss_worker_gps_lite(struct link_entry *link, void *lq, bool lost);
static void lq_memorize_foreign_hello_gps_lite(void *local, void *foreign);
static int lq_serialize_lq_pair_gps_lite(unsigned char *buff, void *lq);
static void lq_deserialize_lq_pair_gps_lite(const uint8_t ** curr, void *lq);
static int lq_serialize_global_gps_lite(unsigned char *buff);
static void lq_deserialize_global_gps_lite(const uint8_t ** curr);
static void lq_copy_link2neigh_gps_lite(void *target, void *source);
static void lq_copy_link2tc_gps_lite(void *target, void *source);
static void lq_clear_gps_lite(void *target);
static void lq_clear_link_gps_lite(void *target);
static const char *lq_print_link_gps_lite(void *ptr, char separator, struct lqtextbuffer *buffer);
static const char *lq_print_tc_gps_lite(void *ptr, char separator, struct lqtextbuffer *buffer);
static const char *lq_print_cost_gps_lite(olsr_linkcost cost, struct lqtextbuffer *buffer);
static double get_cost_scaled(olsr_linkcost cost) { return cost; };  // not used:

#define lqdata_size ((size_t) (2 + sizeof(float) + ((2 + sizeof(float)) % 4 == 0 ? 0 : 4 - ((2 + sizeof(float)) % 4))))
#define global_lqdata_size ((size_t) (sizeof(struct gps_position) + (sizeof(struct gps_position) % 4 == 0 ? 0 : 4 - (sizeof(struct gps_position) % 4))))

/* Default lq plugin settings */
struct lq_handler lq_etx_gps_lite_handler = {
  &lq_initialize_gps_lite,

  &lq_calc_cost_gps_lite,
  &lq_calc_cost_gps_lite,

  &lq_packet_loss_worker_gps_lite,

  &lq_memorize_foreign_hello_gps_lite,
  &lq_copy_link2neigh_gps_lite,
  &lq_copy_link2tc_gps_lite,
  &lq_clear_gps_lite,
  &lq_clear_gps_lite,
  &lq_clear_link_gps_lite,

  &lq_serialize_lq_pair_gps_lite,
  &lq_serialize_lq_pair_gps_lite,
  &lq_deserialize_lq_pair_gps_lite,
  &lq_deserialize_lq_pair_gps_lite,

  &lq_serialize_global_gps_lite,
  &olsr_default_serialize_global_lq,
  &lq_deserialize_global_gps_lite,
  &olsr_default_deserialize_global_lq,

  &lq_print_link_gps_lite,
  &lq_print_tc_gps_lite,
  &lq_print_cost_gps_lite,
  &get_cost_scaled,

  sizeof(struct lq_gps_lite),
  sizeof(struct lq_gps_lite),
  sizeof(struct lq_link_gps_lite),
  lqdata_size,
  lqdata_size
};


/**
 * The position of this node, plus the time in ms
 * of the last position update received from the
 * gps
 */

static struct my_position {
  struct gps_position position;
  uint32_t last_gps_update;
} my_position;


/**
 * position used to copy and keep in memory the data
 * found by the 'global' deserializing functions
 */
static struct gps_position buffered_position;

/**
 * distance computation
 */

#define DEG2RAD 0.017453292 // = Pi/180
#define EARTH_RADIUS 6371000 // in meters

/**
 * Line of sight distance from npos to my_position.
 * This distance takes into account the elevation, but
 * assume that a and b are of sight distance from each other.
 * It is the only one that makes sense as far as wireless
 * waves are considered...
 * 
 */
static double
distance_lineofsight(const struct gps_position *npos) {
  double lata = (double) my_position.position.lat*DEG2RAD, latb = (double) npos->lat*DEG2RAD;
  double lona = (double) my_position.position.lon*DEG2RAD, lonb = (double) npos->lon*DEG2RAD;
  double dlat = lata - latb ;
  double dlon = lona - lonb ;
  double sinDlon = sin(dlon*0.5) ;
  double sinDlat = sin(dlat*0.5) ;
  double clata = cos(lata), clatb = cos(latb);
  double a = pow(sinDlat,2) + ( pow(sinDlon,2) * clata * clatb ) ; 
  double c = 2.0 * atan2(sqrt(a),sqrt(1.0-a)) ; 
  return EARTH_RADIUS * c ;
  /** haversine method : from http://www.movable-type.co.uk/scripts/latlong.html */
}


static inline void
print_debug_gpspos(int size, const char* name, const struct gps_position* ptr)
{
#ifdef _OLSR_DEBUG_H_
  olsr_buffered_printf(19 + size + 2*10, "%s :: GPS(%7.4f, %7.3f, %5d)", name, (double) ptr->lat, (double) ptr->lon, ptr->alt);
#endif
}


/**
 * Logs for distance
 */
//static FILE* file_dlog = NULL;


void
lq_gps_lite_update_position(float lat, float lon, int32_t alt)
{
  if(now_times > my_position.last_gps_update)
  {
    my_position.position.lat = lat;
    my_position.position.lon = lon;
    my_position.position.alt = alt;
    my_position.last_gps_update = now_times;
  }
}

static int
lq_print_known_positions(int a __attribute__((unused)), int b __attribute__((unused)), int c __attribute__((unused)))
{
  struct link_entry *link;
  struct lq_link_gps_lite *lq;
  struct ipaddr_str buf;

  OLSR_PRINTF(1, "\n--- %s -------------------------------------- GPS_LITE INFORMATION\n", olsr_wallclock_string());
  OLSR_PRINTF(1, "     IP Address      Lat     Long    Alt distance   speed     MLQ\n");
  OLSR_PRINTF(1, "           self  %7.4f  %7.3f  %4d  %7.2f\n",
      (double) my_position.position.lat, (double) my_position.position.lon,
      my_position.position.alt, my_position.last_gps_update/1000.0);

  OLSR_FOR_ALL_LINK_ENTRIES(link) {
    lq = (struct lq_link_gps_lite *)link->linkquality;
/* 
   if ((lq) && (lq->speed)) {
     // TODO: initialize speed and then remove this fix
      if (lq->speed > 25.0) lq->speed = 0.0;
      else if (lq->speed < -25.0) lq->speed = 0.0;
    }
    if ((lq) && (!lq->speed)) lq->speed = 0.0; */
    OLSR_PRINTF(1, "%15s  %7.4f  %7.3f  %4d  %7.1f  %6.2f  %6.3f\n",
      olsr_ip_to_string(&buf, &link->neighbor_iface_addr),
      (double) lq->nposition.lat, (double) lq->nposition.lon, lq->nposition.alt,
      (double) lq->distance, (double) lq->speed, (double) lq->lq.mlq);
  } OLSR_FOR_ALL_LINK_ENTRIES_END(link)
  return 0;
}


static void
lq_initialize_gps_lite(void)
{
  /* open distances logs */
  /*struct ipaddr_str buf;
  char name[20 + INET6_ADDRSTRLEN];
  sprintf(name, "/tmp/dlog_node%s.csv", olsr_ip_to_string(&buf, &olsr_cnf->main_addr));
  file_dlog = fopen(name, "w");*/

  my_position.last_gps_update = now_times;
  my_position.position.lat = 46.00002;
  my_position.position.lon = 6.0001;
  my_position.position.alt = 400;

  if(olsr_cnf->debug_level > 0)
    register_pcf(&lq_print_known_positions);
}

static inline void
lq_set_dist_multiplier(struct lq_link_gps_lite *lq)
{
  float delta_d = lq->distance;
  uint32_t last_update = my_position.last_gps_update > lq->last_gps_update ?
    my_position.last_gps_update : lq->last_gps_update;

 // for the moment we do not need this check
 // if(lq->last_multiplier_calc + olsr_cnf->lq_alpha >= last_update)
 //   return;

 
  lq->distance = distance_lineofsight(&lq->nposition);
  delta_d = lq->distance - delta_d;

  // olsr_cnf->lq_dist_ext_mult is unused for now
  // delta_t = last_update - lq->last_multiplier_calc is in milliseconds
  //                             1000
  // multiplier = exp(delta_d * ------- * DIST_INT_MULT)
  //                            delta_t
  /*lq->lq.mlq = expf(
    delta_d *
    1000.0f/((float) (last_update - lq->last_multiplier_calc)) *
    (float)olsr_cnf->beta);*/


 /*  if ((lq) && (lq->speed)) {
     // TODO: initialize speed and then remove this fix
      if (lq->speed > 15.0) lq->speed = 0.0;
      else if (lq->speed < -15.0) lq->speed = 0.0;
    }
    if ((lq) && (!lq->speed)) lq->speed = 0.0; */

// exponential smoothing  of the spped
   float instantaneous_speed ;
   float gamma =  olsr_cnf->lq_gamma; 
   instantaneous_speed =  delta_d * 1000.0f /
   ((float) (last_update - lq->last_multiplier_calc));	 

// test on instantaneous_speed
   if (instantaneous_speed > 25.0) instantaneous_speed = 0.0;
   else if (instantaneous_speed < -25.0) instantaneous_speed = 0.0;

   lq->speed = (lq->speed * (1 - gamma))  + ( gamma * instantaneous_speed );

  
  lq->lq.mlq = expf(lq->speed * olsr_cnf->lq_beta);
 
 lq->last_multiplier_calc = last_update;
  return;
}

static olsr_linkcost
lq_calc_cost_gps_lite(void *ptr)
{
  const struct lq_gps_lite *lq = ptr;
  olsr_linkcost cost;

  if (lq->lq < (float)MINIMAL_USEFUL_LQ || lq->nlq < (float)MINIMAL_USEFUL_LQ) {
    return LINK_COST_BROKEN;
  }

  cost = (olsr_linkcost) ((1.0f / (lq->lq * lq->nlq)) *
    lq->mlq * (float) LQ_PLUGIN_LC_MULTIPLIER);

  if (cost > LINK_COST_BROKEN)
    return LINK_COST_BROKEN;
  if (cost == 0) {
    return 1;
  }
  return cost;
}

static int
lq_serialize_lq_pair_gps_lite(unsigned char *buff, void *ptr)
{
  struct lq_gps_lite *lq = ptr;

  buff[0] = (unsigned char)(lq->lq * 255);
  buff[1] = (unsigned char)(lq->nlq * 255);
  memcpy(&buff[2], &lq->mlq, sizeof(float));

  return lqdata_size;
}

static void
lq_deserialize_lq_pair_gps_lite(const uint8_t ** curr, void *ptr)
{
  struct lq_gps_lite *lq = ptr;
  uint8_t lq_value, nlq_value;

  pkt_get_u8(curr, &lq_value);
  pkt_get_u8(curr, &nlq_value);

  memcpy(&lq->mlq, *curr, sizeof(float));
  *curr += lqdata_size - 2;

  lq->lq = (float)lq_value / 255.0f;
  lq->nlq = (float)nlq_value / 255.0f;
}

static int
lq_serialize_global_gps_lite(unsigned char *buff)
{
  memcpy(buff, &my_position.position, sizeof(struct gps_position));
  return global_lqdata_size;
}

static void
lq_deserialize_global_gps_lite(const uint8_t ** curr)
{
  memcpy(&buffered_position, *curr, sizeof(struct gps_position));
  *curr += global_lqdata_size;
}

static void
lq_packet_loss_worker_gps_lite(struct link_entry *link, void *ptr, bool lost)
{
  struct lq_link_gps_lite *tlq = ptr;
  float alpha = olsr_cnf->lq_aging;

  if (tlq->quickstart < LQ_QUICKSTART_STEPS) {
    alpha = LQ_QUICKSTART_AGING;        // fast enough to get the LQ value within 6 Hellos up to 0.9
    tlq->quickstart++;
  }

// Old version of modified aging
/*
  if(tlq->speed > 0 && lost != 0) {
    alpha = alpha * olsr_cnf->lq_delta;
    if(alpha > 1)
      alpha = 1;
  } else if(tlq->speed < 0 && lost == 0) {
    alpha = alpha * olsr_cnf->lq_gamma;
    if(alpha > 1)
      alpha = 1;
  }
*/
  // exponential moving average
  tlq->lq.lq *= (1 - alpha);
  if (lost == 0) {
    tlq->lq.lq += (alpha * link->loss_link_multiplier / 65536);
  }

  link->linkcost = lq_calc_cost_gps_lite(ptr);
  olsr_relevant_linkcost_change();
}

static void
lq_memorize_foreign_hello_gps_lite(void *ptrLocal, void *ptrForeign)
{
  struct lq_link_gps_lite *local = ptrLocal;
  struct lq_gps_lite *foreign = ptrForeign;

  
  if (foreign) {
    local->lq.nlq = foreign->lq;
    if(now_times > local->last_gps_update)
    {
      memcpy(&local->nposition, &buffered_position, sizeof(struct gps_position));
      local->last_gps_update = now_times;
      lq_set_dist_multiplier(local);
    }
  } else {
    local->lq.nlq = 0;
  }
}

static void
lq_copy_link2neigh_gps_lite(void *target, void *source)
{
  memcpy(target, source, sizeof(struct lq_gps_lite));
}

static void
lq_copy_link2tc_gps_lite(void *target, void *source)
{
  memcpy(target, source, sizeof(struct lq_gps_lite));
}

static void
lq_clear_gps_lite(void *target)
{
  memset(target, 0, sizeof(struct lq_gps_lite));
  ((struct lq_gps_lite*)target)->mlq = 1.0f;
}

static void
lq_clear_link_gps_lite(void *target)
{
  memset(target, 0, sizeof(struct lq_link_gps_lite));
  ((struct lq_link_gps_lite*)target)->lq.lq = 0.0f;
  ((struct lq_link_gps_lite*)target)->lq.nlq = 0.0f;
  ((struct lq_link_gps_lite*)target)->lq.mlq = 1.0f;
}

static const char *
lq_print_link_gps_lite(void *ptr, char separator, struct lqtextbuffer *buffer)
{
  struct lq_gps_lite *lq = ptr;
  snprintf(buffer->buf, sizeof(struct lqtextbuffer), "%2.2f%c%2.2f%c%5.2f", (double)lq->lq, separator, (double)lq->nlq, separator, (double)lq->mlq);
  return buffer->buf;
}

static const char *
lq_print_tc_gps_lite(void *ptr, char separator, struct lqtextbuffer *buffer)
{
  struct lq_gps_lite *lq = ptr;
  snprintf(buffer->buf, sizeof(struct lqtextbuffer), "%2.2f%c%2.2f%c%5.2f", (double)lq->lq, separator, (double)lq->nlq, separator, (double)lq->mlq);
  return buffer->buf;
}

static const char *
lq_print_cost_gps_lite(olsr_linkcost cost, struct lqtextbuffer *buffer)
{
  snprintf(buffer->buf, sizeof(struct lqtextbuffer), " %2.3f", (double)(((float)cost) / (float)LQ_PLUGIN_LC_MULTIPLIER));

  return buffer->buf;
}
