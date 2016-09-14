
#ifndef LQ_PLUGIN_GPS_LITE_H_
#define LQ_PLUGIN_GPS_LITE_H_

#include "olsr_types.h"
#include "common/avl.h"
#include "lq_plugin.h"

#define LQ_ALGORITHM_ETX_GPS_LITE_NAME "etx_gps_lite"

#define LQ_PLUGIN_LC_MULTIPLIER 1024

struct gps_position {
  float lat;
  float lon;
  int32_t alt;
};

struct lq_gps_lite {
  float lq, nlq, mlq;
};

struct lq_link_gps_lite {
  struct lq_gps_lite lq;
  uint16_t quickstart;
  struct gps_position nposition;
  uint32_t last_gps_update;  
  uint32_t last_multiplier_calc;
  float distance;
  float speed; // to be removed
};


extern struct lq_handler lq_etx_gps_lite_handler;
void lq_gps_lite_update_position(float, float, int32_t);

#endif /* LQ_PLUGIN_GPS_LITE_H_ */
