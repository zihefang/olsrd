
#ifndef LQ_PLUGIN_GPS_PUD_H_
#define LQ_PLUGIN_GPS_PUD_H_

#include "olsr_types.h"
#include "common/avl.h"
#include "lq_plugin.h"

#define LQ_ALGORITHM_ETX_GPS_PUD_NAME "etx_gps_pud"

#define LQ_PLUGIN_LC_MULTIPLIER 1024

struct lq_gps_pud {
  float lq, nlq;
  uint16_t quickstart;
  union olsr_ip_addr addr;
};

extern struct lq_handler lq_etx_gps_pud_handler;

struct lq_gps_record {
	uint32_t time;
	double lat;
	double lon;
	int32_t elv;
	uint32_t speed;
	uint32_t track;
	double hdop;
};

struct lq_node_position {
	struct avl_node node;
	union olsr_ip_addr addr;
	struct lq_gps_record gps_record;
	uint32_t last_update_time;	
	float distance;
	float delta_d;
	float delta_t;
	uint32_t last_computed_time;
	float lq_multiplier;
};

AVLNODE2STRUCT(avlnode2node_position, struct lq_node_position, node);
#define OLSR_FOR_ALL_GPS_RECORD(node_position) \
{ \
  struct avl_node *avlnode, *next_avlnode; \
  for (avlnode = avl_walk_first(&node_position_tree); \
    avlnode; avlnode = next_avlnode) { \
    next_avlnode = avl_walk_next(avlnode); \
    node_position = avlnode2node_position(avlnode);
#define OLSR_FOR_ALL_GPS_RECORD_END(node_position) }}

void lq_update_gps_record_pud(const union olsr_ip_addr*, const struct lq_gps_record*);
void lq_update_local_record_pud(const struct lq_gps_record*);

#endif /* LQ_PLUGIN_GPS_PUD_H_ */
