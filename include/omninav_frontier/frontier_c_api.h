#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct OmniNavFrontierConfig
{
  int width;
  int height;
  double meters_per_pixel;
  double origin_x;
  double origin_y;
  double robot_radius_m;
  double obstacle_radius_m;
  int lidar_y_positive_is_left;
  double min_frontier_distance_m;
  double max_frontier_distance_m;
  size_t min_boundary_length_px;
} OmniNavFrontierConfig;

typedef struct OmniNavFrontierOdom
{
  double x;
  double y;
  double z;
  double yaw;
} OmniNavFrontierOdom;

void omninav_frontier_default_config(OmniNavFrontierConfig * config);

void * omninav_frontier_create(const OmniNavFrontierConfig * config);

void omninav_frontier_destroy(void * handle);

int omninav_frontier_update(
  void * handle,
  const double * ranges,
  size_t range_count,
  double angle_min,
  double angle_increment,
  double range_min,
  double range_max,
  OmniNavFrontierOdom odom,
  int reset,
  uint8_t * grid_output,
  size_t grid_output_count,
  char ** result_json,
  char * error_output,
  size_t error_output_count);

void omninav_frontier_free_string(char * value);

#ifdef __cplusplus
}
#endif
