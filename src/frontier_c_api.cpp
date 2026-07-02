#include "omninav_frontier/frontier_c_api.h"

#include "omninav_frontier/standalone_frontier.hpp"

#include <cstdlib>
#include <cstring>
#include <exception>
#include <algorithm>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>

namespace
{

using omninav_frontier::FrontierConfig;
using omninav_frontier::Odom;
using omninav_frontier::StandaloneFrontierMap;

FrontierConfig toCppConfig(const OmniNavFrontierConfig & input)
{
  FrontierConfig config;
  config.width = input.width;
  config.height = input.height;
  config.meters_per_pixel = input.meters_per_pixel;
  config.origin_x = input.origin_x;
  config.origin_y = input.origin_y;
  config.robot_radius_m = input.robot_radius_m;
  config.obstacle_radius_m = input.obstacle_radius_m;
  config.lidar_y_positive_is_left = input.lidar_y_positive_is_left != 0;
  config.min_frontier_distance_m = input.min_frontier_distance_m;
  config.max_frontier_distance_m = input.max_frontier_distance_m;
  config.min_boundary_length_px = input.min_boundary_length_px;
  config.log_odds_hit = input.log_odds_hit;
  config.log_odds_miss = input.log_odds_miss;
  config.log_odds_min = input.log_odds_min;
  config.log_odds_max = input.log_odds_max;
  config.occupied_log_odds_threshold = input.occupied_log_odds_threshold;
  config.free_log_odds_threshold = input.free_log_odds_threshold;
  config.endpoint_inflation_radius_m = input.endpoint_inflation_radius_m;
  config.ray_endpoint_clearance_m = input.ray_endpoint_clearance_m;
  config.median_filter_radius = input.median_filter_radius;
  config.outlier_jump_m = input.outlier_jump_m;
  config.morph_close_radius_m = input.morph_close_radius_m;
  config.morph_open_radius_m = input.morph_open_radius_m;
  config.reachable_erosion_radius_m = input.reachable_erosion_radius_m;
  return config;
}

Odom toCppOdom(const OmniNavFrontierOdom & input)
{
  Odom odom;
  odom.x = input.x;
  odom.y = input.y;
  odom.z = input.z;
  odom.yaw = input.yaw;
  return odom;
}

void writeError(const std::string & message, char * output, std::size_t output_count)
{
  if (output == nullptr || output_count == 0U) {
    return;
  }
  const std::size_t copy_count = std::min(output_count - 1U, message.size());
  std::memcpy(output, message.data(), copy_count);
  output[copy_count] = '\0';
}

char * duplicateString(const std::string & value)
{
  char * output = static_cast<char *>(std::malloc(value.size() + 1U));
  if (output == nullptr) {
    throw std::bad_alloc();
  }
  std::memcpy(output, value.c_str(), value.size() + 1U);
  return output;
}

}  // namespace

extern "C" {

void omninav_frontier_default_config(OmniNavFrontierConfig * config)
{
  if (config == nullptr) {
    return;
  }
  const FrontierConfig defaults;
  config->width = defaults.width;
  config->height = defaults.height;
  config->meters_per_pixel = defaults.meters_per_pixel;
  config->origin_x = defaults.origin_x;
  config->origin_y = defaults.origin_y;
  config->robot_radius_m = defaults.robot_radius_m;
  config->obstacle_radius_m = defaults.obstacle_radius_m;
  config->lidar_y_positive_is_left = defaults.lidar_y_positive_is_left ? 1 : 0;
  config->min_frontier_distance_m = defaults.min_frontier_distance_m;
  config->max_frontier_distance_m = defaults.max_frontier_distance_m;
  config->min_boundary_length_px = defaults.min_boundary_length_px;
  config->log_odds_hit = defaults.log_odds_hit;
  config->log_odds_miss = defaults.log_odds_miss;
  config->log_odds_min = defaults.log_odds_min;
  config->log_odds_max = defaults.log_odds_max;
  config->occupied_log_odds_threshold = defaults.occupied_log_odds_threshold;
  config->free_log_odds_threshold = defaults.free_log_odds_threshold;
  config->endpoint_inflation_radius_m = defaults.endpoint_inflation_radius_m;
  config->ray_endpoint_clearance_m = defaults.ray_endpoint_clearance_m;
  config->median_filter_radius = defaults.median_filter_radius;
  config->outlier_jump_m = defaults.outlier_jump_m;
  config->morph_close_radius_m = defaults.morph_close_radius_m;
  config->morph_open_radius_m = defaults.morph_open_radius_m;
  config->reachable_erosion_radius_m = defaults.reachable_erosion_radius_m;
}

void * omninav_frontier_create(const OmniNavFrontierConfig * config)
{
  try {
    OmniNavFrontierConfig resolved_config;
    if (config == nullptr) {
      omninav_frontier_default_config(&resolved_config);
      config = &resolved_config;
    }
    return new StandaloneFrontierMap(toCppConfig(*config));
  } catch (...) {
    return nullptr;
  }
}

void omninav_frontier_destroy(void * handle)
{
  delete static_cast<StandaloneFrontierMap *>(handle);
}

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
  size_t error_output_count)
{
  if (error_output != nullptr && error_output_count > 0U) {
    error_output[0] = '\0';
  }
  if (result_json != nullptr) {
    *result_json = nullptr;
  }

  try {
    if (handle == nullptr) {
      throw std::runtime_error("frontier map handle is null");
    }
    if (result_json == nullptr) {
      throw std::runtime_error("result_json output pointer is null");
    }

    auto * map = static_cast<StandaloneFrontierMap *>(handle);
    const auto result = map->update(
      ranges,
      range_count,
      angle_min,
      angle_increment,
      range_min,
      range_max,
      toCppOdom(odom),
      reset != 0);
    map->copyCellsTo(grid_output, grid_output_count);
    *result_json = duplicateString(omninav_frontier::resultToJson(result));
    return 0;
  } catch (const std::exception & error) {
    writeError(error.what(), error_output, error_output_count);
    return -1;
  } catch (...) {
    writeError("unknown C++ exception", error_output, error_output_count);
    return -1;
  }
}

int omninav_frontier_copy_debug_layers(
  void * handle,
  uint8_t * raw_hit_output,
  uint8_t * occupied_output,
  uint8_t * reachable_free_output,
  size_t grid_output_count,
  char * error_output,
  size_t error_output_count)
{
  if (error_output != nullptr && error_output_count > 0U) {
    error_output[0] = '\0';
  }

  try {
    if (handle == nullptr) {
      throw std::runtime_error("frontier map handle is null");
    }

    auto * map = static_cast<StandaloneFrontierMap *>(handle);
    map->copyDebugLayersTo(
      raw_hit_output,
      occupied_output,
      reachable_free_output,
      grid_output_count);
    return 0;
  } catch (const std::exception & error) {
    writeError(error.what(), error_output, error_output_count);
    return -1;
  } catch (...) {
    writeError("unknown C++ exception", error_output, error_output_count);
    return -1;
  }
}

void omninav_frontier_free_string(char * value)
{
  std::free(value);
}

}  // extern "C"
