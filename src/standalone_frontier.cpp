#include "omninav_frontier/standalone_frontier.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <queue>
#include <sstream>
#include <stdexcept>

namespace omninav_frontier
{

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kMinFrontierSeparationM = 1.0;
constexpr std::array<StandaloneFrontierMap::Direction, 4> kCardinalDirections{{
  {-1, 0},
  {1, 0},
  {0, -1},
  {0, 1},
}};
constexpr std::array<StandaloneFrontierMap::Direction, 8> kComponentDirections{{
  {-1, -1},
  {-1, 0},
  {-1, 1},
  {0, -1},
  {0, 1},
  {1, -1},
  {1, 0},
  {1, 1},
}};

std::size_t checkedCellCount(const FrontierConfig & config)
{
  if (config.width <= 0 || config.height <= 0) {
    throw std::runtime_error("width and height must be positive");
  }
  return static_cast<std::size_t>(config.width) * static_cast<std::size_t>(config.height);
}

double normalizeAngle(double angle_rad)
{
  while (angle_rad > kPi) {
    angle_rad -= 2.0 * kPi;
  }
  while (angle_rad <= -kPi) {
    angle_rad += 2.0 * kPi;
  }
  return angle_rad;
}

double odomYawToMapYaw(double odom_yaw_rad)
{
  return normalizeAngle((kPi * 0.5) - odom_yaw_rad);
}

bool isFiniteHit(double range_m, double range_min, double range_max)
{
  return std::isfinite(range_m) && range_m >= range_min && range_m < range_max;
}

bool isNoHit(double range_m, double range_max)
{
  if (std::isinf(range_m) && range_m > 0.0) {
    return true;
  }
  return std::isfinite(range_m) && range_m >= range_max;
}

double medianOf(std::vector<double> values)
{
  const auto middle = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2U);
  std::nth_element(values.begin(), middle, values.end());
  return *middle;
}

std::size_t countNonZero(const std::vector<std::uint8_t> & values)
{
  return static_cast<std::size_t>(std::count(values.begin(), values.end(), 1U));
}

template<typename Func>
void forDiskPixels(int row, int col, int radius_px, int width, int height, Func func)
{
  const int radius_sq = radius_px * radius_px;
  for (int dr = -radius_px; dr <= radius_px; ++dr) {
    for (int dc = -radius_px; dc <= radius_px; ++dc) {
      const int next_row = row + dr;
      const int next_col = col + dc;
      if ((dr * dr + dc * dc) <= radius_sq &&
        next_row >= 0 && next_row < height && next_col >= 0 && next_col < width)
      {
        func(next_row, next_col, static_cast<std::size_t>(next_row * width + next_col));
      }
    }
  }
}

bool diskAllSet(
  const std::vector<std::uint8_t> & mask,
  int row,
  int col,
  int radius_px,
  int width,
  int height)
{
  const int radius_sq = radius_px * radius_px;
  for (int dr = -radius_px; dr <= radius_px; ++dr) {
    for (int dc = -radius_px; dc <= radius_px; ++dc) {
      if ((dr * dr + dc * dc) > radius_sq) {
        continue;
      }
      const int next_row = row + dr;
      const int next_col = col + dc;
      if (next_row < 0 || next_row >= height || next_col < 0 || next_col >= width ||
        mask[static_cast<std::size_t>(next_row * width + next_col)] == 0U)
      {
        return false;
      }
    }
  }
  return true;
}

}  // namespace

StandaloneFrontierMap::StandaloneFrontierMap(FrontierConfig config)
: config_(config),
  cells_(checkedCellCount(config), kGridCellUnknown),
  log_odds_(cells_.size(), 0.0),
  raw_hit_mask_(cells_.size(), 0U),
  occupied_mask_(cells_.size(), 0U),
  reachable_free_mask_(cells_.size(), 0U)
{
  if (config_.meters_per_pixel <= 0.0) {
    throw std::runtime_error("meters_per_pixel must be positive");
  }
  if (config_.robot_radius_m < 0.0 || config_.obstacle_radius_m < 0.0 ||
    config_.endpoint_inflation_radius_m < 0.0 || config_.ray_endpoint_clearance_m < 0.0 ||
    config_.morph_close_radius_m < 0.0 || config_.morph_open_radius_m < 0.0 ||
    config_.reachable_erosion_radius_m < 0.0)
  {
    throw std::runtime_error("frontier radii must be non-negative");
  }
  if (config_.min_frontier_distance_m < 0.0 || config_.max_frontier_distance_m < 0.0 ||
    config_.min_frontier_distance_m > config_.max_frontier_distance_m)
  {
    throw std::runtime_error("invalid frontier distance range");
  }
  if (config_.log_odds_hit <= 0.0 || config_.log_odds_miss >= 0.0) {
    throw std::runtime_error("log_odds_hit must be positive and log_odds_miss must be negative");
  }
  if (config_.log_odds_min >= config_.log_odds_max) {
    throw std::runtime_error("log_odds_min must be smaller than log_odds_max");
  }
  if (config_.free_log_odds_threshold >= config_.occupied_log_odds_threshold) {
    throw std::runtime_error("free threshold must be smaller than occupied threshold");
  }
  if (config_.median_filter_radius < 0 || config_.outlier_jump_m < 0.0) {
    throw std::runtime_error("range filter parameters must be non-negative");
  }
}

void StandaloneFrontierMap::reset()
{
  std::fill(cells_.begin(), cells_.end(), kGridCellUnknown);
  std::fill(log_odds_.begin(), log_odds_.end(), 0.0);
  std::fill(raw_hit_mask_.begin(), raw_hit_mask_.end(), 0U);
  std::fill(occupied_mask_.begin(), occupied_mask_.end(), 0U);
  std::fill(reachable_free_mask_.begin(), reachable_free_mask_.end(), 0U);
}

FrontierResult StandaloneFrontierMap::update(
  const double * ranges,
  std::size_t range_count,
  double angle_min,
  double angle_increment,
  double range_min,
  double range_max,
  const Odom & odom,
  bool reset_first)
{
  if (reset_first) {
    reset();
  }
  if (ranges == nullptr && range_count > 0U) {
    throw std::runtime_error("ranges pointer is null");
  }
  if (range_max < range_min) {
    throw std::runtime_error("range_max must be >= range_min");
  }

  std::fill(raw_hit_mask_.begin(), raw_hit_mask_.end(), 0U);
  PixelRC agent;
  if (!worldToPixel(odom.x, odom.y, &agent)) {
    return detectFrontiers(odom);
  }

  std::vector<std::uint8_t> miss_update_mask(cells_.size(), 0U);
  std::vector<std::uint8_t> hit_update_mask(cells_.size(), 0U);
  const std::vector<RangeSample> samples = filterRanges(ranges, range_count, range_min, range_max);

  double angle = angle_min;
  for (const RangeSample & sample : samples) {
    if (sample.valid) {
      const double clear_range = sample.hit ?
        std::max(0.0, sample.range - config_.ray_endpoint_clearance_m) :
        sample.range;
      markRayFreeCells(miss_update_mask, odom, angle, clear_range);

      if (sample.hit) {
        const auto [world_x, world_y] = lidarPointToWorld(sample.range, angle, odom);
        PixelRC endpoint;
        if (worldToPixel(world_x, world_y, &endpoint)) {
          const auto endpoint_index = static_cast<std::size_t>(index(endpoint.row, endpoint.col));
          raw_hit_mask_[endpoint_index] = 1U;
          setMaskDisk(
            hit_update_mask,
            endpoint.row,
            endpoint.col,
            config_.endpoint_inflation_radius_m,
            1U);
        }
      }
    }
    angle += angle_increment;
  }

  for (std::size_t i = 0; i < cells_.size(); ++i) {
    if (hit_update_mask[i] != 0U) {
      addLogOdds(i, config_.log_odds_hit);
    } else if (miss_update_mask[i] != 0U) {
      addLogOdds(i, config_.log_odds_miss);
    }
  }

  addLogOddsDisk(agent.row, agent.col, config_.robot_radius_m, config_.log_odds_miss);
  rebuildGridFromLogOdds(agent);
  return detectFrontiers(odom);
}

void StandaloneFrontierMap::copyCellsTo(std::uint8_t * output, std::size_t output_count) const
{
  copyMaskTo(cells_, output, output_count);
}

void StandaloneFrontierMap::copyDebugLayersTo(
  std::uint8_t * raw_hit_output,
  std::uint8_t * occupied_output,
  std::uint8_t * reachable_free_output,
  std::size_t output_count) const
{
  copyMaskTo(raw_hit_mask_, raw_hit_output, output_count);
  copyMaskTo(occupied_mask_, occupied_output, output_count);
  copyMaskTo(reachable_free_mask_, reachable_free_output, output_count);
}

bool StandaloneFrontierMap::worldToPixel(double world_x, double world_y, PixelRC * pixel) const
{
  const double center_col = static_cast<double>(config_.width) * 0.5;
  const double center_row = static_cast<double>(config_.height) * 0.5;
  const int col = static_cast<int>(
    std::floor(((world_x - config_.origin_x) / config_.meters_per_pixel) + center_col));
  const int row = static_cast<int>(
    std::floor(((config_.origin_y - world_y) / config_.meters_per_pixel) + center_row));

  if (!inBounds(row, col)) {
    return false;
  }
  pixel->row = row;
  pixel->col = col;
  return true;
}

bool StandaloneFrontierMap::inBounds(int row, int col) const
{
  return row >= 0 && row < config_.height && col >= 0 && col < config_.width;
}

int StandaloneFrontierMap::index(int row, int col) const
{
  return row * config_.width + col;
}

int StandaloneFrontierMap::radiusMetersToPixels(double radius_m) const
{
  return std::max(0, static_cast<int>(std::ceil(radius_m / config_.meters_per_pixel)));
}

void StandaloneFrontierMap::copyMaskTo(
  const std::vector<std::uint8_t> & mask,
  std::uint8_t * output,
  std::size_t output_count) const
{
  if (output == nullptr) {
    throw std::runtime_error("grid output pointer is null");
  }
  if (output_count != mask.size()) {
    throw std::runtime_error("grid output size does not match configured map size");
  }
  std::memcpy(output, mask.data(), mask.size() * sizeof(std::uint8_t));
}

void StandaloneFrontierMap::setMaskDisk(
  std::vector<std::uint8_t> & mask,
  int row,
  int col,
  double radius_m,
  std::uint8_t value) const
{
  forDiskPixels(row, col, radiusMetersToPixels(radius_m), config_.width, config_.height,
    [&](int, int, std::size_t idx) { mask[idx] = value; });
}

void StandaloneFrontierMap::addLogOdds(std::size_t cell_index, double delta)
{
  log_odds_[cell_index] =
    std::clamp(log_odds_[cell_index] + delta, config_.log_odds_min, config_.log_odds_max);
}

void StandaloneFrontierMap::addLogOddsDisk(int row, int col, double radius_m, double delta)
{
  forDiskPixels(row, col, radiusMetersToPixels(radius_m), config_.width, config_.height,
    [&](int, int, std::size_t idx) { addLogOdds(idx, delta); });
}

std::vector<StandaloneFrontierMap::RangeSample> StandaloneFrontierMap::filterRanges(
  const double * ranges,
  std::size_t range_count,
  double range_min,
  double range_max) const
{
  std::vector<RangeSample> samples(range_count);
  for (std::size_t i = 0; i < range_count; ++i) {
    const double range = ranges[i];
    if (isFiniteHit(range, range_min, range_max)) {
      samples[i] = RangeSample{range, true, true};
    } else if (isNoHit(range, range_max)) {
      samples[i] = RangeSample{range_max, true, false};
    }
  }

  const int radius = config_.median_filter_radius;
  const auto hit_window = [&](std::size_t i) {
      std::vector<std::size_t> indices;
      const int begin = std::max(0, static_cast<int>(i) - radius);
      const int end = std::min(static_cast<int>(range_count) - 1, static_cast<int>(i) + radius);
      for (int j = begin; j <= end; ++j) {
        if (samples[static_cast<std::size_t>(j)].hit) {
          indices.push_back(static_cast<std::size_t>(j));
        }
      }
      return indices;
    };

  if (radius > 0) {
    std::vector<RangeSample> median_filtered = samples;
    for (std::size_t i = 0; i < range_count; ++i) {
      if (!samples[i].hit) {
        continue;
      }

      const std::vector<std::size_t> neighbors = hit_window(i);
      if (neighbors.size() >= 2U) {
        std::vector<double> values;
        for (std::size_t neighbor : neighbors) {
          values.push_back(samples[neighbor].range);
        }
        median_filtered[i].range = medianOf(values);
      }
    }
    samples.swap(median_filtered);
  }

  if (radius > 0 && config_.outlier_jump_m > 0.0 && range_count > 1U) {
    std::vector<RangeSample> filtered = samples;
    for (std::size_t i = 0; i < range_count; ++i) {
      if (!samples[i].hit) {
        continue;
      }

      const std::vector<std::size_t> neighbors = hit_window(i);
      const bool supported = std::any_of(neighbors.begin(), neighbors.end(), [&](std::size_t neighbor) {
          return neighbor != i &&
            std::abs(samples[neighbor].range - samples[i].range) <= config_.outlier_jump_m;
        });
      if (!supported) {
        filtered[i] = RangeSample{};
      }
    }
    samples.swap(filtered);
  }

  return samples;
}

void StandaloneFrontierMap::markRayFreeCells(
  std::vector<std::uint8_t> & miss_mask,
  const Odom & odom,
  double angle_rad,
  double clear_range_m) const
{
  if (clear_range_m <= 0.0) {
    return;
  }

  const double step_m = std::max(0.01, config_.meters_per_pixel * 0.5);
  int previous_index = -1;
  const int step_count = static_cast<int>(std::ceil(clear_range_m / step_m));
  for (int step = 0; step <= step_count; ++step) {
    const double distance = std::min(clear_range_m, static_cast<double>(step) * step_m);
    const auto [world_x, world_y] = lidarPointToWorld(distance, angle_rad, odom);
    PixelRC pixel;
    if (!worldToPixel(world_x, world_y, &pixel)) {
      continue;
    }

    const int cell_index = index(pixel.row, pixel.col);
    if (cell_index == previous_index) {
      continue;
    }
    previous_index = cell_index;
    miss_mask[static_cast<std::size_t>(cell_index)] = 1U;
  }
}

std::vector<std::uint8_t> StandaloneFrontierMap::dilateMask(
  const std::vector<std::uint8_t> & mask,
  int radius_px) const
{
  if (radius_px <= 0) {
    return mask;
  }

  std::vector<std::uint8_t> dilated(mask.size(), 0U);
  for (int row = 0; row < config_.height; ++row) {
    for (int col = 0; col < config_.width; ++col) {
      if (mask[static_cast<std::size_t>(index(row, col))] == 0U) {
        continue;
      }
      forDiskPixels(row, col, radius_px, config_.width, config_.height,
        [&](int, int, std::size_t idx) { dilated[idx] = 1U; });
    }
  }
  return dilated;
}

std::vector<std::uint8_t> StandaloneFrontierMap::erodeMask(
  const std::vector<std::uint8_t> & mask,
  int radius_px) const
{
  if (radius_px <= 0) {
    return mask;
  }

  std::vector<std::uint8_t> eroded(mask.size(), 0U);
  for (int row = 0; row < config_.height; ++row) {
    for (int col = 0; col < config_.width; ++col) {
      if (diskAllSet(mask, row, col, radius_px, config_.width, config_.height)) {
        eroded[static_cast<std::size_t>(index(row, col))] = 1U;
      }
    }
  }
  return eroded;
}

std::vector<std::uint8_t> StandaloneFrontierMap::closeMask(
  const std::vector<std::uint8_t> & mask,
  int radius_px) const
{
  return erodeMask(dilateMask(mask, radius_px), radius_px);
}

std::vector<std::uint8_t> StandaloneFrontierMap::openMask(
  const std::vector<std::uint8_t> & mask,
  int radius_px) const
{
  return dilateMask(erodeMask(mask, radius_px), radius_px);
}

std::vector<std::uint8_t> StandaloneFrontierMap::reachableMask(
  const PixelRC & start,
  const std::vector<std::uint8_t> & free_mask) const
{
  std::vector<std::uint8_t> reachable(free_mask.size(), 0U);
  const auto start_index = static_cast<std::size_t>(index(start.row, start.col));
  if (free_mask[start_index] == 0U) {
    return reachable;
  }

  std::queue<PixelRC> pending;
  pending.push(start);
  reachable[start_index] = 1U;

  while (!pending.empty()) {
    const PixelRC current = pending.front();
    pending.pop();

    for (const auto & direction : kCardinalDirections) {
      const int neighbor_row = current.row + direction.row;
      const int neighbor_col = current.col + direction.col;
      if (!inBounds(neighbor_row, neighbor_col)) {
        continue;
      }

      const auto neighbor_index = static_cast<std::size_t>(index(neighbor_row, neighbor_col));
      if (reachable[neighbor_index] != 0U || free_mask[neighbor_index] == 0U) {
        continue;
      }
      reachable[neighbor_index] = 1U;
      pending.push(PixelRC{neighbor_row, neighbor_col});
    }
  }

  return reachable;
}

void StandaloneFrontierMap::rebuildGridFromLogOdds(const PixelRC & agent)
{
  std::vector<std::uint8_t> occupied(cells_.size(), 0U);
  std::vector<std::uint8_t> free(cells_.size(), 0U);

  for (std::size_t i = 0; i < cells_.size(); ++i) {
    if (log_odds_[i] >= config_.occupied_log_odds_threshold) {
      occupied[i] = 1U;
    } else if (log_odds_[i] <= config_.free_log_odds_threshold) {
      free[i] = 1U;
    }
  }

  occupied = openMask(occupied, radiusMetersToPixels(config_.morph_open_radius_m));
  occupied = closeMask(occupied, radiusMetersToPixels(config_.morph_close_radius_m));
  setMaskDisk(occupied, agent.row, agent.col, config_.robot_radius_m, 0U);
  setMaskDisk(free, agent.row, agent.col, config_.robot_radius_m, 1U);
  for (std::size_t i = 0; i < cells_.size(); ++i) {
    if (occupied[i] != 0U) {
      free[i] = 0U;
    }
  }

  std::vector<std::uint8_t> reachable = reachableMask(agent, free);
  reachable = erodeMask(reachable, radiusMetersToPixels(config_.reachable_erosion_radius_m));

  std::fill(cells_.begin(), cells_.end(), kGridCellUnknown);
  for (std::size_t i = 0; i < cells_.size(); ++i) {
    if (occupied[i] != 0U) {
      cells_[i] = kGridCellOccupied;
    } else if (reachable[i] != 0U) {
      cells_[i] = kGridCellFree;
    }
  }

  occupied_mask_.swap(occupied);
  reachable_free_mask_.swap(reachable);
}

std::pair<double, double> StandaloneFrontierMap::lidarPointToWorld(
  double range_m,
  double angle_rad,
  const Odom & odom) const
{
  const double local_forward = range_m * std::cos(angle_rad);
  const double local_left =
    (config_.lidar_y_positive_is_left ? 1.0 : -1.0) * range_m * std::sin(angle_rad);
  const double c = std::cos(odom.yaw);
  const double s = std::sin(odom.yaw);
  const double world_x = odom.x + c * local_forward - s * local_left;
  const double world_y = odom.y + s * local_forward + c * local_left;
  return {world_x, world_y};
}

bool StandaloneFrontierMap::hasUnknownNeighbor(int row, int col) const
{
  for (const auto & direction : kCardinalDirections) {
    const int neighbor_row = row + direction.row;
    const int neighbor_col = col + direction.col;
    if (inBounds(neighbor_row, neighbor_col) &&
      cells_[static_cast<std::size_t>(index(neighbor_row, neighbor_col))] == kGridCellUnknown)
    {
      return true;
    }
  }
  return false;
}

FrontierResult StandaloneFrontierMap::detectFrontiers(const Odom & odom) const
{
  FrontierResult result;
  result.width = config_.width;
  result.height = config_.height;
  result.meters_per_pixel = config_.meters_per_pixel;
  result.origin_x = config_.origin_x;
  result.origin_y = config_.origin_y;
  result.agent_in_bounds = worldToPixel(odom.x, odom.y, &result.agent_pixel);
  result.agent_yaw = odomYawToMapYaw(odom.yaw);
  result.raw_hit_count = countNonZero(raw_hit_mask_);
  result.reachable_free_count = countNonZero(reachable_free_mask_);

  const auto cell_count = static_cast<std::size_t>(config_.width * config_.height);
  std::vector<std::uint8_t> frontier_mask(cell_count, 0U);
  std::vector<std::uint8_t> visited(cell_count, 0U);

  for (int row = 0; row < config_.height; ++row) {
    for (int col = 0; col < config_.width; ++col) {
      const auto idx = static_cast<std::size_t>(index(row, col));
      const std::uint8_t cell = cells_[idx];
      result.free_count += cell == kGridCellFree ? 1U : 0U;
      result.occupied_count += cell == kGridCellOccupied ? 1U : 0U;
      result.unknown_count += cell == kGridCellUnknown ? 1U : 0U;
      if (result.agent_in_bounds && cell == kGridCellFree && hasUnknownNeighbor(row, col)) {
        frontier_mask[idx] = 1U;
      }
    }
  }
  if (!result.agent_in_bounds) {
    return result;
  }

  int next_id = 0;
  for (int row = 0; row < config_.height; ++row) {
    for (int col = 0; col < config_.width; ++col) {
      const auto start_idx = static_cast<std::size_t>(index(row, col));
      if (frontier_mask[start_idx] == 0U || visited[start_idx] != 0U) {
        continue;
      }

      FrontierSegment segment;
      std::queue<PixelRC> pending;
      pending.push(PixelRC{row, col});
      visited[start_idx] = 1U;

      double midpoint_row_sum = 0.0;
      double midpoint_col_sum = 0.0;

      while (!pending.empty()) {
        const PixelRC current = pending.front();
        pending.pop();
        ++segment.cell_count;

        for (const auto & direction : kCardinalDirections) {
          const int neighbor_row = current.row + direction.row;
          const int neighbor_col = current.col + direction.col;
          if (!inBounds(neighbor_row, neighbor_col) ||
            cells_[static_cast<std::size_t>(index(neighbor_row, neighbor_col))] != kGridCellUnknown)
          {
            continue;
          }

          midpoint_row_sum += static_cast<double>(current.row) + 0.5 +
            0.5 * static_cast<double>(direction.row);
          midpoint_col_sum += static_cast<double>(current.col) + 0.5 +
            0.5 * static_cast<double>(direction.col);
          ++segment.boundary_edge_count;
        }

        for (const auto & direction : kComponentDirections) {
          const int neighbor_row = current.row + direction.row;
          const int neighbor_col = current.col + direction.col;
          if (!inBounds(neighbor_row, neighbor_col)) {
            continue;
          }
          const auto neighbor_idx = static_cast<std::size_t>(index(neighbor_row, neighbor_col));
          if (frontier_mask[neighbor_idx] == 0U || visited[neighbor_idx] != 0U) {
            continue;
          }
          visited[neighbor_idx] = 1U;
          pending.push(PixelRC{neighbor_row, neighbor_col});
        }
      }

      if (segment.boundary_edge_count < config_.min_boundary_length_px) {
        continue;
      }

      const double edge_count = static_cast<double>(segment.boundary_edge_count);
      segment.midpoint.row = midpoint_row_sum / edge_count;
      segment.midpoint.col = midpoint_col_sum / edge_count;
      const double drow_m =
        (segment.midpoint.row - static_cast<double>(result.agent_pixel.row)) *
        config_.meters_per_pixel;
      const double dcol_m =
        (segment.midpoint.col - static_cast<double>(result.agent_pixel.col)) *
        config_.meters_per_pixel;
      const double distance_sq_m = drow_m * drow_m + dcol_m * dcol_m;
      if (distance_sq_m < config_.min_frontier_distance_m * config_.min_frontier_distance_m ||
        distance_sq_m > config_.max_frontier_distance_m * config_.max_frontier_distance_m)
      {
        continue;
      }
      const bool near_existing = std::any_of(
        result.frontiers.begin(),
        result.frontiers.end(),
        [&](const FrontierSegment & existing) {
          const double row_delta_m =
            (segment.midpoint.row - existing.midpoint.row) * config_.meters_per_pixel;
          const double col_delta_m =
            (segment.midpoint.col - existing.midpoint.col) * config_.meters_per_pixel;
          return row_delta_m * row_delta_m + col_delta_m * col_delta_m <
            kMinFrontierSeparationM * kMinFrontierSeparationM;
        });
      if (near_existing) {
        continue;
      }

      const auto [right_m, forward_m] = frontierPointToLocalXz(result, segment.midpoint);
      segment.id = next_id++;
      segment.local_right_m = right_m;
      segment.local_forward_m = forward_m;
      result.frontiers.push_back(segment);
    }
  }

  if (result.frontiers.empty()) {
    const auto near_existing = [&](const FrontierPointRC & point) {
        return std::any_of(
          result.frontiers.begin(),
          result.frontiers.end(),
          [&](const FrontierSegment & existing) {
            const double row_delta_m =
              (point.row - existing.midpoint.row) * config_.meters_per_pixel;
            const double col_delta_m =
              (point.col - existing.midpoint.col) * config_.meters_per_pixel;
            return row_delta_m * row_delta_m + col_delta_m * col_delta_m <
              kMinFrontierSeparationM * kMinFrontierSeparationM;
          });
      };
    const auto add_farthest_free_frontier = [&](double direction_right, double direction_forward) {
        bool found = false;
        FrontierPointRC best_point;
        double best_projection_m = 0.0;
        double best_lateral_m = 0.0;

        for (int row = 0; row < config_.height; ++row) {
          for (int col = 0; col < config_.width; ++col) {
            if (cells_[static_cast<std::size_t>(index(row, col))] != kGridCellFree) {
              continue;
            }

            const FrontierPointRC point{
              static_cast<double>(row) + 0.5,
              static_cast<double>(col) + 0.5};
            const auto [right_m, forward_m] = frontierPointToLocalXz(result, point);
            const double projection_m =
              right_m * direction_right + forward_m * direction_forward;
            if (projection_m <= 0.0) {
              continue;
            }
            const double lateral_m =
              std::abs(right_m * direction_forward - forward_m * direction_right);
            const bool better = !found ||
              projection_m > best_projection_m + 1e-9 ||
              (std::abs(projection_m - best_projection_m) <= 1e-9 &&
              lateral_m < best_lateral_m);
            if (better) {
              found = true;
              best_point = point;
              best_projection_m = projection_m;
              best_lateral_m = lateral_m;
            }
          }
        }

        if (!found || near_existing(best_point)) {
          return;
        }

        FrontierSegment segment;
        segment.id = next_id++;
        segment.cell_count = 1;
        segment.boundary_edge_count = 1U;
        segment.midpoint = best_point;
        const auto [right_m, forward_m] = frontierPointToLocalXz(result, segment.midpoint);
        segment.local_right_m = right_m;
        segment.local_forward_m = forward_m;
        result.frontiers.push_back(segment);
      };

    add_farthest_free_frontier(0.0, 1.0);
    add_farthest_free_frontier(0.0, -1.0);
    add_farthest_free_frontier(-1.0, 0.0);
    add_farthest_free_frontier(1.0, 0.0);
  }

  return result;
}

std::pair<double, double> StandaloneFrontierMap::frontierPointToLocalXz(
  const FrontierResult & result,
  const FrontierPointRC & point) const
{
  const double dcol_m =
    (point.col - static_cast<double>(result.agent_pixel.col)) * config_.meters_per_pixel;
  const double drow_m =
    (point.row - static_cast<double>(result.agent_pixel.row)) * config_.meters_per_pixel;
  const double c = std::cos(result.agent_yaw);
  const double s = std::sin(result.agent_yaw);
  const double right_m = c * dcol_m + s * drow_m;
  const double forward_m = s * dcol_m - c * drow_m;
  return {right_m, forward_m};
}

std::string resultToJson(const FrontierResult & result)
{
  std::ostringstream out;
  out.precision(17);
  out << "{";
  out << "\"width\":" << result.width << ",";
  out << "\"height\":" << result.height << ",";
  out << "\"meters_per_pixel\":" << result.meters_per_pixel << ",";
  out << "\"origin_x\":" << result.origin_x << ",";
  out << "\"origin_y\":" << result.origin_y << ",";
  out << "\"agent_pixel\":[" << result.agent_pixel.row << "," << result.agent_pixel.col << "],";
  out << "\"agent_in_bounds\":" << std::boolalpha << result.agent_in_bounds << ",";
  out << "\"agent_yaw\":" << result.agent_yaw << ",";
  out << "\"unknown_count\":" << result.unknown_count << ",";
  out << "\"free_count\":" << result.free_count << ",";
  out << "\"occupied_count\":" << result.occupied_count << ",";
  out << "\"raw_hit_count\":" << result.raw_hit_count << ",";
  out << "\"reachable_free_count\":" << result.reachable_free_count << ",";
  out << "\"frontiers\":[";
  for (std::size_t i = 0; i < result.frontiers.size(); ++i) {
    const FrontierSegment & frontier = result.frontiers[i];
    if (i > 0U) {
      out << ",";
    }
    out << "{";
    out << "\"id\":\"f" << frontier.id << "\",";
    out << "\"index\":" << frontier.id << ",";
    out << "\"cell_count\":" << frontier.cell_count << ",";
    out << "\"boundary_edge_count\":" << frontier.boundary_edge_count << ",";
    out << "\"midpoint_rc\":[" << frontier.midpoint.row << "," << frontier.midpoint.col << "],";
    out << "\"local_xz\":[" << frontier.local_right_m << "," << frontier.local_forward_m << "]";
    out << "}";
  }
  out << "]}";
  return out.str();
}

}  // namespace omninav_frontier
