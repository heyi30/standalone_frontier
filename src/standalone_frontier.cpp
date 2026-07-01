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

void appendJsonBool(std::ostringstream & out, bool value)
{
  out << (value ? "true" : "false");
}

}  // namespace

StandaloneFrontierMap::StandaloneFrontierMap(FrontierConfig config)
: config_(config),
  cells_(static_cast<std::size_t>(config.width * config.height), kGridCellUnknown)
{
  if (config_.width <= 0 || config_.height <= 0) {
    throw std::runtime_error("width and height must be positive");
  }
  if (config_.meters_per_pixel <= 0.0) {
    throw std::runtime_error("meters_per_pixel must be positive");
  }
  if (config_.robot_radius_m < 0.0 || config_.obstacle_radius_m < 0.0) {
    throw std::runtime_error("robot_radius_m and obstacle_radius_m must be non-negative");
  }
  if (config_.min_frontier_distance_m < 0.0 || config_.max_frontier_distance_m < 0.0 ||
    config_.min_frontier_distance_m > config_.max_frontier_distance_m)
  {
    throw std::runtime_error("invalid frontier distance range");
  }
}

void StandaloneFrontierMap::reset()
{
  std::fill(cells_.begin(), cells_.end(), kGridCellUnknown);
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

  PixelRC agent;
  const bool agent_in_bounds = worldToPixel(odom.x, odom.y, &agent);
  if (agent_in_bounds) {
    markDisk(agent.row, agent.col, config_.robot_radius_m, kGridCellFree);
  }

  double angle = angle_min;
  for (std::size_t i = 0; i < range_count; ++i) {
    const double raw_range = ranges[i];
    const bool finite = std::isfinite(raw_range);
    double range = finite ? raw_range : range_max;
    range = std::min(range, range_max);

    if (range >= range_min && agent_in_bounds) {
      const auto [world_x, world_y] = lidarPointToWorld(range, angle, odom);
      PixelRC endpoint;
      if (worldToPixel(world_x, world_y, &endpoint)) {
        traceFreeRay(agent, endpoint);
        if (finite && raw_range <= range_max) {
          markDisk(endpoint.row, endpoint.col, config_.obstacle_radius_m, kGridCellOccupied);
        }
      }
    }

    angle += angle_increment;
  }

  if (agent_in_bounds) {
    markDisk(agent.row, agent.col, config_.robot_radius_m, kGridCellFree);
  }
  return detectFrontiers(odom);
}

const FrontierConfig & StandaloneFrontierMap::config() const
{
  return config_;
}

const std::vector<std::uint8_t> & StandaloneFrontierMap::cells() const
{
  return cells_;
}

void StandaloneFrontierMap::copyCellsTo(std::uint8_t * output, std::size_t output_count) const
{
  if (output == nullptr) {
    throw std::runtime_error("grid output pointer is null");
  }
  if (output_count != cells_.size()) {
    throw std::runtime_error("grid output size does not match configured map size");
  }
  std::memcpy(output, cells_.data(), cells_.size() * sizeof(std::uint8_t));
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

std::uint8_t StandaloneFrontierMap::cellAt(int row, int col) const
{
  return cells_[static_cast<std::size_t>(index(row, col))];
}

void StandaloneFrontierMap::setCell(int row, int col, std::uint8_t value)
{
  if (inBounds(row, col)) {
    cells_[static_cast<std::size_t>(index(row, col))] = value;
  }
}

void StandaloneFrontierMap::markDisk(int row, int col, double radius_m, std::uint8_t value)
{
  const int radius_px = std::max(1, static_cast<int>(std::ceil(radius_m / config_.meters_per_pixel)));
  for (int dr = -radius_px; dr <= radius_px; ++dr) {
    for (int dc = -radius_px; dc <= radius_px; ++dc) {
      if ((dr * dr + dc * dc) <= radius_px * radius_px) {
        setCell(row + dr, col + dc, value);
      }
    }
  }
}

void StandaloneFrontierMap::traceFreeRay(const PixelRC & start, const PixelRC & end)
{
  int row = start.row;
  int col = start.col;
  const int dcol = std::abs(end.col - start.col);
  const int drow = -std::abs(end.row - start.row);
  const int step_col = start.col < end.col ? 1 : -1;
  const int step_row = start.row < end.row ? 1 : -1;
  int error = dcol + drow;

  while (true) {
    setCell(row, col, kGridCellFree);
    if (row == end.row && col == end.col) {
      break;
    }
    const int e2 = 2 * error;
    if (e2 >= drow) {
      error += drow;
      col += step_col;
    }
    if (e2 <= dcol) {
      error += dcol;
      row += step_row;
    }
    if (!inBounds(row, col)) {
      break;
    }
  }
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
      cellAt(neighbor_row, neighbor_col) == kGridCellUnknown)
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
  if (!result.agent_in_bounds) {
    return result;
  }

  const auto cell_count = static_cast<std::size_t>(config_.width * config_.height);
  std::vector<std::uint8_t> frontier_mask(cell_count, 0U);
  std::vector<std::uint8_t> visited(cell_count, 0U);

  for (int row = 0; row < config_.height; ++row) {
    for (int col = 0; col < config_.width; ++col) {
      const auto idx = static_cast<std::size_t>(index(row, col));
      if (cells_[idx] == kGridCellFree && hasUnknownNeighbor(row, col)) {
        frontier_mask[idx] = 1U;
      }
    }
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
            cellAt(neighbor_row, neighbor_col) != kGridCellUnknown)
          {
            continue;
          }

          FrontierPointRC start;
          FrontierPointRC end;
          if (direction.row < 0) {
            start = {static_cast<double>(current.row), static_cast<double>(current.col)};
            end = {static_cast<double>(current.row), static_cast<double>(current.col + 1)};
          } else if (direction.row > 0) {
            start = {static_cast<double>(current.row + 1), static_cast<double>(current.col)};
            end = {static_cast<double>(current.row + 1), static_cast<double>(current.col + 1)};
          } else if (direction.col < 0) {
            start = {static_cast<double>(current.row), static_cast<double>(current.col)};
            end = {static_cast<double>(current.row + 1), static_cast<double>(current.col)};
          } else {
            start = {static_cast<double>(current.row), static_cast<double>(current.col + 1)};
            end = {static_cast<double>(current.row + 1), static_cast<double>(current.col + 1)};
          }

          midpoint_row_sum += (start.row + end.row) * 0.5;
          midpoint_col_sum += (start.col + end.col) * 0.5;
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

      const auto [right_m, forward_m] = frontierPointToLocalXz(result, segment.midpoint);
      segment.id = next_id++;
      segment.local_right_m = right_m;
      segment.local_forward_m = forward_m;
      result.frontiers.push_back(segment);
    }
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
  out << "\"agent_in_bounds\":";
  appendJsonBool(out, result.agent_in_bounds);
  out << ",";
  out << "\"agent_yaw\":" << result.agent_yaw << ",";
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
