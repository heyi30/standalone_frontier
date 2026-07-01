#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace omninav_frontier
{

inline constexpr std::uint8_t kGridCellUnknown = 0U;
inline constexpr std::uint8_t kGridCellFree = 1U;
inline constexpr std::uint8_t kGridCellOccupied = 2U;

struct FrontierConfig
{
  int width{1024};
  int height{1024};
  double meters_per_pixel{0.1};
  double origin_x{0.0};
  double origin_y{0.0};
  double robot_radius_m{0.25};
  double obstacle_radius_m{0.15};
  bool lidar_y_positive_is_left{true};
  double min_frontier_distance_m{0.5};
  double max_frontier_distance_m{20.0};
  std::size_t min_boundary_length_px{20U};
};

struct Odom
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

struct PixelRC
{
  int row{0};
  int col{0};
};

struct FrontierPointRC
{
  double row{0.0};
  double col{0.0};
};

struct FrontierSegment
{
  int id{0};
  int cell_count{0};
  std::size_t boundary_edge_count{0U};
  FrontierPointRC midpoint;
  double local_right_m{0.0};
  double local_forward_m{0.0};
};

struct FrontierResult
{
  int width{0};
  int height{0};
  double meters_per_pixel{0.1};
  double origin_x{0.0};
  double origin_y{0.0};
  PixelRC agent_pixel;
  bool agent_in_bounds{false};
  double agent_yaw{0.0};
  std::vector<FrontierSegment> frontiers;
};

class StandaloneFrontierMap
{
public:
  struct Direction
  {
    int row;
    int col;
  };

  explicit StandaloneFrontierMap(FrontierConfig config);

  void reset();

  FrontierResult update(
    const double * ranges,
    std::size_t range_count,
    double angle_min,
    double angle_increment,
    double range_min,
    double range_max,
    const Odom & odom,
    bool reset_first);

  const FrontierConfig & config() const;
  const std::vector<std::uint8_t> & cells() const;
  void copyCellsTo(std::uint8_t * output, std::size_t output_count) const;

private:
  bool worldToPixel(double world_x, double world_y, PixelRC * pixel) const;
  bool inBounds(int row, int col) const;
  int index(int row, int col) const;
  std::uint8_t cellAt(int row, int col) const;
  void setCell(int row, int col, std::uint8_t value);
  void markDisk(int row, int col, double radius_m, std::uint8_t value);
  std::vector<PixelRC> linePixels(const PixelRC & start, const PixelRC & end) const;
  void traceFreeRay(const PixelRC & start, const PixelRC & end);
  void markUnknownPastObstacle(const PixelRC & obstacle, const PixelRC & truncation);
  std::pair<double, double> lidarPointToWorld(
    double range_m,
    double angle_rad,
    const Odom & odom) const;
  bool hasUnknownNeighbor(int row, int col) const;
  FrontierResult detectFrontiers(const Odom & odom) const;
  std::pair<double, double> frontierPointToLocalXz(
    const FrontierResult & result,
    const FrontierPointRC & point) const;

  FrontierConfig config_;
  std::vector<std::uint8_t> cells_;
};

std::string resultToJson(const FrontierResult & result);

}  // namespace omninav_frontier
