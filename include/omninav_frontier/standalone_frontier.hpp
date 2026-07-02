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
  double log_odds_hit{0.85};
  double log_odds_miss{-0.20};
  double log_odds_min{-4.0};
  double log_odds_max{4.0};
  double occupied_log_odds_threshold{1.20};
  double free_log_odds_threshold{-0.35};
  double endpoint_inflation_radius_m{0.25};
  double ray_endpoint_clearance_m{0.10};
  int median_filter_radius{1};
  double outlier_jump_m{0.75};
  double morph_close_radius_m{0.10};
  double morph_open_radius_m{0.10};
  double reachable_erosion_radius_m{0.0};
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
  std::size_t unknown_count{0U};
  std::size_t free_count{0U};
  std::size_t occupied_count{0U};
  std::size_t raw_hit_count{0U};
  std::size_t reachable_free_count{0U};
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

  void copyCellsTo(std::uint8_t * output, std::size_t output_count) const;
  void copyDebugLayersTo(
    std::uint8_t * raw_hit_output,
    std::uint8_t * occupied_output,
    std::uint8_t * reachable_free_output,
    std::size_t output_count) const;

private:
  struct RangeSample
  {
    double range{0.0};
    bool valid{false};
    bool hit{false};
  };

  bool worldToPixel(double world_x, double world_y, PixelRC * pixel) const;
  bool inBounds(int row, int col) const;
  int index(int row, int col) const;
  int radiusMetersToPixels(double radius_m) const;
  void copyMaskTo(
    const std::vector<std::uint8_t> & mask,
    std::uint8_t * output,
    std::size_t output_count) const;
  void setMaskDisk(
    std::vector<std::uint8_t> & mask,
    int row,
    int col,
    double radius_m,
    std::uint8_t value) const;
  void addLogOdds(std::size_t cell_index, double delta);
  void addLogOddsDisk(int row, int col, double radius_m, double delta);
  std::vector<RangeSample> filterRanges(
    const double * ranges,
    std::size_t range_count,
    double range_min,
    double range_max) const;
  void markRayFreeCells(
    std::vector<std::uint8_t> & miss_mask,
    const Odom & odom,
    double angle_rad,
    double clear_range_m) const;
  std::vector<std::uint8_t> dilateMask(
    const std::vector<std::uint8_t> & mask,
    int radius_px) const;
  std::vector<std::uint8_t> erodeMask(
    const std::vector<std::uint8_t> & mask,
    int radius_px) const;
  std::vector<std::uint8_t> closeMask(
    const std::vector<std::uint8_t> & mask,
    int radius_px) const;
  std::vector<std::uint8_t> openMask(
    const std::vector<std::uint8_t> & mask,
    int radius_px) const;
  std::vector<std::uint8_t> reachableMask(
    const PixelRC & start,
    const std::vector<std::uint8_t> & free_mask) const;
  void rebuildGridFromLogOdds(const PixelRC & agent);
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
  std::vector<double> log_odds_;
  std::vector<std::uint8_t> raw_hit_mask_;
  std::vector<std::uint8_t> occupied_mask_;
  std::vector<std::uint8_t> reachable_free_mask_;
};

std::string resultToJson(const FrontierResult & result);

}  // namespace omninav_frontier
