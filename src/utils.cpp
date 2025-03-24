#include "utils.hpp"
#include <algorithm>
#include <vector>

namespace astar_path_planner
{

bool IsPointEqual::operator()(const Point & left, const Point & right) const
{
  return left.isApprox(right);
}

std::size_t PointHash::operator()(const Point & point) const
{
  std::size_t hash = 0;
  hash ^= std::hash<int>{}(std::round(point.x() * 1000)) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
  hash ^= std::hash<int>{}(std::round(point.y() * 1000)) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
  return hash;
}

bool FrontierEntryComparator::operator()(const FrontierEntry & a, const FrontierEntry & b) const
{
  return a.cost > b.cost;
}

std::vector<nav2_costmap_2d::MapLocation> PolygonForCircle(
  const nav2_costmap_2d::Costmap2D & costmap,
  const Point & center,
  const double & radius,
  const std::size_t & resolution)
{
  std::vector<nav2_costmap_2d::MapLocation> polygon;

  auto generate_point = [&, angle = 0.0]() mutable {
      const double wx = (radius * std::cos(angle)) + center.x();
      const double wy = (radius * std::sin(angle)) + center.y();
      angle += (2 * M_PI) / resolution;
      int mx = 0;
      int my = 0;
      costmap.worldToMapNoBounds(wx, wy, mx, my);
      return nav2_costmap_2d::MapLocation{static_cast<unsigned int>(mx),
      static_cast<unsigned int>(my)};
    };

  std::generate_n(std::back_inserter(polygon), resolution, generate_point);

  return polygon;
}

}  
