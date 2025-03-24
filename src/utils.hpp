#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <Eigen/Dense>
#include <vector>
#include <nav2_costmap_2d/costmap_2d.hpp>

namespace astar_path_planner
{

using Point = Eigen::Vector2d;

struct IsPointEqual
{
  bool operator()(const Point & left, const Point & right) const;
};

struct PointHash
{
  std::size_t operator()(const Point & point) const;
};

struct FrontierEntry
{
  std::vector<Point> path;
  double cost;
};

struct FrontierEntryComparator
{
  bool operator()(const FrontierEntry & a, const FrontierEntry & b) const;
};

std::vector<nav2_costmap_2d::MapLocation> PolygonForCircle(
  const nav2_costmap_2d::Costmap2D & costmap,
  const Point & center,
  const double & radius,
  const std::size_t & resolution = 20);

}  

#endif 
