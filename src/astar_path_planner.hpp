#ifndef ASTAR_PATH_PLANNER_HPP_
#define ASTAR_PATH_PLANNER_HPP_

#include <memory>
#include <queue>
#include <unordered_set>
#include <vector>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "utils.hpp"

namespace astar_path_planner
{

class AStarPathPlanner
{
public:
  using FrontierQueue = std::priority_queue<FrontierEntry, std::vector<FrontierEntry>,
      FrontierEntryComparator>;
  using ExpandedSet = std::unordered_set<Point, PointHash, PointEqualityComparator>;

  static void DeclareParameters(rclcpp_lifecycle::LifecycleNode::SharedPtr node);

  AStarPathPlanner(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> ros_costmap);

  std::vector<Point> Plan(const Point & start, const Point & goal);
  std::vector<Point> SmoothPath(const std::vector<Point>& path);

  const ExpandedSet & GetExpandedSet() const
  {
    return expanded_;
  }

private:
  rclcpp::Logger logger_;
  Point goal_;
  double goal_threshold_;
  double grid_size_;
  double collision_radius_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> ros_costmap_;
  ExpandedSet expanded_;
  FrontierQueue frontier_;
  // Differential drive robot parameters
    double wheel_base_;        // Distance between wheels
    double max_linear_vel_;    // Maximum linear velocity
    double max_angular_vel_;   // Maximum angular velocity
    double max_acceleration_;  // Maximum acceleration
    double min_turning_radius_; // Minimum turning radius

  void ExtendPathAndAddToFrontier(
    const std::vector<Point> & path, const double & path_cost,
    const Point & next_point);

  std::vector<Point> GetAdjacentPoints(const Point & point);
  



  double GetHeuristicCost(const Point & point);

  double GetStepCost(const Point & point, const Point & next);

  bool IsGoal(const Point & point);

  bool IsPointInCollision(const Point & point);
  
  std::vector<Point> GenerateMotionPrimitives(const Point& current, double current_heading);
  
  double GetKinodynamicCost(const Point& from, const Point& to, double current_heading);
  
  bool IsTrajectoryFeasible(const Point& from, const Point& to, double heading);
};

}  // namespace astar_path_planner

#endif  // ASTAR_PATH_PLANNER_HPP_
