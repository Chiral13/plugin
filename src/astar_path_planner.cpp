

#include "astar_path_planner.hpp"
#include <memory>
#include <set>
#include <vector>
#include <queue>
#include <angles/angles.h>


namespace astar_path_planner
{

void AStarPathPlanner::DeclareParameters(rclcpp_lifecycle::LifecycleNode::SharedPtr node)
{
  node->declare_parameter("goal_threshold", 0.015);
  node->declare_parameter("grid_size", 0.01);
  node->declare_parameter("collision_radius", 0.08);
  node->declare_parameter("wheel_base", 0.16);  // Default for Turtlebot
    node->declare_parameter("max_linear_vel", 0.22);
    node->declare_parameter("max_angular_vel", 2.5);
    node->declare_parameter("max_acceleration", 0.5);
    node->declare_parameter("min_turning_radius", 0.1);
}

AStarPathPlanner::AStarPathPlanner(
  rclcpp_lifecycle::LifecycleNode::SharedPtr node,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> ros_costmap)
: logger_(node->get_logger()),
  ros_costmap_(ros_costmap)
{
  goal_threshold_ = node->get_parameter("goal_threshold").as_double();
  grid_size_ = node->get_parameter("grid_size").as_double();
  collision_radius_ = node->get_parameter("collision_radius").as_double();
  wheel_base_ = node->get_parameter("wheel_base").as_double();
    max_linear_vel_ = node->get_parameter("max_linear_vel").as_double();
    max_angular_vel_ = node->get_parameter("max_angular_vel").as_double();
    max_acceleration_ = node->get_parameter("max_acceleration").as_double();
    min_turning_radius_ = node->get_parameter("min_turning_radius").as_double();
}

std::vector<Point> AStarPathPlanner::Plan(const Point & start, const Point & goal)
{
    if (IsPointInCollision(goal)) {
        RCLCPP_ERROR(logger_, "Provided goal position would cause a collision");
        return {};
    }

    if (IsPointInCollision(start)) {
        RCLCPP_ERROR(
            logger_, "Starting position (%f, %f) is currently in a collision",
            start.x(), start.y());
        return {};
    }

    // Clear previous search data
    expanded_.clear();
    frontier_ = FrontierQueue{};
    goal_ = goal;
    
    // Initial heading (towards goal)
    double initial_heading = atan2(goal.y() - start.y(), goal.x() - start.x());
    
    // Create a new state structure that includes heading
    struct State {
        Point position;
        double heading;
        
        bool operator==(const State& other) const {
            return position.isApprox(other.position) && 
                   std::abs(angles::shortest_angular_distance(heading, other.heading)) < 0.1;
        }
    };
    
    // Hash function for State
    struct StateHash {
        std::size_t operator()(const State& state) const {
            std::size_t hash = 0;
            hash ^= std::hash<double>{}(std::round(state.position.x() * 1000)) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
            hash ^= std::hash<double>{}(std::round(state.position.y() * 1000)) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
            hash ^= std::hash<double>{}(std::round(state.heading * 10)) + 0x9e3779b9 + (hash << 6) + (hash >> 2);
            return hash;
        }
    };
    
    // Equality comparator for State
    struct StateEqual {
        bool operator()(const State& a, const State& b) const {
            return a == b;
        }
    };
    
    // Modified frontier entry to include heading
    struct HeadingFrontierEntry {
        std::vector<Point> path;
        std::vector<double> headings;
        double cost;
    };
    
    // Modified frontier comparator
    struct HeadingFrontierComparator {
        bool operator()(const HeadingFrontierEntry& a, const HeadingFrontierEntry& b) const {
            return a.cost > b.cost;
        }
    };
    
    using HeadingFrontierQueue = std::priority_queue<HeadingFrontierEntry, 
                                                    std::vector<HeadingFrontierEntry>, 
                                                    HeadingFrontierComparator>;
    
    using ExpandedStateSet = std::unordered_set<State, StateHash, StateEqual>;
    
    HeadingFrontierQueue heading_frontier;
    ExpandedStateSet expanded_states;
    
    // Initialize frontier with start state
    heading_frontier.push({{start}, {initial_heading}, GetHeuristicCost(start)});
    
    while (!heading_frontier.empty()) {
        // Get next state to expand
        const auto entry = heading_frontier.top();
        heading_frontier.pop();
        
        const auto path = entry.path;
        const auto headings = entry.headings;
        const auto cost = entry.cost;
        
        const auto last_state = path.back();
        const auto last_heading = headings.back();
        
        // Create current state
        State current_state = {last_state, last_heading};
        
        // Skip if we've already expanded this state
        if (expanded_states.count(current_state) > 0) {
            continue;
        }
        
        // Add state to expanded set
        expanded_states.insert(current_state);
        expanded_.insert(last_state);  // For visualization
        
        // Check if we've found our goal
        if (IsGoal(last_state)) {
            return path;
        }
        
        // Generate motion primitives based on current state and heading
        const auto neighbors = GenerateMotionPrimitives(last_state, last_heading);
        
        for (const auto& neighbor : neighbors) {
            // Skip if neighbor is in collision
            if (IsPointInCollision(neighbor)) {
                continue;
            }
            
            // Calculate new heading
            double new_heading = atan2(neighbor.y() - last_state.y(), 
                                      neighbor.x() - last_state.x());
            
            // Check if trajectory is feasible
            if (!IsTrajectoryFeasible(last_state, neighbor, last_heading)) {
                continue;
            }
            
            // Create new path and headings
            std::vector<Point> new_path(path);
            new_path.push_back(neighbor);
            
            std::vector<double> new_headings(headings);
            new_headings.push_back(new_heading);
            
            // Calculate new cost with kinodynamic constraints
            const auto step_cost = GetKinodynamicCost(last_state, neighbor, last_heading);
            const auto heuristic_diff = GetHeuristicCost(neighbor) - GetHeuristicCost(last_state);
            const auto new_cost = cost + step_cost + heuristic_diff;
            
            // Add to frontier
            heading_frontier.push({new_path, new_headings, new_cost});
        }
    }
    
    RCLCPP_ERROR(logger_, "No path found after exhausting search space.");
    return {};
}

std::vector<Point> AStarPathPlanner::SmoothPath(const std::vector<Point>& path)
{
    if (path.size() <= 2) {
        return path;  // No need to smooth paths with 2 or fewer points
    }
    
    std::vector<Point> smoothed_path;
    
    // Add the start point
    smoothed_path.push_back(path.front());
    
    // Use cubic B-spline for smoothing
    for (size_t i = 1; i < path.size() - 1; ++i) {
        // Get four control points (or fewer at the ends)
        std::vector<Point> control_points;
        for (int j = -1; j <= 2; ++j) {
            size_t idx = i + j;
            if (idx >= 0 && idx < path.size()) {
                control_points.push_back(path[idx]);
            }
        }
        
        // If we don't have enough control points, skip this point
        if (control_points.size() < 3) {
            continue;
        }
        
        // Generate points along the B-spline
        const int num_points = 5;  // Number of points to generate between each pair
        for (int t = 0; t < num_points; ++t) {
            double u = static_cast<double>(t) / num_points;
            
            // B-spline basis functions
            double b0, b1, b2, b3;
            if (control_points.size() == 4) {
                // Cubic B-spline
                b0 = (1-u)*(1-u)*(1-u)/6.0;
                b1 = (3*u*u*u - 6*u*u + 4)/6.0;
                b2 = (-3*u*u*u + 3*u*u + 3*u + 1)/6.0;
                b3 = u*u*u/6.0;
                
                Point spline_point = b0 * control_points[0] + 
                                    b1 * control_points[1] + 
                                    b2 * control_points[2] + 
                                    b3 * control_points[3];
                
                // Check if the spline point is collision-free
                if (!IsPointInCollision(spline_point)) {
                    smoothed_path.push_back(spline_point);
                }
            } else if (control_points.size() == 3) {
                // Quadratic B-spline
                b0 = (1-u)*(1-u)/2.0;
                b1 = (2*u*(1-u) + u*u)/2.0;
                b2 = u*u/2.0;
                
                Point spline_point = b0 * control_points[0] + 
                                    b1 * control_points[1] + 
                                    b2 * control_points[2];
                
                // Check if the spline point is collision-free
                if (!IsPointInCollision(spline_point)) {
                    smoothed_path.push_back(spline_point);
                }
            }
        }
    }
    
    // Add the goal point
    smoothed_path.push_back(path.back());
    
    return smoothed_path;
}


std::vector<Point> AStarPathPlanner::GenerateMotionPrimitives(const Point& current, double current_heading)
{
    std::vector<Point> primitives;
    
    // Calculate minimum turning radius based on max angular velocity
    double min_radius = max_linear_vel_ / max_angular_vel_;
    min_radius = std::max(min_radius, min_turning_radius_);
    
    // Generate forward motion primitive
    primitives.push_back(current + Point{grid_size_ * cos(current_heading), 
                                         grid_size_ * sin(current_heading)});
    
    // Generate turning primitives (left and right)
    const double angle_step = grid_size_ / min_radius;
    
    // Right turn
    primitives.push_back(current + Point{min_radius * sin(angle_step) * cos(current_heading - M_PI/2),
                                         min_radius * sin(angle_step) * sin(current_heading - M_PI/2)});
    
    // Left turn
    primitives.push_back(current + Point{min_radius * sin(angle_step) * cos(current_heading + M_PI/2),
                                         min_radius * sin(angle_step) * sin(current_heading + M_PI/2)});
    
    // Add more primitives with different turning radii for smoother paths
    double medium_radius = min_radius * 1.5;
    double large_radius = min_radius * 2.0;
    
    // Medium right turn
    primitives.push_back(current + Point{medium_radius * sin(angle_step) * cos(current_heading - M_PI/2),
                                         medium_radius * sin(angle_step) * sin(current_heading - M_PI/2)});
    
    // Medium left turn
    primitives.push_back(current + Point{medium_radius * sin(angle_step) * cos(current_heading + M_PI/2),
                                         medium_radius * sin(angle_step) * sin(current_heading + M_PI/2)});
    
    // Large right turn
    primitives.push_back(current + Point{large_radius * sin(angle_step) * cos(current_heading - M_PI/2),
                                         large_radius * sin(angle_step) * sin(current_heading - M_PI/2)});
    
    // Large left turn
    primitives.push_back(current + Point{large_radius * sin(angle_step) * cos(current_heading + M_PI/2),
                                         large_radius * sin(angle_step) * sin(current_heading + M_PI/2)});
    
    return primitives;
}


double AStarPathPlanner::GetKinodynamicCost(const Point& from, const Point& to, double current_heading)
{
    // Calculate direction of movement
    double new_heading = atan2(to.y() - from.y(), to.x() - from.x());
    
    // Calculate heading change
    double heading_change = std::abs(angles::shortest_angular_distance(current_heading, new_heading));
    
    // Calculate curvature (inverse of turning radius)
    double distance = (to - from).norm();
    double curvature = heading_change / distance;
    
    // Penalize sharp turns (high curvature)
    double curvature_cost = curvature * 10.0;
    
    // Penalize changes in velocity that would exceed acceleration limits
    double velocity_change_cost = 0.0;
    if (curvature > 0.001) {  // If turning
        double turning_radius = 1.0 / curvature;
        double max_vel_in_turn = std::min(max_linear_vel_, max_angular_vel_ * turning_radius);
        velocity_change_cost = std::max(0.0, max_linear_vel_ - max_vel_in_turn) * 5.0;
    }
    
    return distance + curvature_cost + velocity_change_cost;
}


bool AStarPathPlanner::IsTrajectoryFeasible(const Point& from, const Point& to, double heading)
{
    // Calculate direction of movement
    double new_heading = atan2(to.y() - from.y(), to.x() - from.x());
    
    // Calculate heading change
    double heading_change = std::abs(angles::shortest_angular_distance(heading, new_heading));
    
    // Calculate distance
    double distance = (to - from).norm();
    
    // Calculate curvature
    double curvature = heading_change / distance;
    
    // Check if curvature is within limits (based on min turning radius)
    if (curvature > 0.001) {  // If turning
        double turning_radius = 1.0 / curvature;
        if (turning_radius < min_turning_radius_) {
            return false;
        }
    }
    
    return true;
}



void AStarPathPlanner::ExtendPathAndAddToFrontier(
  const std::vector<Point> & path, const double & path_cost,
  const Point & next_point)
{
  
  std::vector<Point> new_path(path);
  new_path.push_back(next_point);
  const auto new_cost = path_cost - GetHeuristicCost(path.back()) +
    GetStepCost(path.back(), next_point) + GetHeuristicCost(next_point);
  frontier_.push({new_path, new_cost});
  
}

std::vector<Point> AStarPathPlanner::GetAdjacentPoints(const Point & point)
{
  
  std::vector<Point> neighbors;

  for (auto dx = -grid_size_; dx <= grid_size_; dx += grid_size_) {
    for (auto dy = -grid_size_; dy <= grid_size_; dy += grid_size_) {
      if (std::abs(dx) < 1e-4 && std::abs(dy) < 1e-4) {
        continue;
      }
      const Point neighbor = point + Point{dx, dy};
      if (!IsPointInCollision(neighbor)) {
        neighbors.push_back(neighbor);
      }
    }
  }
  return neighbors;
  
}


double AStarPathPlanner::GetHeuristicCost(const Point & point)
{
  
  return (point - goal_).norm();
  
}

double AStarPathPlanner::GetStepCost(const Point & point, const Point & next)
{
  
  return (next - point).norm();
  
}

bool AStarPathPlanner::IsGoal(const Point & point)
{
  
  return (point - goal_).norm() < goal_threshold_;
  
}

bool AStarPathPlanner::IsPointInCollision(const Point & point)
{
  auto costmap = ros_costmap_->getCostmap();

  const auto polygon = PolygonForCircle(*costmap, point, collision_radius_);

  std::vector<nav2_costmap_2d::MapLocation> map_cells;
  costmap->convexFillCells(polygon, map_cells);

  auto is_cell_lethal = [&costmap](const auto & cell) {
      if (cell.x < 0 || cell.y < 0 || cell.x >= costmap->getSizeInCellsX() ||
        cell.y >= costmap->getSizeInCellsY())
      {
        // out of bounds cells are assumed to be empty
        return false;
      }
      const auto cost = costmap->getCost(cell.x, cell.y);
      return cost == nav2_costmap_2d::LETHAL_OBSTACLE;
    };

  return std::any_of(map_cells.begin(), map_cells.end(), is_cell_lethal);
}

}  // namespace astar_path_planner
