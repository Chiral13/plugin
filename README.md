# Kinodynamic A* Global Planner for TurtleBot3 (ROS2 Humble)

A custom Nav2 global planner plugin implementing kinodynamic A* algorithm for TurtleBot3 navigation in Gazebo simulations.

## Prerequisites
- ROS 2 Humble ([installation instructions](https://docs.ros.org/en/humble/Installation.html))
- Gazebo Classic
- TurtleBot3 packages
- Nav2 package

  
## Installation
1. Clone this repository in your src folder
2. Build package using colcon build command
3. Source the environment using source install/setup.bash


## Usage
1. Launch simulation environment: ros2 launch nav2_bringup tb3_simulation_launch.py params_file:=/path/to/your_params.yaml
- Use argument **headless:=False** to start gazebo simulation
- A sample parameter file is given in the repository 


## Troubleshooting
- **Plugin not Found**: Ensure the package is properly sourced
- **Simulation not starting**: Before using the package flollow steps given at ([getting started](https://docs.nav2.org/getting_started/index.html))
  

   


