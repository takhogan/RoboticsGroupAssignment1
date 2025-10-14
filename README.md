### Instructions
Clone this with:\
git clone https://github.com/takhogan/RoboticsGroupAssignment1.git
Or preferably use ssh if you have it setup:\
git clone git@github.com:takhogan/RoboticsGroupAssignment1.git

### Objectives
- Visualize the robot in RVIZ.
  - RVIZ cannot render SDF files so the robot would just appear as a vector and would not have a proper robot model
  - Is this fine or do we need to create a URDF file and a publisher? **(Needs to be clarified with prof)**
- Visualize the robot in Gazebo.
- Drive the robot around using TeleopTwistKeyboard while in RVIZ.
- Drive the robot around using TeleopTwistKeyboard while in Gazebo.
- Set goal pose in RVIZ and take video of robot driving to goal pose in RVIZ
- ? Set goal pose in RVIZ and take video of robot driving to goal pose in Gazebo
  - It is not possible to set goal poses in Gazebo but may be good to set goal pose in RVIZ, visualize in Gazebo and take a video

### TODO
- Create a sdf file
  - one approach is to convert from urdf to sdf: (gz sdf -p robot.urdf > robot.sdf)
  - SDF file also needs to map teleop inputs / keyboard inputs to robot movement
  - Instead of creating a robot simulator node, how the robot responds to input is defined in the sdf file
- Fill out pid node src/gazebo_controller/gazebo_controller/pid_controller.py
  - This will be a regular ros2 node
- Setup ROS to gazebo message bridge
- Save RVIZ configuration to src/gazebo_controller/rviz/gazebo_bot.rviz
- Create a launch file that
  1. Based on parameters, launches a Gazebo or RVIZ world
  2. Creates the robot in the world - ros_gz_sim create
  3. setup ros/teleop to gazebo message bridge 
  4. Launches the PID controller
