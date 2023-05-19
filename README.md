# multi-robot-path-planning

## COSC81, Spring 2023, Final Project

Jason Pak, Jake Olson, Andres Ibarra

## Setup
1. Install Docker on your computer.
2. Run ROS using Docker by cloning & following the setup instructions in this [repository](https://github.com/quattrinili/vnc-ros)
3. Download [this archive](https://canvas.dartmouth.edu/courses/58298/files/folder/pa?preview=10795090) containing the environment (also included in this repo).
4. Run `roscore` in a terminal.
5. Install `map_server` by running `sudo apt install ros-melodic-map-server`
6. Run `rosrun map_server map_server maze.yml` in a terminal.
7. Start the simulation by running `rosrun stage_ros stageros maze.world` in a terminal.
8. For each robot in the simulation, use the terminal to call a command to run the transform mapTodom. For instance, robot_0 with an initial pose of [ 2.0 2.0 0.0 0.0 ] would need `rosrun tf static_transform_publisher 2.0 2.0 0.0 0.0 0.0 0.0 /map /robot_0/odom 100`.
9. Place the `pathplanning.py` code in your workspace and execute by running `python pathplanning.py` in a terminal.
10. Open your browser to `localhost:8080/vnc.html` and click connect to see the robot in action. 
11. Adjust the parameters in the code as needed (ex: goal destination, velocity, etc.)
12. To see the trail of the robot, you need to enable it from the simulator Stage GUI: View->Trails->Fast
13. To see the visualizations in rviz, run `rviz` in a terminal.
14. In rviz, you can add a Map under the topic `/map` and a PoseArray under the appropriate topic to visualize the planning algorithm.
