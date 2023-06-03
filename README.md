# multi-robot-path-planning

## COSC81, Spring 2023, Final Project

Jason Pak, Jake Olson, Andres Ibarra

## Setup
1. Install Docker on your computer.
2. Run ROS using Docker by cloning & following the setup instructions in this [repository](https://github.com/quattrinili/vnc-ros)
3. Download [this archive](https://canvas.dartmouth.edu/courses/58298/files/folder/pa?preview=10795090) containing the environment (also included in this repo).
4. Run `docker-compose exec ros bash` in the terminal under the project package
4. Run `roscore` in a terminal.
5. Install `map_server` by running `sudo apt install ros-melodic-map-server`
6. Run `roslaunch launch/launcher.launch` in the `catkin_ws/src/[package_name]` root in another terminal. This will run the map server, start the simulation, initialize each robot node, and run the `global.py` python script.
7. Open your browser to `localhost:8080/vnc.html` and click connect to see the robot in action. 
8. Adjust the parameters in the code as needed (ex: goal destination, velocity, etc.)
9. To see the trail of the robot, you need to enable it from the simulator Stage GUI: View->Trails->Fast
10. To see the visualizations in rviz, run `rviz` in a terminal.
11. In rviz, you can add a Map under the topic `/map` and a PoseArray under the appropriate topic to visualize the planning algorithm.
