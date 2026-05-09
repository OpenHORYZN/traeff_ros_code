# Traeff ROS code

### How to run:
 ```
 pip install ompl
 pip install opencv-python
 cd ros_src
 colcon build
 source install/setup.bash
 # ros2 launch visualizer visualizer_launch.py only for debug
 # map needs the boxes publisher to be running
 ros2 run path_generator path_generator
 ros2 run map_generator map_generator
 ```