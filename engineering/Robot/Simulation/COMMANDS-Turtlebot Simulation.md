
# Launch

```
nvidia ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true
```


```
ros2 launch turtlebot4_bringup diagnostics.launch.py
Package 'turtlebot4_bringup' not found: "package 'turtlebot4_bringup' not found, searching: ['/opt/ros/humble']"
```




# Map Related Commands

```
ros2 run nav2_map_server map_saver_cli --help
Usage:
  map_saver_cli [arguments] [--ros-args ROS remapping args]

Arguments:
  -h/--help
  -t <map_topic>
  -f <mapname>
  --occ <threshold_occupied>
  --free <threshold_free>
  --fmt <image_format>
  --mode trinary(default)/scale/raw

NOTE: --ros-args should be passed at the end of command line
```

```
ros2 run nav2_map_server map_saver_cli -t map -f my_map
```




```
Hello



```







