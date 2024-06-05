
pointcloud to laserscan
roslaunch my_pkg point2scan.launch

navigation:
roslaunch jackal_navigation gmapping_demo.launch scan_topic:=/ouster/scan

RViz shwo:
roslaunch jackal_viz view_robot.launch config:=gmapping

Set goal postion:
rosrun my_pkg movebase_client_py.py 

Cancel all goal:
rosrun my_pkg stop_navigation.py 

## Updated

roslaunch my_pkg point2scan.launch

roslaunch jackal_navigation amcl_demo.launch scan_topic:=/my_laser map_file:=/home/jackal/mymap.yaml

roslaunch jackal_viz view_robot.launch config:=localization