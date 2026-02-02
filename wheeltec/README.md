# 真实机器人
+ 启动底盘
 roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch 

    + 修改urdf：   mini_4wd_robot.urdf 中 base_link的名字
    + 修改 robot_model_visualization.launch中 base_link的名字
    + 修改 robot_model_visualization.launch中 mini_4wd 对应的 laser坐标系的位置  (54/2cm-2.5cm = 27-2.5cm = 24.5cm = 0.245m)
    + 启动turn_on_wheeltec_robot.launch 中的movebase
    + sudo apt-get install ros-noetic-costmap-converter  ros-noetic-mbf-costmap-core   ros-noetic-mbf-msgs


+ 雷达
 roslaunch urg_node urg_lidar.launch 

+ KINECT dk 相机
roslaunch azure_kinect_ros_driver driver.launch


+ 激光建图
source ~/ws_cartographer/devel_isolated/setup.bash && roslaunch cartographer_ros wheeltec_2d_real.launch 

+ 键盘 
roslaunch wheeltec_robot_rc keyboard_teleop.launch


+ 机械臂
roslaunch rm_control rm_control.launch 
roslaunch rm_bringup rm_robot.launch 
roslaunch rm_bringup rm_robot_bringup_and_control.launch   # 只运行着一个就行

+ 实机视点规划
rosrun view_planning_real view_planning_real

+ 
rostopic pub /object_centor geometry_msgs/PointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
point:
  x: 0.0
  y: 1.0
  z: 0.0"

rostopic pub /stop_loop std_msgs/Bool  "data: false" 


rostopic pub /rm_driver/MoveJ_Cmd rm_msgs/MoveJ "joint: [0.2, 0.2, 0.2, 0.2, 0.2, 0.2] speed: 0.5" 


rostopic pub /rm_driver/MoveJ_Cmd rm_msgs/MoveJ "joint: [0.2, 0.2, 0.2, 0.2, 0.2, 0.2] 
speed: 0.5" 





# 真实机器人
+ 启动底盘
 roslaunch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch 

    + 修改urdf mini_4wd_robot.urdf 中 base_link的名字
    + 修改 robot_model_visualization.launch中 base_link的名字
    + 修改 robot_model_visualization.launch中 mini_4wd 对应的 laser坐标系的位置  (54/2cm-2.5cm = 27-2.5cm = 24.5cm = 0.245m)
    + 启动turn_on_wheeltec_robot.launch 中的movebase
    + sudo apt-get install ros-noetic-costmap-converter  ros-noetic-mbf-costmap-core   ros-noetic-mbf-msgs

+ 键盘 
roslaunch wheeltec_robot_rc keyboard_teleop.launch


+ 雷达
 roslaunch urg_node urg_lidar.launch 

+ 激光建图
source ~/ws_cartographer/devel_isolated/setup.bash && roslaunch cartographer_ros wheeltec_2d_real.launch 

+ KINECT dk 相机
roslaunch azure_kinect_ros_driver driver.launch


+ 机械臂
roslaunch rm_control rm_control.launch 
roslaunch rm_bringup rm_robot.launch 

+ 实机视点规划
rosrun view_planning_real view_plannin_real

+ 
rostopic pub /object_centor geometry_msgs/PointStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
point:
  x: 0.0
  y: 1.0
  z: 0.0"

rostopic pub /stop_loop std_msgs/Bool  "data: false" 


rostopic pub /rm_driver/MoveJ_Cmd rm_msgs/MoveJ "joint: [0.2, 0.2, 0.2, 0.2, 0.2, 0.2] speed: 0.5" 


rostopic pub /rm_driver/MoveJ_Cmd rm_msgs/MoveJ "joint: [0.2, 0.2, 0.2, 0.2, 0.2, 0.2] 
speed: 0.5" 

+ 边界探索
source ws_rrt/devel/setup.bash &&  roslaunch explore_lite explore_wheeltec.launch 
