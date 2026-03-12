cmake_minimum_required(VERSION 3.0.2)
project(StepEgoMapPose_msgs)

# 1. 确保添加了 message_generation 组件
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  message_generation  # <--- 必须有这一行！
)


# 4. 这一段代码在 CMakeLists.txt 中通常是被注释掉的，请取消注释并确保位置正确
add_message_files(
  FILES
  StepEgoMapPose.msg
)

# 2. 如果之前有 generate_messages，确保它在这里被调用
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

# 3. 设置 catkin 依赖
catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)