cmake_minimum_required(VERSION 3.8...3.10)
project(player_pkg)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(geometry_msgs REQUIRED) 
find_package(OpenCV REQUIRED)
find_package(pose_msg REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(referee_pkg REQUIRED)
find_package(sensor_msgs REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wno-deprecated-declarations)
endif()

add_executable(sphere_vision_node src/sphere_vision.cpp)
ament_target_dependencies(sphere_vision_node
  rclcpp
  std_msgs
  OpenCV 
  geometry_msgs
  cv_bridge
  pose_msg
  referee_pkg
)
add_executable(rec_vision_node src/rec_vision.cpp)
ament_target_dependencies(rec_vision_node
  rclcpp
  std_msgs
  OpenCV 
  geometry_msgs
  cv_bridge
  pose_msg
  referee_pkg
  sensor_msgs
)
add_executable(armor_vision_node src/armor_node.cpp)
ament_target_dependencies(armor_vision_node
  rclcpp
  std_msgs
  OpenCV 
  geometry_msgs
  cv_bridge
  pose_msg
  referee_pkg
)
add_executable(multi_armor_vision_node src/multi_armors_node.cpp)
ament_target_dependencies(multi_armor_vision_node
  rclcpp
  std_msgs
  OpenCV 
  geometry_msgs
  cv_bridge
  pose_msg
  referee_pkg
)
add_executable(armor_shooter_node src/shooter_node.cpp)
ament_target_dependencies(armor_shooter_node
  rclcpp
  std_msgs
  geometry_msgs
  pose_msg
  referee_pkg
)
# 安装可执行文件
install(TARGETS sphere_vision_node rec_vision_node armor_vision_node multi_armor_vision_node armor_shooter_node
  DESTINATION lib/${PROJECT_NAME}
)

install (DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# 链接OpenCV库
target_link_libraries(sphere_vision_node ${OpenCV_LIBS})
target_link_libraries(rec_vision_node ${OpenCV_LIBS})
target_link_libraries(armor_vision_node ${OpenCV_LIBS})
target_link_libraries(multi_armor_vision_node ${OpenCV_LIBS})

# 为Vision_test添加消息类型支持（新方法）
target_link_libraries(sphere_vision_node ${typesupport_target})
target_link_libraries(rec_vision_node${typesupport_target})
target_link_libraries(armor_vision_node${typesupport_target})
target_link_libraries(multi_armor_vision_node${typesupport_target})
target_link_libraries(armor_shooter_node${typesupport_target})

# 包含OpenCV头文件
include_directories(${OpenCV_INCLUDE_DIRS})


ament_package()
