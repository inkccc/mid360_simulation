cmake_minimum_required(VERSION 3.5)
project(mid360_simulation)

# ============================================================
# 编译器设置
# ============================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# 查找依赖包
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS chrono)

# ============================================================
# 包含目录设置
# ============================================================
include_directories(include)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# ============================================================
# 构建 Gazebo 插件库
# ============================================================
add_library(mid360_plugin SHARED
  src/mid360_points_plugin.cpp
  src/mid360_ode_multiray_shape.cpp
)

target_include_directories(mid360_plugin PRIVATE include)
target_link_libraries(mid360_plugin
  ${GAZEBO_LIBRARIES}
  RayPlugin
  GpuRayPlugin
  ${Boost_LIBRARIES}
)
ament_target_dependencies(mid360_plugin
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  gazebo_dev
  gazebo_ros
  tf2_ros
)

# ============================================================
# 安装配置
# ============================================================

# 安装插件库到 lib 目录
install(TARGETS mid360_plugin
  LIBRARY DESTINATION lib
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装 URDF 文件
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# 安装 mesh 模型文件
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# 安装扫描模式 CSV 文件
install(DIRECTORY scan_mode/
  DESTINATION share/${PROJECT_NAME}/scan_mode
)

# ============================================================
# 导出配置
# ============================================================
ament_export_libraries(mid360_plugin)
ament_export_include_directories(include)

ament_package()
