cmake_minimum_required(VERSION 3.8)
project(PointCloud_Registration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 找到依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(OpenMP REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

# 添加可执行文件
add_executable(PointCloud_Registration src/PointCloud_Registration.cpp)

# 链接依赖库
ament_target_dependencies(
  PointCloud_Registration
  rclcpp
  pcl_conversions
  pcl_ros
)

# 链接 OpenMP
target_link_libraries(PointCloud_Registration
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)

# 安装可执行文件
install(TARGETS
  PointCloud_Registration
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件和启动文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
