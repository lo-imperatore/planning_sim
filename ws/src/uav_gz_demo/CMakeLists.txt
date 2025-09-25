cmake_minimum_required(VERSION 3.8)
project(uav_gz_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-transport REQUIRED)
find_package(gz-msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(cmdvel_pinger src/cmdvel_pinger.cpp)
ament_target_dependencies(cmdvel_pinger rclcpp geometry_msgs)
target_include_directories(cmdvel_pinger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(cmdvel_pinger PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS cmdvel_pinger
  DESTINATION lib/${PROJECT_NAME})


add_executable(x3_position_controller src/x3_position_controller.cpp)
  ament_target_dependencies(x3_position_controller
    rclcpp geometry_msgs nav_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs)
install(TARGETS x3_position_controller
    DESTINATION lib/${PROJECT_NAME})

add_executable(trajectory_streamer src/trajectory_streamer.cpp)
ament_target_dependencies(trajectory_streamer rclcpp geometry_msgs tf2)
install(TARGETS trajectory_streamer
  DESTINATION lib/${PROJECT_NAME})

add_executable(global_planner src/global_planner.cpp)
  ament_target_dependencies(global_planner
    rclcpp geometry_msgs nav_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs)
install(TARGETS global_planner
    DESTINATION lib/${PROJECT_NAME})


add_executable(trajectory_visualizer src/trajectory_visualizer.cpp)
  ament_target_dependencies(trajectory_visualizer
    rclcpp geometry_msgs nav_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs visualization_msgs)
  target_link_libraries(trajectory_visualizer gz-transport::gz-transport gz-msgs::gz-msgs)
install(TARGETS trajectory_visualizer
    DESTINATION lib/${PROJECT_NAME})  



install(DIRECTORY worlds launch models trajectories scripts pointclouds
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
