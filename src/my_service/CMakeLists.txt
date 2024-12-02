cmake_minimum_required(VERSION 3.8)
project(my_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)

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

# We add an executable macro to this CMAKE file so that you can run this node via ros2 run. 
add_executable(server src/my_service.cpp)
ament_target_dependencies(server rclcpp std_srvs geometry_msgs) # here we list the node dependencies

add_executable(client src/my_client.cpp)
ament_target_dependencies(client rclcpp std_srvs geometry_msgs)

#enables ros2 run to find the executable.
install(TARGETS 
        server
        client
        DESTINATION lib/${PROJECT_NAME})

# need to tell ament_package where to find the package so we run the line above.
ament_package()
