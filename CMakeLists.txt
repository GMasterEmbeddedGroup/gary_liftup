cmake_minimum_required(VERSION 3.5)
project(gary_liftup)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gary_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


add_executable(liftup_teleop src/liftup_teleop.cpp src/utils/first_order_filter.cpp)
target_include_directories(liftup_teleop PRIVATE include)
ament_target_dependencies(
        liftup_teleop
        rclcpp
        std_msgs
        gary_msgs
        geometry_msgs
        rclcpp_components
)

add_library(liftup_teleop_lib SHARED src/liftup_teleop.cpp src/utils/first_order_filter.cpp)
target_include_directories(liftup_teleop_lib PRIVATE include)
ament_target_dependencies(
        liftup_teleop_lib
        rclcpp
        std_msgs
        gary_msgs
        geometry_msgs
        rclcpp_components
)

rclcpp_components_register_nodes(liftup_teleop_lib "gary_liftup::LiftupTeleop")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


install(TARGETS liftup_teleop
        DESTINATION lib/${PROJECT_NAME}
        )

install(TARGETS liftup_teleop_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

ament_export_dependencies(
        rclcpp
        std_msgs
        gary_msgs
        geometry_msgs
)

ament_package()
