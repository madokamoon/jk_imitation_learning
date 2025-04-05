cmake_minimum_required(VERSION 3.8)
project(robotiq)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(jk_robot_msgs REQUIRED)
set(ROS2PACKAGES std_msgs rclcpp jk_robot_msgs)

find_package(nlohmann_json 3.11.3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_library(ROBOTIQ_LIBRARY NAMES robotiq-gripper-interface PATHS /usr/local/lib/)

# 处理目标文件
add_executable(robotiq_2F85_server src/robotiq_2F85_server.cpp)
set(OBJ_LIST robotiq_2F85_server)

# 处理链接
target_link_libraries(robotiq_2F85_server nlohmann_json::nlohmann_json yaml-cpp ${ROBOTIQ_LIBRARY})
# 给目标文件链接ROS2功能包
foreach(OBJ ${OBJ_LIST})
  target_include_directories(${OBJ} PUBLIC
          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>)
  target_compile_features(${OBJ} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  ament_target_dependencies(
          ${OBJ}
          ${ROS2PACKAGES}
  )
endforeach()

install(TARGETS robotiq_2F85_server
        DESTINATION lib/${PROJECT_NAME})

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