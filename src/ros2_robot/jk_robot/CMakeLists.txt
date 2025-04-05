cmake_minimum_required(VERSION 3.8)
project(jk_robot)

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
find_package(rclcpp_lifecycle REQUIRED)
find_package(jk_robot_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
set(ROS2PACKAGES std_msgs rclcpp rclcpp_lifecycle jk_robot_msgs sensor_msgs)

find_package(Eigen3 REQUIRED) # 寻找eigen库
find_package(nlohmann_json 3.11.3 REQUIRED)
find_package(yaml-cpp REQUIRED)

#message(${RobotiqGripperInterface_INCLUDE_DIRS})
# 包含头文件
include_directories(${EIGEN3_INCLUDE_DIRS})

#include_directories(/usr/local/include/yaml-cpp)

#include_directories(${NLOHMANN_JSON_INCLUDE_DIRS})

# 处理目标文件
set(LIB_SRC_LIST src/robotState.cpp src/TCPClient.cpp src/TCPServer.cpp src/RobotStateUnifiedInterfaceWrapper.cpp src/JKRobotClient.cpp)
add_library(jk_robot_driver SHARED ${LIB_SRC_LIST})
add_executable(jk_robot_server src/jk_robot_server.cpp)
add_executable(jk_robot_client_test src/jk_robot_client_test.cpp)
set(OBJ_LIST jk_robot_driver jk_robot_server jk_robot_client_test)

## 设置头文件搜索路径
#foreach(OBJ ${OBJ_LIST})
#  target_include_directories(${OBJ} PUBLIC
#          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#          $<INSTALL_INTERFACE:include>)
#endforeach()

# 处理链接
target_link_libraries(jk_robot_driver nlohmann_json::nlohmann_json)
target_link_libraries(jk_robot_server jk_robot_driver nlohmann_json::nlohmann_json yaml-cpp)
target_link_libraries(jk_robot_client_test jk_robot_driver nlohmann_json::nlohmann_json yaml-cpp)
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



## 为了使下游文件可以访问，要做以下两步
#ament_export_targets(lib_jk_driver HAS_LIBRARY_TARGET)
## 注册 导出头文件
#install(
#        DIRECTORY include/
#        DESTINATION include
#)

# 注册 导出库文件
install(
        TARGETS jk_robot_driver # 告诉ros2有这么个目标（可执行文件或者库）
        EXPORT jk_robot_driver
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)
# 配置文件安装到install中
# install(FILES config/config.yaml
#         DESTINATION config)

# 目标文件安装到install中作为库
install(TARGETS jk_robot_server jk_robot_client_test
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
