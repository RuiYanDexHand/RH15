cmake_minimum_required(VERSION 3.5)
project(rh15_ctrl)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wno-unused-variable -Wno-cpp )
  # add_compile_options(-Wno-cpp)
endif()


include_directories(include)
include_directories(lib)


find_library(RUIYAN_SRC_LIB
    NAMES Ryhand64
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib
    NO_DEFAULT_PATH  # 禁止在其他路径中搜索
)

# 查找系统库，限制在系统目录下搜索
find_library(SYSTEM_FOO
  NAMES urdfdom_sensor urdfdom_model  urdfdom_model_state  urdfdom_world  octomap octomath    
  PATHS /usr/lib/x86_64-linux-gnu  # 系统路径
  NO_DEFAULT_PATH                  # 禁止搜索其他路径（如ROS路径）
)


if (NOT SYSTEM_FOO)
  message(FATAL_ERROR "System library 'foo' not found")
endif()
message(STATUS "System found! ${SYSTEM_FOO}")

# 确保库文件被找到
if (NOT RUIYAN_SRC_LIB)
    message(FATAL_ERROR "library not found!")
endif()
message(STATUS "library found! ${RUIYAN_SRC_LIB}")


include_directories(include)



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(rh15_cmd REQUIRED)
find_package(rh15_msg REQUIRED)

find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(pinocchio REQUIRED)



add_executable(rh_ctrl src/ryhandlib_port.c src/can_socket.c src/rh15_ctrl.cpp )
target_include_directories(rh_ctrl PUBLIC include lib )
target_link_libraries(rh_ctrl 
  ${RUIYAN_SRC_LIB}
  rclcpp::rclcpp
  pinocchio::pinocchio
  ament_index_cpp::ament_index_cpp
)
ament_target_dependencies(rh_ctrl rclcpp rh15_cmd rh15_msg pinocchio ament_index_cpp geometry_msgs Eigen3 eigen3_cmake_module)


add_executable(rh_test src/rh15_test.cpp )
target_include_directories(rh_test PUBLIC include )
ament_target_dependencies(rh_test rclcpp rh15_cmd)



add_executable(rh15_kinematics src/rh15_kinematics.cpp)
target_link_libraries(rh15_kinematics
  # ${SYSTEM_FOO}             # 系统库在前
  rclcpp::rclcpp
  pinocchio::pinocchio
  ament_index_cpp::ament_index_cpp
)
ament_target_dependencies(rh15_kinematics rclcpp pinocchio ament_index_cpp geometry_msgs Eigen3 eigen3_cmake_module)




install(TARGETS
  rh_ctrl rh_test rh15_kinematics
  DESTINATION lib/${PROJECT_NAME}
)


install(FILES ${RUIYAN_SRC_LIB}
  DESTINATION lib/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py")

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml")

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)




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

ament_package()


