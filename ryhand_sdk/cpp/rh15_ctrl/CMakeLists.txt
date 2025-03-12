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
  add_compile_options(-Wall -Wextra -Wpedantic)
  # add_compile_options(-Wno-unused-variable)
endif()


include_directories(include)
include_directories(lib)


find_library(RUIYAN_SRC_LIB
    NAMES Ryhand64
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib
    NO_DEFAULT_PATH  # 禁止在其他路径中搜索
)

# 确保库文件被找到
if (NOT RUIYAN_SRC_LIB)
    message(FATAL_ERROR "library not found!")
endif()
message(STATUS "library found! ${RUIYAN_SRC_LIB}")


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(rh15_cmd REQUIRED)
find_package(rh15_msg REQUIRED)



include_directories(include)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})



# Find required packages
# find_package(Pinocchio REQUIRED)

# Include directories
# include_directories(${PINOCCHIO_INCLUDE_DIRS})


add_executable(rh_ctrl src/ryhandlib_port.c src/can_socket.c src/rh15_ctrl.cpp )
target_include_directories(rh_ctrl PUBLIC include lib )
target_link_libraries(rh_ctrl ${RUIYAN_SRC_LIB})
ament_target_dependencies(rh_ctrl rclcpp rh15_cmd rh15_msg)


add_executable(rh_test src/rh15_test.cpp )
target_include_directories(rh_test PUBLIC include )
ament_target_dependencies(rh_test rclcpp rh15_cmd)


# add_executable(rh15_kinematics src/rh15_kinematics.cpp)
# target_link_libraries(rh15_kinematics ${PINOCCHIO_LIBRARIES})
# ament_target_dependencies(rh15_kinematics rclcpp)


install(TARGETS
  rh_ctrl rh_test
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

install(FILES hand_left.urdf
        DESTINATION share/${PROJECT_NAME}
)

# install(FILES R5_master.urdf
#         DESTINATION share/${PROJECT_NAME}
# )





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


