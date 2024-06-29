cmake_minimum_required(VERSION 3.5)
project(miivii_gmsl_camera)

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
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
# find_package(OpenCV REQUIRED)

find_package(yaml-cpp REQUIRED)

find_library(MVGMSLCAMERA_NOOPENCV_LIBRARY
  NAMES mvgmslcamera_noopencv
  PATHS /opt/miivii/lib)
include_directories(/opt/miivii/include)

#添加自定的msg
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
	"msg/MiiviiTime.msg"
	DEPENDENCIES builtin_interfaces std_msgs
)
ament_export_dependencies(rosidl_default_runtime)


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

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

add_executable(miivii_gmsl_camera_node 
	src/miivii_gmsl_camera.cpp
	src/miivii_gmsl_camera_node.cpp
	)
target_include_directories(miivii_gmsl_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(miivii_gmsl_camera_node ${MVGMSLCAMERA_NOOPENCV_LIBRARY} -lyaml-cpp)

#必须设置,否则找不到自定义的msg
rosidl_target_interfaces(miivii_gmsl_camera_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")


target_compile_definitions(miivii_gmsl_camera_node PUBLIC MIIVII_NO_OPENCV)
ament_target_dependencies(miivii_gmsl_camera_node rclcpp std_msgs sensor_msgs image_transport camera_info_manager yaml-cpp)#OpenCV )
install(TARGETS
  miivii_gmsl_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

