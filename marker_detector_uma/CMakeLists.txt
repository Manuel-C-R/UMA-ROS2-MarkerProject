cmake_minimum_required(VERSION 3.8)
project(marker_detector_uma)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
FIND_PACKAGE(aruco REQUIRED )


include_directories(/usr/local/include/aruco)
link_directories(/usr/local/lib)
include_directories(include)
include_directories(calibration)

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


# HP40FractalProcessor
add_executable(HP40FractalProcessor src/HP40FractalProcessor.cpp)
target_link_libraries(HP40FractalProcessor /usr/local/lib/libaruco.so.3.1.15)
ament_target_dependencies(HP40FractalProcessor rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV aruco)

# GP7FractalProcessor
add_executable(GP7FractalProcessor src/GP7FractalProcessor.cpp)
target_link_libraries(GP7FractalProcessor /usr/local/lib/libaruco.so.3.1.15)
ament_target_dependencies(GP7FractalProcessor rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV aruco)

# ArucoProcessor
add_executable(ArucoProcessor src/ArucoProcessor.cpp)
ament_target_dependencies(ArucoProcessor rclcpp std_msgs OpenCV)

install(DIRECTORY calibration
        DESTINATION share/${PROJECT_NAME})

install(TARGETS
	HP40FractalProcessor
	GP7FractalProcessor
	ArucoProcessor
	DESTINATION lib/${PROJECT_NAME}
)


ament_package()
