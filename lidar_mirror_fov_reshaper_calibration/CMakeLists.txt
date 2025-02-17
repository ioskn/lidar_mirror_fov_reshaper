cmake_minimum_required(VERSION 3.9)
project(lidar_mirror_fov_reshaper_calibration)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
# find_package(PythonLibs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(NLopt REQUIRED)

find_package(lidar_mirror_fov_reshaper_transformation REQUIRED)

set(dependencies
    rclcpp
    sensor_msgs
    visualization_msgs
    pcl_ros
    tf2_ros
    tf2
    lidar_mirror_fov_reshaper_transformation
)
add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_11)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${NLOPT_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
  ament_index_cpp
  pcl_conversions
  rclcpp
  sensor_msgs
  tf2
  visualization_msgs
)
target_link_libraries(${PROJECT_NAME} ${PYTHON_LIBRARIES} ${NLOPT_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
