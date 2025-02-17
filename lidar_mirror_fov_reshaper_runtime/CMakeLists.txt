cmake_minimum_required(VERSION 3.9)
project(lidar_mirror_fov_reshaper_runtime)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(laser_geometry REQUIRED)

find_package(lidar_mirror_fov_reshaper_transformation REQUIRED)


set(dependencies
    rclcpp
    sensor_msgs
    tf2_ros
    pcl_ros
    pcl_conversions
    laser_geometry
    lidar_mirror_fov_reshaper_transformation)

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)

target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME})


install(TARGETS
    ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
