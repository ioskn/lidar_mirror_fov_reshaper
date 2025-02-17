cmake_minimum_required(VERSION 3.9)
project(lidar_mirror_fov_reshaper_transformation)
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(PythonLibs REQUIRED)


set(dependencies
    pcl_ros
    pcl_conversions
    sensor_msgs
    rclcpp)

include_directories(
    include
    ${PYTHON_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED src/${PROJECT_NAME}.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
    ${pcl_conversions_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}>
    $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_link_libraries(${PROJECT_NAME} ${PYTHON_LIBRARIES})

ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies})

install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION include)

install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(${PROJECT_NAME}_test test/${PROJECT_NAME}_test.cpp)
        
    ament_target_dependencies(${PROJECT_NAME}_test
        ${dependencies}
    )
    target_link_libraries(${PROJECT_NAME}_test ${PYTHON_LIBRARIES} ${PROJECT_NAME})
endif()

ament_package()
