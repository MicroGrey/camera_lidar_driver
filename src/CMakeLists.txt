cmake_minimum_required(VERSION 3.15)
project(sensor)

# Use C++17
set(CMAKE_CXX_STANDARD 17)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rclcpp_components REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CERES_INCLUDE_DIRS})
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/sensor_node.cpp
)

ament_target_dependencies(${PROJECT_NAME} 
  rclcpp
  rclcpp_components
  cv_bridge
)

target_include_directories(${PROJECT_NAME} PUBLIC hikSDK/include)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})


# found hikvision camera SDK directory automatically according to device architecture
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(${PROJECT_NAME} PUBLIC hikSDK/lib/amd64)
  install(
    DIRECTORY hikSDK/lib/amd64/
    DESTINATION lib
  )
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(${PROJECT_NAME} PUBLIC hikSDK/lib/arm64)
  install(
    DIRECTORY hikSDK/lib/arm64/
    DESTINATION lib
  )
else()
  message(FATAL_ERROR "Unsupport host system architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_link_libraries(${PROJECT_NAME}
  FormatConversion
  MediaProcess
  MvCameraControl
  MVRender
  MvUsb3vTL
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN sensor::SensorNode
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)