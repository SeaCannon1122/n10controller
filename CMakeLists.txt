cmake_minimum_required(VERSION 3.8)
project(n10controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(X11 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(n10controller src/main.cpp src/controller_node.cpp src/screen_elements.cpp src/platform.cpp src/gui/pixel_char.cpp src/gui/menu.cpp src/gui/argb_image.cpp)

target_link_libraries(n10controller ${X11_LIBRARIES})

ament_target_dependencies(n10controller rclcpp std_msgs geometry_msgs std_srvs)

install(TARGETS
  n10controller
  DESTINATION lib/${PROJECT_NAME})

ament_package()
