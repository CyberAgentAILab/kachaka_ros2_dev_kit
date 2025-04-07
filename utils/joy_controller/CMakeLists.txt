cmake_minimum_required(VERSION 3.8)
project(joy_controller)

# use C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# enable all warnings 
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies with ament_cmake_auto
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# define the target libraries
ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/${PROJECT_NAME}_component.cpp
)

# register the component
rclcpp_components_register_nodes(${PROJECT_NAME}_component
  "joy_controller::JoyControllerComponent"
)
ament_export_libraries(${PROJECT_NAME}_component)

# add executable
ament_auto_add_executable(joy_controller_node
  src/joy_controller_node.cpp
)

# install directories
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# export the package
ament_auto_package()
