cmake_minimum_required(VERSION 3.8)
project(path_generator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)

ament_auto_find_build_dependencies()

ament_auto_add_library(resample_path SHARED
  src/resample_path.cpp
)

rclcpp_components_register_nodes(resample_path
  "path_generator::ResamplePath"
  EXECUTABLE resample_path
)



ament_auto_add_executable(path_generator_node
  src/path_generator_node.cpp
)

target_link_libraries(path_generator_node
  resample_path
)

install(
  DIRECTORY "include/"
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)


install(FILES DESTINATION share/${PROJECT_NAME})

ament_auto_package()