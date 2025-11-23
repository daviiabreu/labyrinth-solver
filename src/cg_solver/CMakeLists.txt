cmake_minimum_required(VERSION 3.8)
project(cg_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cg_interfaces REQUIRED)

include_directories(include)

add_executable(part1_solver
  src/part1_solver.cpp
  src/pathfinder.cpp
)
ament_target_dependencies(part1_solver
  rclcpp
  cg_interfaces
)
if(APPLE)
  set_target_properties(part1_solver PROPERTIES
    LINK_FLAGS "-Wl,-rpath,@loader_path/../../../../lib"
  )
endif()

add_executable(part2_mapper
  src/part2_mapper.cpp
  src/mapper.cpp
  src/pathfinder.cpp
)
ament_target_dependencies(part2_mapper
  rclcpp
  cg_interfaces
)
if(APPLE)
  set_target_properties(part2_mapper PROPERTIES
    LINK_FLAGS "-Wl,-rpath,@loader_path/../../../../lib"
  )
endif()

install(TARGETS
  part1_solver
  part2_mapper
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
