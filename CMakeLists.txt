cmake_minimum_required(VERSION 3.16)
project(omninav_frontier_standalone LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wpedantic)

add_library(omninav_frontier_core STATIC
  src/standalone_frontier.cpp
)
target_include_directories(omninav_frontier_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
set_target_properties(omninav_frontier_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

add_library(omninav_frontier SHARED
  src/frontier_c_api.cpp
)
target_include_directories(omninav_frontier
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(omninav_frontier
  PRIVATE
    omninav_frontier_core
)
set_target_properties(omninav_frontier PROPERTIES
  OUTPUT_NAME omninav_frontier
)

install(TARGETS omninav_frontier omninav_frontier_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
