cmake_minimum_required(VERSION 3.5)

project(mg-mapf)

find_package(Boost 1.58 REQUIRED COMPONENTS program_options)
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

# Enable C++14 and warnings
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include_directories(
   ${CMAKE_CURRENT_SOURCE_DIR}
)

# Examples

## cbs
add_executable(main
  main.cpp
)
target_link_libraries(main
  ${Boost_LIBRARIES}
  yaml-cpp
)
