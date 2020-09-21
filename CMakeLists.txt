cmake_minimum_required(VERSION 3.10)
project(steprdf VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Ceres REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(eris SHARED src/solver.cpp)

target_include_directories(eris
                           PRIVATE 
target_link_libraries(eris PRIVATE)

target_include_directories(samson PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CERES_INCLUDE_DIRS})
  $<INSTALL_INTERFACE:include>  
)

target_link_libraries(eris PUBLIC Eigen3::Eigen)
