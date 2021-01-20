cmake_minimum_required(VERSION 3.10)
project(steprdf VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Ceres REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_subdirectory(third_party/pybind11)

add_library(_eris MODULE src/solver.cpp src/laser_2d_solver.cpp src/wrapper.cpp)

target_include_directories(_eris PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CERES_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include>  
)

target_link_libraries(_eris PRIVATE ${CERES_LIBRARIES} Eigen3::Eigen pybind11::module)

set_target_properties(_eris
    PROPERTIES	
        PREFIX "${PYTHON_MODULE_PREFIX}"	
        SUFFIX "${PYTHON_MODULE_EXTENSION}"	
)

install(TARGETS _eris LIBRARY DESTINATION modules/_eris)
