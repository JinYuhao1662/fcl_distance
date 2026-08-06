# fcl_distance — header-only extraction of mesh-vs-mesh fcl::distance from
# FCL 0.7.0.
#
# Consuming projects normally do NOT need this file: add include/ to the
# include path and make sure Eigen 3 is reachable.  It exists so the bundled
# test can be built, and so CMake users can add_subdirectory() and link the
# INTERFACE target.
#
#   add_subdirectory(fcl_distance)
#   target_link_libraries(my_app PRIVATE fcl_distance)
#
# Note: unlike upstream FCL this needs no libccd.  Mesh-vs-mesh distance never
# reaches the GJK/EPA solvers, so they are not part of the extraction.

cmake_minimum_required(VERSION 3.10)
project(fcl_distance VERSION 0.7.0 LANGUAGES CXX)

add_library(fcl_distance INTERFACE)
add_library(fcl_distance::fcl_distance ALIAS fcl_distance)

target_include_directories(fcl_distance INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(fcl_distance INTERFACE cxx_std_11)

# --- Eigen 3 (headers only, the single dependency) -----------------------
find_package(Eigen3 3.3 QUIET NO_MODULE)
if(TARGET Eigen3::Eigen)
  target_link_libraries(fcl_distance INTERFACE Eigen3::Eigen)
else()
  find_path(EIGEN3_INCLUDE_DIR NAMES signature_of_eigen3_matrix_library
            PATH_SUFFIXES eigen3 eigen)
  if(EIGEN3_INCLUDE_DIR)
    target_include_directories(fcl_distance INTERFACE ${EIGEN3_INCLUDE_DIR})
  else()
    message(WARNING "fcl_distance: Eigen 3 not found; set EIGEN3_INCLUDE_DIR")
  endif()
endif()

# --- tests ---------------------------------------------------------------
option(FCL_DISTANCE_BUILD_TESTS "Build the fcl_distance tests" ON)
if(FCL_DISTANCE_BUILD_TESTS)
  enable_testing()
  add_executable(test_mesh_distance tests/test_mesh_distance.cpp)
  target_link_libraries(test_mesh_distance PRIVATE fcl_distance)
  add_test(NAME test_mesh_distance COMMAND test_mesh_distance)
endif()

install(DIRECTORY include/ DESTINATION include)
