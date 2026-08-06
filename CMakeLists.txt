# fcl_distance — header-only extraction of fcl::distance from FCL 0.7.0.
#
# Consuming projects normally do NOT need this file: just add include/ to the
# include path and make sure Eigen 3 and libccd are reachable.  It exists so
# the bundled tests can be built, and so CMake users can `add_subdirectory()`
# and link the INTERFACE target.
#
#   add_subdirectory(fcl_distance)
#   target_link_libraries(my_app PRIVATE fcl_distance)

cmake_minimum_required(VERSION 3.10)
project(fcl_distance VERSION 0.7.0 LANGUAGES CXX)

add_library(fcl_distance INTERFACE)
add_library(fcl_distance::fcl_distance ALIAS fcl_distance)

target_include_directories(fcl_distance INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(fcl_distance INTERFACE cxx_std_11)

# --- Eigen 3 (headers only) ---------------------------------------------
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

# --- libccd (needs linking) ---------------------------------------------
find_package(ccd QUIET)
if(TARGET ccd)
  target_link_libraries(fcl_distance INTERFACE ccd)
else()
  find_path(CCD_INCLUDE_DIR NAMES ccd/ccd.h)
  find_library(CCD_LIBRARY NAMES ccd libccd)
  if(CCD_INCLUDE_DIR AND CCD_LIBRARY)
    target_include_directories(fcl_distance INTERFACE ${CCD_INCLUDE_DIR})
    target_link_libraries(fcl_distance INTERFACE ${CCD_LIBRARY})
  else()
    message(WARNING "fcl_distance: libccd not found; set CCD_INCLUDE_DIR and CCD_LIBRARY")
  endif()
endif()

# --- tests ---------------------------------------------------------------
option(FCL_DISTANCE_BUILD_TESTS "Build the fcl_distance tests" ON)
if(FCL_DISTANCE_BUILD_TESTS)
  enable_testing()

  add_executable(test_fcl_distance tests/test_fcl_distance.cpp)
  target_link_libraries(test_fcl_distance PRIVATE fcl_distance)
  add_test(NAME test_fcl_distance COMMAND test_fcl_distance)

  # Instantiates every registered dispatch-matrix entry: if this links, the
  # extracted closure is complete.
  add_executable(instantiate_all tests/instantiate_all.cpp)
  target_link_libraries(instantiate_all PRIVATE fcl_distance)
  add_test(NAME instantiate_all COMMAND instantiate_all)
endif()

install(DIRECTORY include/ DESTINATION include)
