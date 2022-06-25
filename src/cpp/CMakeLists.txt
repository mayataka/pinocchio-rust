cmake_minimum_required(VERSION 3.11)
project(pinocchio-rust CXX)

set(CMAKE_CXX_STANDARD 14)

#############
## Options ##
#############
option(OPTIMIZE_FOR_NATIVE "Enable -march=native" OFF)

##########################
## Build pinocchio-rust ##
##########################
if(NOT CMAKE_BUILD_TYPE MATCHES Release)
  message(STATUS "WARNING: CMAKE_BUILD_TYPE is NOT set to Release, which can decrease performance significantly.")
endif()
set(CMAKE_INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)
# find Eigen
find_package(Eigen3 REQUIRED)
# find pinocchio
find_package(pinocchio REQUIRED)
# build pinocchio-rust
# file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES src/*.cpp)
# file(GLOB_RECURSE ${PROJECT_NAME}_SOURCES src/*.cpp)
add_library(
  ${PROJECT_NAME} 
  STATIC
  src/multibody/model.cpp
  # ${${PROJECT_NAME}_SOURCES} 
)
message("${PINOCCHIO_LIBRARIES}")
target_link_libraries(
  ${PROJECT_NAME} 
  PRIVATE
  ${PINOCCHIO_LIBRARIES}
)
target_include_directories(
  ${PROJECT_NAME} 
  PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${PINOCCHIO_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if (OPTIMIZE_FOR_NATIVE)
  target_compile_options(
    ${PROJECT_NAME} 
    PUBLIC
    -march=native
  )
endif()

#############
## Install ##
#############
include(GNUInstallDirs)
# Install the static lib file at the build directory
install (
    TARGETS ${PROJECT_NAME} 
    DESTINATION ${PROJECT_BINARY_DIR}
)
# install(
#   TARGETS ${PROJECT_NAME}
#   EXPORT ${PROJECT_NAME}-config
#   ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
#   LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#   RUNTIME DESTINATION ${CMAKE_INSTALL_LIBDIR}/${PROJECT_NAME}
# )