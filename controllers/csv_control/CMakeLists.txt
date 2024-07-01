cmake_minimum_required(VERSION 3.0)

get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)
project(${PROJECT})
set(CMAKE_C_COMPILER "/usr/bin/gcc")
set(CMAKE_CXX_COMPILER "/usr/bin/g++")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set (CMAKE_EXE_LINKER_FLAGS)
set (CMAKE_CXX_STANDARD 17)
set(CMAKE_C_FLAGS "-std=c99")
set(CMAKE_CXX_FLAGS "-std=c99 -Wno-error=deprecated-declarations -Wno-deprecated-declarations ")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/build")

find_package(Eigen3 3.4 REQUIRED NO_MODULE)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
# Get C or C++ sources in the current directory (only).
file(GLOB C_SOURCES *.c)
file(GLOB CPP_SOURCES *.cpp)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES})

# Set the  Webots home path (change it according to your installation method)
# set(WEBOTS_HOME "/Applications/Webots.app/Contents")
# set(WEBOTS_HOME "/snap/webots/current/usr/share/webots")
set(WEBOTS_HOME "/usr/local/webots")

# Link with the Webots controller library.
link_directories(${WEBOTS_HOME}/lib/controller)
set (LIBRARIES m ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
include_directories(${WEBOTS_HOME}/include/controller/c ${WEBOTS_HOME}/include/controller/cpp)

include_directories(${CMAKE_CURRENT_LIST_DIR}/../RobotSetup/)
add_library(RobotSetup ${CMAKE_CURRENT_LIST_DIR}/../RobotSetup/RobotSetup.cpp ${CMAKE_CURRENT_LIST_DIR}/../RobotSetup/Tools.cpp)
target_link_libraries(RobotSetup Eigen3::Eigen)

# Setup the target executable.
add_executable(${PROJECT} ${SOURCES})
target_link_libraries(${PROJECT} ${LIBRARIES} ${_CORE_LIBRARIES} Eigen3::Eigen RobotSetup)
# Copy the target executable at the right location.
add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy ${CMAKE_BINARY_DIR}/${PROJECT} ${CMAKE_SOURCE_DIR}
)
target_compile_definitions(${PROJECT} PRIVATE -DSIMULATION)