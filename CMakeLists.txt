cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()


find_package( SDL REQUIRED )
find_package( SDL_image REQUIRED )
if( NOT SDL_FOUND )
   message( FATAL_ERROR "SDL not found!" )
endif( NOT SDL_FOUND )

link_libraries(
   ${SDL_LIBRARY}
   ${SDLIMAGE_LIBRARY}
   SDLmain
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# phi_keyboard_pilot
rosbuild_add_library(phi_keyboard_pilot src/phi_pilot.cpp)
rosbuild_add_executable(phi_keyboard_pilot src/phi_node_keyboard_pilot.cpp)

# phi_locate_circle
rosbuild_add_library(phi_locate_circle src/phi_follower.cpp)
rosbuild_add_library(phi_locate_circle src/phi_locate_circle.cpp)
rosbuild_add_executable(phi_locate_circle src/phi_node_locate_circle.cpp)