cmake_minimum_required(VERSION 2.8)
project(QuadCtrl)

option(BUILD_ROS OFF)
option(BUILD_CREATOR OFF)
option(BUILD_ONBOARD OFF)


# CMAKE_ARCHIVE_OUTPUT_DIRECTORY
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

set(EXT_DIR ${CMAKE_SOURCE_DIR}/ext)



find_package(Eigen3 REQUIRED)


include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(include)



add_library(quadctrl SHARED
	src/model.cpp
	src/simulate.cpp
	src/attitude_ctrl.cpp
	src/pid.cpp
	src/position_ctrl.cpp
	src/MadgwickAHRS.cpp
)
set_target_properties(quadctrl PROPERTIES COMPILE_FLAGS "-std=c++11 -w -g -O3") #  -g

include_directories(src)


#add_executable(hello
#	src/main.cpp
#)
#
#set_target_properties(hello PROPERTIES COMPILE_FLAGS "-std=c++11 -w -g -O3") #  -g
#target_link_libraries(hello quadctrl)



if(ROS)

	find_package(roscpp REQUIRED)

	include_directories(${roscpp_INCLUDE_DIRS})


	add_executable(sim
		src/rsim.cpp
	)
	target_link_libraries(sim ${roscpp_LIBRARIES} quadctrl)


	add_executable(simrun
		src/simrun.cpp
	)
	target_link_libraries(simrun ${roscpp_LIBRARIES} quadctrl)

endif()




if(BUILD_CREATOR)
	add_subdirectory(src/creator)
endif()

if(BUILD_ONBOARD)
	add_subdirectory(src/onboard)
endif()


# set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/teensy-arm.toolchain.cmake")
# Need to have target for arm,
# Compile 'cores' and 'i2c_t3' as libs

