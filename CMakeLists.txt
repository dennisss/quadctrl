cmake_minimum_required(VERSION 2.8)
project(QuadCtrl)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


find_package(Eigen3 REQUIRED)


include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(include)



add_library(quadctrl
	src/model.cpp
	src/simulate.cpp
	src/attitude_ctrl.cpp
	src/pid.cpp
	src/position_ctrl.cpp
)
set_target_properties(quadctrl PROPERTIES COMPILE_FLAGS "-std=c++11 -w -g -O3") #  -g


add_executable(test
	src/main.cpp
)

set_target_properties(test PROPERTIES COMPILE_FLAGS "-std=c++11 -w -g -O3") #  -g
target_link_libraries(test quadctrl)



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
