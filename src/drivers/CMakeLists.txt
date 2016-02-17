
add_library(quadctrl_drivers SHARED
	optitrack/natnet/ClientCore.cpp
	optitrack/natnet/NatNetClient.cpp
)
target_link_libraries(quadctrl_drivers pthread)

#set_target_properties(quadctrl_drivers PROPERTIES COMPILE_FLAGS "-std=c++11 -w -g -O3") #  -g


add_executable(opti_test
	optitrack/test.cpp
)
target_link_libraries(opti_test quadctrl_drivers)
