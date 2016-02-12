cmake_minimum_required(VERSION 2.8.11)

project(creator)


set(CMAKE_MODULE_PATH "/usr/lib/OGRE/cmake/;${CMAKE_MODULE_PATH}")

find_package(OGRE REQUIRED)





# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
# TODO: Look this up to make sure it is only scanning the 'creator' directory and if we can change the output directory of it
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON) # for compiling the ui forms



# TODO: Find core and gui as well
# Find the QtWidgets library
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Widgets)


# Tell CMake to create the helloworld executable
add_executable(creator
	main.cpp
	net.cpp
	mainwindow.cpp
	QTOgreWindow.cpp
	PoseWindow.cpp
)



include_directories(
	${OGRE_INCLUDE_DIRS}
	#${OGRE_SAMPLES_INCLUDEPATH}
	#${OGRE_Overlay_INCLUDE_DIRS}
)

set_target_properties(creator PROPERTIES COMPILE_FLAGS "-std=c++11")
#set_target_properties(OgreApp PROPERTIES DEBUG_POSTFIX _d)

target_link_libraries(creator ${OGRE_LIBRARIES} pthread)

target_link_libraries(creator Qt5::Widgets)

target_link_libraries(creator ${EXT_DIR}/libxbee3/lib/libxbee.so)

#file(GLOB xbee3_SRC
#    "ext/libxbee3/*.cpp"
#	"ext/libxbee3/*.c"
#)
#add_definitions(-DXBEE_API2)
#add_library(xbee3 SHARED ${xbee3_SRC})
#target_link_libraries(xbee3 pthread rt)
#set_target_properties(xbee3 PROPERTIES COMPILE_FLAGS "-fexceptions")
include_directories(${EXT_DIR}/libxbee3)


#add_executable(creator
#	src/creator/main.cpp
#)
#target_link_libraries(creator
#	quadctrl
#
#	#${CMAKE_BINARY_DIR}/ext/libxbee3/lib/libxbee.a
#	${EXT_DIR}/libxbee3/lib/libxbee.so
#)
