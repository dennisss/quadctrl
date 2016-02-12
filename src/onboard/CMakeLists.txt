
#set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/teensy.toolchain.cmake")



### Teensy3 Core Libraries ###
file(GLOB core_SRC ${EXT_DIR}/cores/teensy3/*.c)
file(GLOB core_SRC2 ${EXT_DIR}/cores/teensy3/*.cpp)
add_library(core ${core_SRC} ${core_SRC2})
include_directories(${EXT_DIR}/cores/teensy3)



### i2c_t3 ###
add_library(i2c_t3 ${EXT_DIR}/i2c_t3/i2c_t3.cpp)
include_directories(${EXT_DIR}/i2c_t3)
target_link_libraries(i2c_t3 core)


### xbee-arduino ###
add_library(xbee-arduino ${EXT_DIR}/xbee-arduino/XBee.cpp)
include_directories(${EXT_DIR}/xbee-arduino)
target_link_libraries(xbee-arduino core)


### i2cdevlib ###
add_library(i2cdevlib
	${EXT_DIR}/i2cdevlib/Arduino/I2Cdev/I2Cdev.cpp
	${EXT_DIR}/i2cdevlib/Arduino/MPU9150/MPU9150.cpp
)
include_directories(${EXT_DIR}/i2cdevlib/Arduino/I2Cdev ${EXT_DIR}/i2cdevlib/Arduino/MPU9150)
target_link_libraries(i2cdevlib core i2c_t3)


add_executable(onboard
	main.cpp
	motors.cpp
	mpu.cpp
	net.cpp
)

set_target_properties(onboard PROPERTIES COMPILE_FLAGS "-std=c++11 -w -g -O3") #  -g
target_link_libraries(onboard core i2cdevlib xbee-arduino quadctrl)

set_target_properties(onboard PROPERTIES SUFFIX ".elf")
add_custom_command(TARGET onboard POST_BUILD COMMAND arm-none-eabi-objcopy -O ihex -R .eeprom ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/onboard.elf ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/onboard.hex)
#set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES onboard.hex)
