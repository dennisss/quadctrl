

ROOT_DIR=$(shell pwd)



build: local arm-none-eabi


local:
	mkdir -p "$(ROOT_DIR)/build/local"
	cd "$(ROOT_DIR)/build/local"; cmake ../../ -DBUILD_CREATOR=ON
	$(MAKE) -C "$(ROOT_DIR)/build/local"

arm-none-eabi:
	mkdir -p "$(ROOT_DIR)/build/arm-none-eabi"
	cd "$(ROOT_DIR)/build/arm-none-eabi"; cmake ../../ -DBUILD_ONBOARD=ON -DCMAKE_TOOLCHAIN_FILE="$(ROOT_DIR)/cmake/teensy.toolchain.cmake"
	$(MAKE) -C "$(ROOT_DIR)/build/arm-none-eabi"




program:
	$(TOOLSPATH)/teensy_post_compile -file=$(basename $@) -path=$(shell pwd) -tools=$(TOOLSPATH)
	-$(TOOLSPATH)/teensy_reboot


clean:
	rm -r build
