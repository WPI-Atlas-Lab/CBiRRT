ifneq ($(ROS_ROOT),)
include $(shell rospack find mk)/cmake.mk
else

.PHONY: all install clean

all: build/Makefile
	make -C build

install: build/Makefile
	make -C build install

clean:
	rm -rf build

build/Makefile: CMakeLists.txt
	mkdir -p build
	cd build && cmake ..

endif
