# WASI SDK toolchain configuration. Download a WASI SDK from https://github.com/WebAssembly/wasi-sdk/releases/ and point WASI_SDK_PATH to it.
WASI_SDK_PATH ?= wasi-sdk-25.0-arm64-macos
CC := $(WASI_SDK_PATH)/bin/clang
CXX := $(WASI_SDK_PATH)/bin/clang++
LD := $(WASI_SDK_PATH)/bin/lld
CFLAGS := -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE --target=wasm32-wasi
CXXFLAGS :=  -Wall -Werror \
		-mexec-model=reactor \
		-fno-exceptions \
		--target=wasm32-wasi
LDFLAGS := --target=wasm32-wasi

sdk_srcs:= \
	foxglove_data_loader_sdk/src/foxglove/error.cpp \
	foxglove_data_loader_sdk/src/foxglove/schemas.cpp

srcs:= \
	src/event_log.cpp \
	src/transcode.cpp \
	src/lcm_data_loader.cpp

lcm_objects:=\
	build/lcm/eventlog.o \
	build/lcm/lcmtypes_gps_to_local_t.o \
	build/lcm/lcmtypes_pose_t.o \
	build/lcm/lcmtypes_image_t.o \
	build/lcm/lcmtypes_velodyne_t.o \
	build/lcm/lcmtypes_laser_t.o \
	build/lcm/config.o \
	build/lcm/rotations.o \
	build/lcm/math_util.o \
	build/lcm/velodyne.o \
	build/lcm/config_util.o \
	build/lcm/camtrans.o \

all: lcm-loader/data-loader.wasm mitdgc-log-sample.lcm

.PHONY: builddir
builddir:
	mkdir -p build/lcm

lcm-loader/data-loader.wasm: $(srcs) $(lcm_objects) $(sdk_srcs)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $^ -lm \
		-Lfoxglove_data_loader_sdk/lib \
		-Isrc \
		-Ifoxglove_data_loader_sdk/include \
		-lfoxglove

mitdgc-log-sample.lcm:
	curl -o $@ https://grandchallenge.mit.edu/public/mitdgc-log-sample

build/lcm/%.o: src/lcm/%.c | builddir
	$(CC) -g -c -Wall $(CFLAGS) -o $@ $<  -Isrc

clean:
	rm -r build

.PHONY: all clean
