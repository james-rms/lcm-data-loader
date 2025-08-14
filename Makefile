# WASI SDK toolchain configuration. Download a WASI SDK from https://github.com/WebAssembly/wasi-sdk/releases/tag/wasi-sdk-25.0 and point WASI_SDK_PATH to it.
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

srcs:= \
	src/event_log.cpp \
	src/transcode.cpp \
	src/lcm_data_loader.cpp

objects:=\
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
	build/foxglove/CompressedImage.pb-c.o \
	build/foxglove/LaserScan.pb-c.o \
	build/foxglove/LocationFix.pb-c.o \
	build/foxglove/PackedElementField.pb-c.o \
	build/foxglove/PointCloud.pb-c.o \
	build/foxglove/Pose.pb-c.o \
	build/foxglove/PosesInFrame.pb-c.o \
	build/foxglove/Quaternion.pb-c.o \
	build/foxglove/Vector3.pb-c.o \
    build/google/timestamp.pb-c.o

all: lcm-loader/data-loader.wasm mitdgc-log-sample.lcm

.PHONY: builddir
builddir:
	mkdir -p build/foxglove
	mkdir -p build/google
	mkdir -p build/lcm

lcm-loader/data-loader.wasm: $(srcs) $(objects)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $^ -lm -Llib -lprotobuf-c -Isrc

mitdgc-log-sample.lcm:
	curl -o $@ https://grandchallenge.mit.edu/public/mitdgc-log-sample

build/lcm/%.o: src/lcm/%.c | builddir
	$(CC) -g -c -Wall $(CFLAGS) -o $@ $<  -Isrc

build/foxglove/%.o: src/foxglove/%.c | builddir
	$(CC) -g -c -Wall $(CFLAGS) -o $@ $<  -Isrc

build/google/%.o: src/google/protobuf/%.c | builddir
	$(CC) -g -c -Wall $(CFLAGS) -o $@ $<  -Isrc

clean:
	rm -r build

.PHONY: all clean
