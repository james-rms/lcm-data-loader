# WASI SDK toolchain configuration
WASI_SDK_PATH ?= ../../wasi-sdk-25.0-arm64-macos
CC := $(WASI_SDK_PATH)/bin/clang
CFLAGS := -D_FILE_OFFSET_BITS=64 -D_LARGEFILE_SOURCE --target=wasm32-wasi
LDFLAGS := --target=wasm32-wasi

objects:=eventlog.o \
   lcmtypes_gps_to_local_t.o \
   lcmtypes_pose_t.o \
   lcmtypes_image_t.o \
   lcmtypes_velodyne_t.o \
   lcmtypes_laser_t.o \
   config.o \
   rotations.o \
   math_util.o \
   velodyne.o \
   config_util.o \
   camtrans.o

examples:=example1-poses.wasm \
	example2-images.wasm \
	example3-gps.wasm \
	example4-velodyne.wasm \
	example5-sick.wasm

all: $(examples)

example1-poses.wasm: example1-poses.o $(objects)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

example2-images.wasm: example2-images.o $(objects)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

example3-gps.wasm: example3-gps.o $(objects)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

example4-velodyne.wasm: example4-velodyne.o $(objects)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

example5-sick.wasm: example5-sick.o $(objects)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

%.o: %.c
	$(CC) -g -c -Wall $(CFLAGS) -o $@ $< 

clean:
	rm -f $(examples) *.o

.PHONY: all clean
