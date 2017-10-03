COMPILER = g++
FLAGS = -O3 -std=c++14 -Wall -Werror -Wextra

SO_DEPS = $(shell pkg-config --libs --cflags sfml-all libSimpleAmqpClient msgpack librabbitmq opencv theoradec theoraenc)
SO_DEPS += -lboost_program_options -lpthread  -lboost_filesystem -lboost_system

all: charuco-intrinsic-calib aruco-extrinsic-calib

clean:
	rm charuco-intrinsic-calib aruco-extrinsic-calib

charuco-intrinsic-calib: charuco-intrinsic-calib.cpp
	$(COMPILER) $^ -o $@ $(FLAGS) $(SO_DEPS) 

aruco-extrinsic-calib: aruco-extrinsic-calib.cpp
	$(COMPILER) $^ -o $@ $(FLAGS) $(SO_DEPS) 