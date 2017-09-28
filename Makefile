COMPILER = g++
FLAGS = -O3 -std=c++14 -Wall -Werror -Wextra

SO_DEPS = $(shell pkg-config --libs --cflags sfml-all libSimpleAmqpClient msgpack librabbitmq opencv theoradec theoraenc)
SO_DEPS += -lboost_program_options -lpthread 

all: charuco-calibration

clean:
	rm charuco-calibration

charuco-calibration: charuco-calibration.cpp 
	$(COMPILER) $^ -o $@ $(FLAGS) $(SO_DEPS) 