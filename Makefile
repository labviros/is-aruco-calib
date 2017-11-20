CXX = clang++
CXXFLAGS += -std=c++1z -Wall -Werror# -O2
DEBUGFLAGS = -g  -fsanitize=address -fno-omit-frame-pointer 
LDFLAGS += -L/usr/local/lib -I/usr/local/include `pkg-config --libs sfml-all protobuf librabbitmq libSimpleAmqpClient opencv`\
           -Wl,--no-as-needed -Wl,--as-needed -ldl -lboost_system -lboost_chrono -lboost_program_options\
					 -lboost_filesystem -lismsgs
PROTOC = protoc

LOCAL_PROTOS_PATH = ./msgs/

vpath %.proto $(LOCAL_PROTOS_PATH)

all: charuco-intrinsic-calib aruco-extrinsic-calib

charuco-intrinsic-calib: charuco-intrinsic-calib.o 
	$(CXX) $(DEBUGFLAGS) $^ $(LDFLAGS) -o $@

aruco-extrinsic-calib: aruco-extrinsic-calib.o 
	$(CXX) $(DEBUGFLAGS) $^ $(LDFLAGS) -o $@

.PRECIOUS: %.pb.cc
%.pb.cc: %.proto
	$(PROTOC) -I $(LOCAL_PROTOS_PATH) --cpp_out=. $<

clean:
	rm -f *.o *.pb.cc *.pb.h charuco-intrinsic-calib aruco-extrinsic-calib
