# Variables
CXX = g++
CC = gcc
CXXFLAGS = -std=c++11 -DPACKAGE="mavlink-routerd" -I. -Imodules/mavlink_c_library_v2/ardupilotmega -Isrc -Wno-address-of-packed-member -Wno-return-type
LDFLAGS = -lrt -lm -lpthread

SRC_FILES = \
	src/autolog.cpp \
	src/binlog.cpp \
	src/dedup.cpp \
	src/endpoint.cpp \
	src/git_version.cpp \
	src/logendpoint.cpp \
	src/main.cpp \
	src/mainloop.cpp \
	src/pollable.cpp \
	src/timeout.cpp \
	src/ulog.cpp \
	src/tlog.cpp \
	src/common/conf_file.cpp \
	src/common/log.cpp \
	src/common/util.cpp \
	src/common/xtermios.cpp

TARGET = mavlink-routerd

all: $(TARGET)

$(TARGET): $(SRC_FILES)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(AIO_LIB) $(LDFLAGS)

src/git_version.h: src/version.h.in
	echo "Nothing" 

$(TARGET): src/git_version.h

clean:
	rm -f $(TARGET) src/git_version.h

.PHONY: all clean

