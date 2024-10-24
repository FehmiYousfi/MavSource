# Root Makefile (MavSource/TestigModule/Makefile)
# Developed and Maintained by Fehmi Yousfi for remote PX4 drone control 

CC = gcc
MAVLINK_PATH = ../thirdparty/c_library_v1/common
MAVLINK_CMD_PATH = ../thirdparty/c_library_v1

CFLAGS = -Wall -Wextra -O2 -I../src -Wno-unused-parameter -Wno-unused-result -lpthread 
MAVLINK_FLAGS = -w -I$(MAVLINK_PATH) -I$(MAVLINK_CMD_PATH)

CONF_PATH ?= /tmp/mavconf.conf

LIB_MVLK_OBJ = libmavlink.o
LIB_MVLK_SRC = ../src/libmavlink.c

CSSL_OBJ = cssl.O
CSSL_SRC = ../src/cssl.c

TARGET = ../MavSource
SRC = main.c

all: $(TARGET)

$(TARGET): $(SRC) $(LIB_MVLK_OBJ) $(CSSL_OBJ)
	$(CC) $(CFLAGS) $(MAVLINK_FLAGS) -DCONF_PATH=\"$(CONF_PATH)\" -o $@ $^

$(LIB_MVLK_OBJ): $(LIB_MVLK_SRC)
	$(CC) $(CFLAGS) $(MAVLINK_FLAGS) -c -o $@ $<

$(CSSL_OBJ): $(CSSL_SRC) 
	$(CC) $(CFLAGS) -c -o $@ $< -D _GNU_SOURCE

clean:
	rm -f $(TARGET) $(LIB_MVLK_OBJ)

.PHONY: all clean
