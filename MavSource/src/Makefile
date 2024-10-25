# Root Makefile (MavSource/Makefile)
# Developed and Maintained by Fehmi Yousfi for remote PX4 drone control

CONF_PATH ?= /tmp/mavconf.conf

.PHONY: all clean

all:
	# Call the Makefile in the TestigModule directory with CONF_PATH
	$(MAKE) -C TestigModule CONF_PATH=$(CONF_PATH)

clean:
	# Call the clean target in the TestigModule directory
	$(MAKE) -C TestigModule clean
