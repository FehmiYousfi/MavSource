#!/bin/sh

LOG_DIR=/var/log/mavrouter
STDOUT_LOG=$LOG_DIR/mavrouter.log

[ ! -d $LOG_DIR ] && mkdir -p $LOG_DIR

MAVRouterPorted -c /etc/config/mavconf.conf -e 192.168.1.1:11001 >> $STDOUT_LOG 2>&1

