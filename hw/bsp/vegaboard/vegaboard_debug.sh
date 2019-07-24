#!/bin/sh -x

# Licensed to the Apache Software Foundation (ASF) under one
# or more contributor license agreements.  See the NOTICE file
# distributed with this work for additional information
# regarding copyright ownership.  The ASF licenses this file
# to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance
# with the License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
# KIND, either express or implied.  See the License for the
# specific language governing permissions and limitations
# under the License.
#

# Called with following variables set:
#  - CORE_PATH is absolute path to @apache-mynewt-core
#  - BSP_PATH is absolute path to hw/bsp/bsp_name
#  - BIN_BASENAME is the path to prefix to target binary,
#    .elf appended to name is the ELF file
#  - FEATURES holds the target features string
#  - EXTRA_JTAG_CMD holds extra parameters to pass to jtag software
#  - RESET set if target should be reset when attaching
#  - NO_GDB set if we should not start gdb to debug
#

. $CORE_PATH/hw/scripts/common.sh

GDB=riscv32-unknown-elf-gdb
FILE_NAME=$BIN_BASENAME.elf
CFG="-f $CORE_PATH/hw/bsp/vegaboard/rv32m1_ri5cy.cfg"
OCD_BIN=openocd-rv32m1

#
# NO_GDB should be set if gdb should not be started
# FILE_NAME should point to elf-file being debugged
#
openocd_debug () {
    OCD_CMD_FILE=.openocd_cmds

    windows_detect
    parse_extra_jtag_cmd $EXTRA_JTAG_CMD

    echo "gdb_port $PORT" > $OCD_CMD_FILE
    echo "telnet_port $(($PORT+1))" >> $OCD_CMD_FILE
    echo "$EXTRA_JTAG_CMD" >> $OCD_CMD_FILE

    if [ -z "$NO_GDB" ]; then
        if [ -z $FILE_NAME ]; then
            echo "Missing filename"
            exit 1
        fi
        if [ ! -f "$FILE_NAME" ]; then
            echo "Cannot find file" $FILE_NAME
            exit 1
        fi

        if [ $WINDOWS -eq 1 ]; then
            #
            # Launch openocd in a separate command interpreter, to make sure
            # it doesn't get killed by Ctrl-C signal from bash.
            #

            CFG=`echo $CFG | sed 's/\//\\\\/g'`
            $COMSPEC /C "start $COMSPEC /C $OCD_BIN -l openocd.log $CFG -f $OCD_CMD_FILE -c init -c halt"
        else
            #
            # Block Ctrl-C from getting passed to openocd.
            #
            set -m
            $OCD_BIN $CFG -f $OCD_CMD_FILE -c init -c halt >openocd.log 2>&1 &
            openocdpid=$!
            set +m
        fi

        GDB_CMD_FILE=.gdb_cmds

        echo "target remote localhost:$PORT" > $GDB_CMD_FILE
        if [ ! -z "$RESET" ]; then
            echo "mon reset halt" >> $GDB_CMD_FILE
        fi

        echo "$EXTRA_GDB_CMDS" >> $GDB_CMD_FILE

        if [ $WINDOWS -eq 1 ]; then
            FILE_NAME=`echo $FILE_NAME | sed 's/\//\\\\/g'`
            $COMSPEC /C "start $COMSPEC /C $GDB -x $GDB_CMD_FILE $FILE_NAME"
        else
            set -m
            $GDB -x $GDB_CMD_FILE $FILE_NAME
            set +m
            rm $GDB_CMD_FILE
            sleep 1
            if [ -d /proc/$openocdpid ] ; then
                kill -9 $openocdpid
            fi
        fi
    else
        # No GDB, wait for openocd to exit
        $OCD_BIN $CFG -f $OCD_CMD_FILE -c init -c halt
        return $?
    fi

    if [ ! -x "$COMSPEC" ]; then
        rm $OCD_CMD_FILE
    fi
    return 0
}

openocd_debug
