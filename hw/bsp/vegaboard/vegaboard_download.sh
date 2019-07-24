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

# Called with following variables set:
#  - CORE_PATH is absolute path to @apache-mynewt-core
#  - BSP_PATH is absolute path to hw/bsp/bsp_name
#  - BIN_BASENAME is the path to prefix to target binary,
#    .elf appended to name is the ELF file
#  - IMAGE_SLOT is the image slot to download to (for non-mfg-image, non-boot)
#  - FEATURES holds the target features string
#  - EXTRA_JTAG_CMD holds extra parameters to pass to jtag software
#  - MFG_IMAGE is "1" if this is a manufacturing image
#  - FLASH_OFFSET contains the flash offset to download to
#  - BOOT_LOADER is set if downloading a bootloader

. $CORE_PATH/hw/scripts/common.sh

CFG_RESET="reset halt"
OCD_BIN=openocd-rv32m1

#
# FILE_NAME must contain the name of the file to load
# FLASH_OFFSET must contain the offset in flash where to place it
#
openocd_load () {
    OCD_CMD_FILE=.openocd_cmds

    windows_detect
    parse_extra_jtag_cmd $EXTRA_JTAG_CMD

    if [ $WINDOWS -eq 1 ] ; then
        FILE_NAME=`cygpath -u $FILE_NAME`
    fi
    echo "$EXTRA_JTAG_CMD" > $OCD_CMD_FILE
    echo "init" >> $OCD_CMD_FILE
    echo "$CFG_RESET" >> $OCD_CMD_FILE
    echo "$CFG_POST_INIT" >> $OCD_CMD_FILE
    echo "flash write_image erase $FILE_NAME $FLASH_OFFSET" >> $OCD_CMD_FILE

    if [ -z $FILE_NAME ]; then
        echo "Missing filename"
        exit 1
    fi
    if [ ! -f "$FILE_NAME" ]; then
        # tries stripping current path for readability
        FILE=${FILE_NAME##$(pwd)/}
        echo "Cannot find file" $FILE
        exit 1
    fi
    if [ -z $FLASH_OFFSET ]; then
        echo "Missing flash offset"
        exit 1
    fi

    echo "Downloading" $FILE_NAME "to" $FLASH_OFFSET

    $OCD_BIN $CFG -f $OCD_CMD_FILE -c shutdown
    if [ $? -ne 0 ]; then
        exit 1
    fi
    rm $OCD_CMD_FILE
    return 0
}

openocd_halt () {
    $OCD_BIN $CFG -c init -c "halt" -c shutdown
    return $?
}

openocd_reset_run () {
    $OCD_BIN $CFG -c init -c "reset run" -c shutdown
    return $?
}

CFG="-f $CORE_PATH/hw/bsp/vegaboard/rv32m1_ri5cy.cfg"

if [ -z "$BIN_BASENAME" ]; then
    echo "Need binary to download"
    exit 1
fi

FILE_NAME=$BIN_BASENAME.elf.bin
#FLASH_OFFSET=0

echo "Downloading" $FILE_NAME "to" $FLASH_OFFSET
$OCD_BIN -d0 $CFG -c "init; reset halt; flash write_image erase $FILE_NAME; reset run; shutdown" 2>&1 | tee /tmp/newt.log
