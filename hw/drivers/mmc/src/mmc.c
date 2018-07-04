/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <hal/hal_gpio.h>
#include <disk/disk.h>
#include <mmc/mmc.h>
#include <mmc/mmc_bus.h>
#include <stdio.h>

#define MIN(n, m) (((n) < (m)) ? (n) : (m))

/* MMC commands used by the driver */
#define CMD0                (0)            /* GO_IDLE_STATE */
#define CMD1                (1)            /* SEND_OP_COND (MMC) */
#define CMD8                (8)            /* SEND_IF_COND */
#define CMD12               (12)           /* STOP_TRANSMISSION */
#define CMD16               (16)           /* SET_BLOCKLEN */
#define CMD17               (17)           /* READ_SINGLE_BLOCK */
#define CMD18               (18)           /* READ_MULTIPLE_BLOCK */
#define CMD24               (24)           /* WRITE_BLOCK */
#define CMD25               (25)           /* WRITE_MULTIPLE_BLOCK */
#define CMD55               (55)           /* APP_CMD */
#define CMD58               (58)           /* READ_OCR */
#define ACMD41              (0x80 + 41)    /* SEND_OP_COND (SDC) */

#define HCS                 ((uint32_t) 1 << 30)

/* Response types */
#define R1                  (0)
#define R1b                 (1)
#define R2                  (2)
#define R3                  (3)            /* CMD58 */
#define R7                  (4)            /* CMD8 */

/* R1 response status */
#define R_IDLE              (0x01)
#define R_ERASE_RESET       (0x02)
#define R_ILLEGAL_COMMAND   (0x04)
#define R_CRC_ERROR         (0x08)
#define R_ERASE_ERROR       (0x10)
#define R_ADDR_ERROR        (0x20)
#define R_PARAM_ERROR       (0x40)

#define START_BLOCK         (0xFE)
#define START_BLOCK_TOKEN   (0xFC)
#define STOP_TRAN_TOKEN     (0xFD)

#define BLOCK_LEN           (512)

/*
 * Amount of times to wait for initialization response
 */
#define MAX_INIT_TRIES      (50)

static uint8_t g_block_buf[BLOCK_LEN];
static struct mmc_bus *g_bus = NULL;

static int
error_by_response(uint8_t status)
{
    int rc = MMC_CARD_ERROR;

    if (status == 0) {
        rc = MMC_OK;
    } else if (status == 0xff) {
        rc = MMC_CARD_ERROR;
    } else if (status & R_IDLE) {
        rc = MMC_TIMEOUT;
    } else if (status & R_ERASE_RESET) {
        rc = MMC_ERASE_ERROR;
    } else if (status & R_ILLEGAL_COMMAND) {
        rc = MMC_INVALID_COMMAND;
    } else if (status & R_CRC_ERROR) {
        rc = MMC_CRC_ERROR;
    } else if (status & R_ERASE_ERROR) {
        rc = MMC_ERASE_ERROR;
    } else if (status & R_ADDR_ERROR) {
        rc = MMC_ADDR_ERROR;
    } else if (status & R_PARAM_ERROR) {
        rc = MMC_PARAM_ERROR;
    }

    return (rc);
}

static struct mmc_bus *
mmc_bus_by_id(uint8_t id)
{
    if (id != 0) {
        return NULL;
    }

    return g_bus;
}

static uint8_t
send_mmc_cmd(struct mmc_bus *bus, uint8_t cmd, uint32_t payload)
{
    int n;
    uint8_t status;
    uint8_t crc;

    if (cmd & 0x80) {
        /* TODO: refactor recursion? */
        /* TODO: error checking */
        send_mmc_cmd(bus, CMD55, 0);
    }

    /* 4.7.2: Command Format */
    bus->txrx_byte(bus, 0x40 | (cmd & ~0x80));

    bus->txrx_byte(bus, payload >> 24 & 0xff);
    bus->txrx_byte(bus, payload >> 16 & 0xff);
    bus->txrx_byte(bus, payload >>  8 & 0xff);
    bus->txrx_byte(bus, payload       & 0xff);

    /**
     * 7.2.2 Bus Transfer Protection
     *   NOTE: SD is in CRC off mode by default but CMD0 and CMD8 always
     *   require a valid CRC.
     *   NOTE: CRC can be turned on with CMD59 (CRC_ON_OFF).
     */
    crc = 0x01;
    switch (cmd) {
    case CMD0:
        crc = 0x95;
        break;
    case CMD8:
        crc = 0x87;
        break;
    }
    bus->txrx_byte(bus, crc);

    for (n = 255; n > 0; n--) {
        status = bus->txrx_byte(bus, 0xff);
        if ((status & 0x80) == 0) {
            break;
        }
    }

    return status;
}

/**
 * Initialize the MMC driver
 *
 * @param spi_num Number of the SPI channel to be used by MMC
 * @param spi_cfg Low-level device specific SPI configuration
 * @param ss_pin Number of SS pin if SW controlled, -1 otherwise
 * @param mmc_spi_cfg 
 *
 * @return 0 on success, non-zero on failure
 */
int
mmc_init(struct mmc_bus *bus, uint8_t *id)
{
    int rc = 0;
    int i;
    uint8_t status;
    uint8_t cmd_resp[4];
    os_time_t timeout;

    /* TODO: add support for multiple interfaces */
    g_bus = bus;
    *id = 0;

    bus->disable(bus);

    /**
     * NOTE: The state machine below follows:
     *       Section 6.4.1: Power Up Sequence for SD Bus Interface.
     *       Section 7.2.1: Mode Selection and Initialization.
     */

    /* give 10ms for VDD rampup */
    os_time_delay(OS_TICKS_PER_SEC / 100);

    bus->enable(bus);

    /* send the required >= 74 clock cycles */
    for (i = 0; i < 74; i++) {
        bus->txrx_byte(bus, 0xff);
    }

    /* put card in idle state */
    status = send_mmc_cmd(bus, CMD0, 0);

    /* No card inserted or bad card? */
    if (status != R_IDLE) {
        rc = error_by_response(status);
        goto out;
    }

    /* end of initialization, can now switch to high speed */
    //rc = bus->fast_clock(bus);
    //if (rc) {
    //    return (rc);
    //}

    /**
     * 4.3.13: Ask for 2.7-3.3V range, send 0xAA pattern
     *
     * NOTE: cards that are not compliant with "Physical Spec Version 2.00"
     *       will answer this with R_ILLEGAL_COMMAND.
     */
    status = send_mmc_cmd(bus, CMD8, 0x1AA);
    if (status != 0xff && !(status & R_ILLEGAL_COMMAND)) {

        /**
         * Ver2.00 or later SD Memory Card
         */

        /* Read the contents of R7 */
        for (i = 0; i < 4; i++) {
            cmd_resp[i] = bus->txrx_byte(bus, 0xff);
        }

        /* Did the card return the same pattern? */
        if (cmd_resp[3] != 0xAA) {
            rc = MMC_RESPONSE_ERROR;
            goto out;
        }

        /**
         * 4.3.13 Send Interface Condition Command (CMD8)
         *   Check VHS for 2.7-3.6V support
         */
        if (cmd_resp[2] != 0x01) {
            rc = MMC_VOLTAGE_ERROR;
            goto out;
        }

        /**
         * Wait for card to leave IDLE state or time out
         */

        timeout = os_time_get() + OS_TICKS_PER_SEC;
        for (;;) {
            status = send_mmc_cmd(bus, ACMD41, HCS);
            if ((status & R_IDLE) == 0 || os_time_get() > timeout) {
                break;
            }
            os_time_delay(OS_TICKS_PER_SEC / 10);
        }

        if (status) {
            rc = error_by_response(status);
            goto out;
        }

        /**
         * Check if this is an high density card
         */

        status = send_mmc_cmd(bus, CMD58, 0);
        for (i = 0; i < 4; i++) {
            cmd_resp[i] = bus->txrx_byte(bus, 0xff);
        }
        if (status == 0 && (cmd_resp[0] & (1 << 6))) {  /* FIXME: CCS */
            /**
             * TODO: if CCS is set this is an SDHC or SDXC card!
             *       SDSC uses byte addressing, SDHC/SDXC block addressing
             *
             *       can also check the whole voltage range supported here
             */
        }

    } else if (status != 0xff && status & R_ILLEGAL_COMMAND) {

        /**
         * Ver1.x SD Memory Card or Not SD Memory Card
         */

        /**
         * Wait for card to leave IDLE state or time out
         */

        timeout = os_time_get() + OS_TICKS_PER_SEC;
        for (;;) {
            status = send_mmc_cmd(bus, ACMD41, 0);
            if ((status & R_IDLE) == 0 || os_time_get() > timeout) {
                break;
            }
            os_time_delay(OS_TICKS_PER_SEC / 10);
        }

        if (status) {
            rc = error_by_response(status);
            goto out;
        }

    } else {
        rc = error_by_response(status);
        goto out;
    }

    //TODO
    //if S18R==1 && S18A==1
    //  CMD11
    //CMD2
    //CMD3

out:
    bus->disable(bus);
    return rc;
}

/**
 * Commands that return response in R1b format and write
 * commands enter busy state and keep return 0 while the
 * operations are in progress.
 */
static uint8_t
wait_busy(struct mmc_bus *bus)
{
    os_time_t timeout;
    uint8_t res;

    timeout = os_time_get() + OS_TICKS_PER_SEC / 2;
    do {
        res = bus->txrx_byte(bus, 0xff);
        if (res) {
            break;
        }
        os_time_delay(OS_TICKS_PER_SEC / 1000);
    } while (os_time_get() < timeout);

    printf("wait_busy, res=%d\n", res);
    return res;
}

/**
 * @return 0 on success, non-zero on failure
 */
int
mmc_read(uint8_t mmc_id, uint32_t addr, void *buf, uint32_t len)
{
    uint8_t cmd;
    uint8_t res;
    int rc;
    uint32_t n;
    size_t block_len;
    size_t block_count;
    os_time_t timeout;
    uint32_t block_addr;
    size_t offset;
    size_t index;
    size_t amount;
    struct mmc_bus *bus;

    printf("mmc_read, mmc_id=%d, addr=0x%lx, len=%lu\n", mmc_id, addr, len);

    //FIXME
    bus = mmc_bus_by_id(mmc_id);
    if (bus == NULL) {
        return (MMC_DEVICE_ERROR);
    }

    rc = MMC_OK;

    block_len = (len + BLOCK_LEN - 1) & ~(BLOCK_LEN - 1);
    block_count = block_len / BLOCK_LEN;
    block_addr = addr / BLOCK_LEN;
    offset = addr - (block_addr * BLOCK_LEN);

    bus->enable(bus);

    cmd = (block_count == 1) ? CMD17 : CMD18;
    res = send_mmc_cmd(bus, cmd, block_addr);
    if (res) {
        rc = error_by_response(res);
        printf("cmd=%d, rc=%d\n", cmd, rc);
        goto out;
    }

    /**
     * 7.3.3 Control tokens
     *   Wait up to 200ms for control token.
     */
    timeout = os_time_get() + OS_TICKS_PER_SEC / 5;
    do {
        res = bus->txrx_byte(bus, 0xff);
        if (res != 0xFF) {
            break;
        }
        os_time_delay(OS_TICKS_PER_SEC / 20);
    } while (os_time_get() < timeout);

    /**
     * 7.3.3.2 Start Block Tokens and Stop Tran Token
     */
    if (res != START_BLOCK) {
        printf("timeout\n");
        rc = MMC_TIMEOUT;
        goto out;
    }

    index = 0;
    while (block_count--) {
        for (n = 0; n < BLOCK_LEN; n++) {
            g_block_buf[n] = bus->txrx_byte(bus, 0xff);
        }

        /* TODO: CRC-16 not used here but would be cool to have */
        bus->txrx_byte(bus, 0xff);
        bus->txrx_byte(bus, 0xff);

        amount = MIN(BLOCK_LEN - offset, len);

        memcpy(((uint8_t *)buf + index), &g_block_buf[offset], amount);

        offset = 0;
        len -= amount;
        index += amount;
    }

    if (cmd == CMD18) {
        send_mmc_cmd(bus, CMD12, 0);
        wait_busy(bus);
    }

out:
    bus->disable(bus);
    return (rc);
}

/**
 * @return 0 on success, non-zero on failure
 */
int
mmc_write(uint8_t mmc_id, uint32_t addr, const void *buf, uint32_t len)
{
    uint8_t cmd;
    uint8_t res;
    uint32_t n;
    size_t block_len;
    size_t block_count;
    os_time_t timeout;
    uint32_t block_addr;
    size_t offset;
    size_t index;
    size_t amount;
    int rc;
    struct mmc_bus *bus;

    //FIXME
    bus = mmc_bus_by_id(mmc_id);
    if (bus == NULL) {
        return (MMC_DEVICE_ERROR);
    }

    block_len = (len + BLOCK_LEN - 1) & ~(BLOCK_LEN - 1);
    block_count = block_len / BLOCK_LEN;
    block_addr = addr / BLOCK_LEN;
    offset = addr - (block_addr * BLOCK_LEN);

    bus->enable(bus);

    /**
     * This code ensures that if the requested address doesn't align with the
     * beginning address of a sector, the initial bytes are first read to the
     * buffer to be then written later.
     *
     * NOTE: this code will never run when using a FS that is sector addressed
     * like FAT (offset is always 0).
     */
    if (offset) {
        res = send_mmc_cmd(bus, CMD17, block_addr);
        if (res != 0) {
            rc = error_by_response(res);
            goto out;
        }

        timeout = os_time_get() + OS_TICKS_PER_SEC / 5;
        do {
            res = bus->txrx_byte(bus, 0xff);
            if (res != 0xff) {
                break;
            }
            os_time_delay(OS_TICKS_PER_SEC / 20);
        } while (os_time_get() < timeout);

        if (res != START_BLOCK) {
            rc = MMC_CARD_ERROR;
            goto out;
        }

        for (n = 0; n < BLOCK_LEN; n++) {
            g_block_buf[n] = bus->txrx_byte(bus, 0xff);
        }

        bus->txrx_byte(bus, 0xff);
        bus->txrx_byte(bus, 0xff);
    }

    /* now start write */

    cmd = (block_count == 1) ? CMD24 : CMD25;
    res = send_mmc_cmd(bus, cmd, block_addr);
    if (res) {
        rc = error_by_response(res);
        goto out;
    }

    index = 0;
    while (block_count--) {
        /**
         * 7.3.3.2 Start Block Tokens and Stop Tran Token
         */
        if (cmd == CMD24) {
            bus->txrx_byte(bus, START_BLOCK);
        } else {
            bus->txrx_byte(bus, START_BLOCK_TOKEN);
        }

        amount = MIN(BLOCK_LEN - offset, len);
        memcpy(&g_block_buf[offset], ((uint8_t *)buf + index), amount);

        for (n = 0; n < BLOCK_LEN; n++) {
            bus->txrx_byte(bus, g_block_buf[n]);
        }

        /* CRC */
        bus->txrx_byte(bus, 0xff);
        bus->txrx_byte(bus, 0xff);

        /**
         * 7.3.3.1 Data Response Token
         */
        res = bus->txrx_byte(bus, 0xff);
        if ((res & 0x1f) != 0x05) {
            break;
        }

        if (cmd == CMD25) {
            wait_busy(bus);
        }

        offset = 0;
        len -= amount;
        index += amount;
    }

    if (cmd == CMD25) {
        bus->txrx_byte(bus, STOP_TRAN_TOKEN);
        wait_busy(bus);
    }

    res &= 0x1f;
    switch (res) {
    case 0x05:
        rc = MMC_OK;
        break;
    case 0x0b:
        rc = MMC_CRC_ERROR;
        break;
    case 0x0d:  /* passthrough */
    default:
        rc = MMC_WRITE_ERROR;
    }

    wait_busy(bus);

out:
    bus->disable(bus);
    return (rc);
}

/*
 *
 */
int
mmc_ioctl(uint8_t mmc_id, uint32_t cmd, void *arg)
{
    return 0;
}

/*
 *
 */
struct disk_ops mmc_ops = {
    .read  = &mmc_read,
    .write = &mmc_write,
    .ioctl = &mmc_ioctl,
};
