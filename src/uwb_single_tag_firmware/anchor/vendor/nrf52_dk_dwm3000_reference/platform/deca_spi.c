/*! ----------------------------------------------------------------------------
 * @file    deca_spi.c
 * @brief   SPI access functions
 *
 * @attention
 *
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 * Copyright 2019 (c) Frederic Mes, RTLOC.
 * Copyright 2021 (c) Callender-Consulting LLC.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#include "deca_spi.h"
#include "deca_device_api.h"
#include "port.h"

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <hal/nrf_spim.h>
#include <hal/nrf_gpio.h>

#define LOG_LEVEL 3
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(deca_spi);

const struct device * spi;
struct spi_config * spi_cfg;
struct spi_config   spi_cfgs [4] = {0};

#define SPI_CFGS_COUNT ((sizeof(spi_cfgs)/sizeof(spi_cfgs[0])))

uint8_t tx_buf [255];
uint8_t rx_buf [255];

struct spi_buf bufs [2];

struct spi_buf_set tx;
struct spi_buf_set rx;

static struct spi_cs_control cs_ctrl;
static bool transfer_failed;

int spi_transfer_faulted(void)
{
    return transfer_failed;
}
#define RAW_SPI_NODE DT_PARENT(DT_INST(0, qorvo_dwm3000))
#define RAW_SPI ((NRF_SPIM_Type *)DT_REG_ADDR(RAW_SPI_NODE))
#define RAW_CS_PIN DT_PHA(RAW_SPI_NODE, cs_gpios, pin)

/* This dedicated DW3000 peripheral is owned by this polling port. Short DMA
 * transactions must not wait for an RTOS completion signal. No other caller
 * uses the Zephyr SPI API on this peripheral. Buffers remain static in RAM. */
static int transfer(bool receive)
{
    if (transfer_failed) return -EIO;
    nrf_spim_tx_buffer_set(RAW_SPI, tx_buf, bufs[0].len);
    nrf_spim_rx_buffer_set(RAW_SPI, receive ? rx_buf : NULL,
                           receive ? bufs[1].len : 0U);
    nrf_spim_event_clear(RAW_SPI, NRF_SPIM_EVENT_END);
    nrf_gpio_pin_clear(RAW_CS_PIN);
    const uint32_t started = k_cycle_get_32();
    nrf_spim_task_trigger(RAW_SPI, NRF_SPIM_TASK_START);
    while (!nrf_spim_event_check(RAW_SPI, NRF_SPIM_EVENT_END)) {
        if ((uint32_t)(k_cycle_get_32() - started) > k_us_to_cyc_ceil32(2000U)) {
            // Disable DMA before static buffers can be reused after a fault.
            nrf_spim_disable(RAW_SPI);
            nrf_gpio_pin_set(RAW_CS_PIN);
            transfer_failed = true;
            return -ETIMEDOUT;
        }
    }
    nrf_gpio_pin_set(RAW_CS_PIN);
    return 0;
}

/*
 *****************************************************************************
 *
 *                              DeviceTree Information
 *
 *****************************************************************************
 */

#define DWM_SPI      DT_PARENT(DT_INST(0, qorvo_dwm3000))
#define DWM_CS_GPIO  DT_PHANDLE_BY_IDX(DT_PARENT(DT_INST(0, qorvo_dwm3000)), cs_gpios, 0)
#define DWM_CS_PIN   DT_PHA(DWM_SPI, cs_gpios, pin)
#define DWM_CS_FLAGS DT_PHA(DWM_SPI, cs_gpios, flags)

/*
 *****************************************************************************
 *
 *                              DW3000 SPI section
 *
 *****************************************************************************
 */

/*
 * Function: openspi()
 *
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 */
int openspi(void)
{
    transfer_failed = false;
    LOG_INF("%s bus %s", __func__, DT_LABEL(DWM_SPI));

    /* Propagate CS config into all spi_cfgs[] elements */
    cs_ctrl.gpio_dev = device_get_binding(DT_LABEL(DWM_CS_GPIO));
    if (!cs_ctrl.gpio_dev) {
        LOG_ERR("%s: GPIO binding failed.", __func__);
        return -1;
    }
    cs_ctrl.gpio_pin = DWM_CS_PIN;
    cs_ctrl.delay = 0U;
    cs_ctrl.gpio_dt_flags = DWM_CS_FLAGS;
    for (int i=0; i < SPI_CFGS_COUNT; i++) {
        spi_cfgs[i].cs = &cs_ctrl;
    }

    gpio_pin_set(cs_ctrl.gpio_dev, DWM_CS_PIN, 1);

    spi_cfg = &spi_cfgs[0];

    spi = device_get_binding(DT_LABEL(DWM_SPI));
    if (!spi) {
        LOG_ERR("%s: SPI binding failed.", __func__);
        return -1;
    }
    spi_cfg->operation = SPI_WORD_SET(8);
    spi_cfg->frequency = 2000000;

    memset(&tx_buf[0], 0, 255);
    memset(&rx_buf[0], 0, 255);
    bufs[0].buf = &tx_buf[0];
    bufs[1].buf = &rx_buf[0];
    tx.buffers = &bufs[0];
    rx.buffers = &bufs[1];
    tx.count = 1;
    rx.count = 1;

    irq_disable(DT_IRQN(RAW_SPI_NODE));
    nrf_spim_disable(RAW_SPI);
    RAW_SPI->INTENCLR = 0xFFFFFFFFUL;
    RAW_SPI->SHORTS = 0;
    nrf_gpio_pin_set(RAW_CS_PIN);
    nrf_gpio_cfg_output(RAW_CS_PIN);
    nrf_gpio_pin_clear(DT_PROP(RAW_SPI_NODE, sck_pin));
    nrf_gpio_cfg_output(DT_PROP(RAW_SPI_NODE, sck_pin));
    nrf_gpio_cfg_output(DT_PROP(RAW_SPI_NODE, mosi_pin));
    nrf_gpio_cfg_input(DT_PROP(RAW_SPI_NODE, miso_pin), NRF_GPIO_PIN_NOPULL);
    nrf_spim_pins_set(RAW_SPI, DT_PROP(RAW_SPI_NODE, sck_pin),
        DT_PROP(RAW_SPI_NODE, mosi_pin), DT_PROP(RAW_SPI_NODE, miso_pin));
    nrf_spim_configure(RAW_SPI, NRF_SPIM_MODE_0, NRF_SPIM_BIT_ORDER_MSB_FIRST);
    nrf_spim_frequency_set(RAW_SPI, NRF_SPIM_FREQ_2M);
    nrf_spim_enable(RAW_SPI);

    return 0;
}

void set_spi_speed_slow(void)
{
    spi_cfg = &spi_cfgs[0];
    spi_cfg->operation = SPI_WORD_SET(8);  // SPI mode(0,0)
    spi_cfg->frequency = 2000000;
    nrf_spim_frequency_set(RAW_SPI, NRF_SPIM_FREQ_2M);

    memset(&tx_buf[0], 0, 255);
    memset(&rx_buf[0], 0, 255);
}

void set_spi_speed_fast(void)
{
    spi_cfg = &spi_cfgs[1];
    spi_cfg->operation = SPI_WORD_SET(8);  // SPI mode(0,0)
    spi_cfg->frequency = 8000000;
    nrf_spim_frequency_set(RAW_SPI, NRF_SPIM_FREQ_8M);

    memset(&tx_buf[0], 0, 255);
    memset(&rx_buf[0], 0, 255);
}

/*
 * Function: closespi()
 *
 * Low level abstract function to close the the SPI device.
 * returns 0 for success, or -1 for error
 */
int closespi(void)
{
    //TODO
    return 0;
}

/*
 * Function: writetospiwithcrc()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success
 */
int writetospiwithcrc(uint16_t           headerLength,
                      const    uint8_t * headerBuffer,
                      uint16_t           bodyLength,
                      const    uint8_t * bodyBuffer,
                      uint8_t            crc8)
{
    uint16_t len =  headerLength + bodyLength + sizeof(crc8);

    if (len > sizeof(tx_buf))
        return -1;

    memcpy(&tx_buf[0],            headerBuffer, headerLength);
    memcpy(&tx_buf[headerLength], bodyBuffer,   bodyLength);

    tx_buf[headerLength + bodyLength] = crc8;

    bufs[0].len = len;
    bufs[1].len = len;

    /* TX-only also keeps one-byte DW fast commands clear of nRF52832 PAN 58. */
    return transfer(false);
}

/*
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success
 */
int writetospi(uint16_t           headerLength,
               const    uint8_t * headerBuffer,
               uint16_t           bodyLength,
               const    uint8_t * bodyBuffer)
{
    if ((uint32_t)headerLength + bodyLength > sizeof(tx_buf)) return -EINVAL;
#if 0
    LOG_HEXDUMP_INF(headerBuffer, headerLength, "writetospi: Header");
    LOG_HEXDUMP_INF(bodyBuffer, bodyLength, "writetospi: Body");
#endif

    memcpy(&tx_buf[0], headerBuffer, headerLength);
    memcpy(&tx_buf[headerLength], bodyBuffer, bodyLength);

    bufs[0].len = headerLength + bodyLength;
    bufs[1].len = headerLength + bodyLength;

    /* No receive buffer: RXD.MAXCNT is zero, including one-byte commands. */
    return transfer(false);
}

/*
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data
 * may be found, or returns 0
 */
int readfromspi(uint16_t        headerLength,
                const uint8_t * headerBuffer,
                uint16_t        readLength,
                uint8_t       * readBuffer)
{
    /* Every read includes an address/header and data; never DMA one RX byte. */
    if (headerLength == 0U || readLength == 0U ||
        (uint32_t)headerLength + readLength > sizeof(tx_buf)) return -EINVAL;
    memset(&tx_buf[0], 0, headerLength + readLength);
    memcpy(&tx_buf[0], headerBuffer, headerLength);

    bufs[0].len = headerLength + readLength;
    bufs[1].len = headerLength + readLength;

    int result = transfer(true);
    if (result != 0) {
        /* Vendor register accessors may ignore the return value. Never leave
         * prior stack/buffer contents looking like a fresh radio status. */
        memset(readBuffer, 0, readLength);
        return result;
    }

#if (CONFIG_SOC_NRF52840_QIAA)
    /*
     *  This is a hack to handle the corrupted response frame through the nRF52840's SPI3.
     *  See this project's issue-log (https://github.com/foldedtoad/dwm3000/issues/2)
     *  for details.
     *  The delay value is set in the CMakeList.txt file for this subproject.
     */
    for (volatile int i=0; i < TX_WAIT_RESP_NRF52840_DELAY; i++) { /* spin */ }
#endif

    memcpy(readBuffer, rx_buf + headerLength, readLength);

#if 0
    LOG_HEXDUMP_INF(headerBuffer, headerLength, "readfromspi: Header");
    LOG_HEXDUMP_INF(readBuffer, readLength, "readfromspi: Body");
#endif

    return 0;
}
