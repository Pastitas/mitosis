#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "nrf_drv_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "nrf_gzll.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"

const nrf_drv_rtc_t rtc_timestamp = NRF_DRV_RTC_INSTANCE(0);

#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1                           /**< UART RX buffer size. */

#define RX_PIN_NUMBER  25
#define TX_PIN_NUMBER  24
#define CTS_PIN_NUMBER 23
#define RTS_PIN_NUMBER 22
#define HWFC           false

#define TX_PAYLOAD_LENGTH 3 ///< 3 byte payload length

#define LEFT_PIPE 0
#define RIGHT_PIPE 1

static uint8_t payload[2][NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH];
static uint8_t data_buffer[10];

static bool packet_received_left, packet_received_right;

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

uint32_t timestamp = 0;
uint32_t left_timestamp = 0;
uint32_t right_timestamp = 0;

bool dirty = false;

void receive_left()
{
    dirty = true;
    left_timestamp = timestamp;

    data_buffer[0] = ((payload[LEFT_PIPE][0] & 1<<3) ? 1:0) << 0 |
                     ((payload[LEFT_PIPE][0] & 1<<4) ? 1:0) << 1 |
                     ((payload[LEFT_PIPE][0] & 1<<5) ? 1:0) << 2 |
                     ((payload[LEFT_PIPE][0] & 1<<6) ? 1:0) << 3 |
                     ((payload[LEFT_PIPE][0] & 1<<7) ? 1:0) << 4;

    data_buffer[2] = ((payload[LEFT_PIPE][1] & 1<<6) ? 1:0) << 0 |
                     ((payload[LEFT_PIPE][1] & 1<<7) ? 1:0) << 1 |
                     ((payload[LEFT_PIPE][0] & 1<<0) ? 1:0) << 2 |
                     ((payload[LEFT_PIPE][0] & 1<<1) ? 1:0) << 3 |
                     ((payload[LEFT_PIPE][0] & 1<<2) ? 1:0) << 4;

    data_buffer[4] = ((payload[LEFT_PIPE][1] & 1<<1) ? 1:0) << 0 |
                     ((payload[LEFT_PIPE][1] & 1<<2) ? 1:0) << 1 |
                     ((payload[LEFT_PIPE][1] & 1<<3) ? 1:0) << 2 |
                     ((payload[LEFT_PIPE][1] & 1<<4) ? 1:0) << 3 |
                     ((payload[LEFT_PIPE][1] & 1<<5) ? 1:0) << 4;

    data_buffer[6] = ((payload[LEFT_PIPE][2] & 1<<5) ? 1:0) << 1 |
                     ((payload[LEFT_PIPE][2] & 1<<6) ? 1:0) << 2 |
                     ((payload[LEFT_PIPE][2] & 1<<7) ? 1:0) << 3 |
                     ((payload[LEFT_PIPE][1] & 1<<0) ? 1:0) << 4;

    data_buffer[8] = ((payload[LEFT_PIPE][2] & 1<<1) ? 1:0) << 1 |
                     ((payload[LEFT_PIPE][2] & 1<<2) ? 1:0) << 2 |
                     ((payload[LEFT_PIPE][2] & 1<<3) ? 1:0) << 3 |
                     ((payload[LEFT_PIPE][2] & 1<<4) ? 1:0) << 4;
}

void receive_right()
{
    dirty = true;
    right_timestamp = timestamp;

    data_buffer[1] = ((payload[RIGHT_PIPE][0] & 1<<7) ? 1:0) << 0 |
                     ((payload[RIGHT_PIPE][0] & 1<<6) ? 1:0) << 1 |
                     ((payload[RIGHT_PIPE][0] & 1<<5) ? 1:0) << 2 |
                     ((payload[RIGHT_PIPE][0] & 1<<4) ? 1:0) << 3 |
                     ((payload[RIGHT_PIPE][0] & 1<<3) ? 1:0) << 4;

    data_buffer[3] = ((payload[RIGHT_PIPE][0] & 1<<2) ? 1:0) << 0 |
                     ((payload[RIGHT_PIPE][0] & 1<<1) ? 1:0) << 1 |
                     ((payload[RIGHT_PIPE][0] & 1<<0) ? 1:0) << 2 |
                     ((payload[RIGHT_PIPE][1] & 1<<7) ? 1:0) << 3 |
                     ((payload[RIGHT_PIPE][1] & 1<<6) ? 1:0) << 4;

    data_buffer[5] = ((payload[RIGHT_PIPE][1] & 1<<5) ? 1:0) << 0 |
                     ((payload[RIGHT_PIPE][1] & 1<<4) ? 1:0) << 1 |
                     ((payload[RIGHT_PIPE][1] & 1<<3) ? 1:0) << 2 |
                     ((payload[RIGHT_PIPE][1] & 1<<2) ? 1:0) << 3 |
                     ((payload[RIGHT_PIPE][1] & 1<<1) ? 1:0) << 4;

    data_buffer[7] = ((payload[RIGHT_PIPE][1] & 1<<0) ? 1:0) << 0 |
                     ((payload[RIGHT_PIPE][2] & 1<<7) ? 1:0) << 1 |
                     ((payload[RIGHT_PIPE][2] & 1<<6) ? 1:0) << 2 |
                     ((payload[RIGHT_PIPE][2] & 1<<5) ? 1:0) << 3;

    data_buffer[9] = ((payload[RIGHT_PIPE][2] & 1<<4) ? 1:0) << 0 |
                     ((payload[RIGHT_PIPE][2] & 1<<3) ? 1:0) << 1 |
                     ((payload[RIGHT_PIPE][2] & 1<<2) ? 1:0) << 2 |
                     ((payload[RIGHT_PIPE][2] & 1<<1) ? 1:0) << 3;
}

void update_QMK()
{
    uint8_t c;
    //if (app_uart_get(&c) == NRF_SUCCESS && c == 's' && dirty)
    if (dirty)
    {
        dirty = false;
        // sending data to QMK, and an end byte
        nrf_drv_uart_tx(data_buffer, 10);
        app_uart_put(0xE0);
        // allowing UART buffers to clear
        nrf_delay_us(10);
    }
}

// 8 Hz event
static void handler_timestamp(nrf_drv_rtc_int_type_t int_type)
{
    static uint32_t ticks = 0;
    timestamp = ++ticks / 8;

    // if no packets recieved from keyboards in a few seconds, assume either
    // out of range, or sleeping due to no keys pressed, update keystates to off

    if ((timestamp - left_timestamp) > 12)
    {
        data_buffer[0] =
        data_buffer[2] =
        data_buffer[4] =
        data_buffer[6] =
        data_buffer[8] = 0;
    }

    if ((timestamp - right_timestamp) > 12)
    {
        data_buffer[1] =
        data_buffer[3] =
        data_buffer[5] =
        data_buffer[7] =
        data_buffer[9] = 0;
    }
}

int main(void)
{
    uint32_t err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud1M
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_error_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_init();
    nrf_drv_clock_lfclk_request(NULL);

    nrf_drv_rtc_init(&rtc_timestamp, NULL, handler_timestamp);
    nrf_drv_rtc_tick_enable(&rtc_timestamp, true);
    nrf_drv_rtc_enable(&rtc_timestamp);

    nrf_gzll_init(NRF_GZLL_MODE_HOST);
    nrf_gzll_set_tx_power(NRF_GZLL_TX_POWER_4_DBM);
    nrf_gzll_set_base_address_0(0x01020304);
    nrf_gzll_set_base_address_1(0x05060708);
    nrf_gzll_enable();

    while (true)
    {
//        update_QMK();

        // __WFE();
        // __SEV();
        // __WFE();
    }
}

void nrf_gzll_host_rx_data_ready(uint32_t pipe, nrf_gzll_host_rx_info_t rx_info)
{   
    uint32_t data_payload_length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;
    
    if (nrf_gzll_fetch_packet_from_rx_fifo(pipe, payload[pipe], &data_payload_length))
    {
        switch (pipe)
        {
            case LEFT_PIPE:
                receive_left();
                break;
            case RIGHT_PIPE:
                receive_right();
                break;
        }

        update_QMK();
    }
}

void nrf_gzll_device_tx_success(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info) {}
void nrf_gzll_device_tx_failed(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info) {}
void nrf_gzll_disabled() {}