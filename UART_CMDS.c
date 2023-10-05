#include <stdint.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "tusb_console.h"
#include "tusb_cdc_acm.h"
#include "sdkconfig.h"

#include "UART_CMDS.h"
#include "ToF_I2C.h"

static const char *TAG = "USB_UART";
static uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];

static void UART_RUN_CMD(const uint8_t * cmd_buf, size_t cmd_size)
{
    uint8_t i = 0;
    if(strncmp((char*) cmd_buf, (const char*) "l", 1) == 0)
    {
        TOF_LOAD_CONFIG(0);
    }
    else if(strncmp((char*) cmd_buf, (const char*) "o", 1) == 0)
    {
        TOF_RESET();
    }
    else if(strncmp((char*) cmd_buf, (const char*) "r", 1) == 0)
    {
        if(cmd_size < 6)
        {
            ESP_LOGE(TAG, "Incorrect size args");
            return;
        }
        uint8_t read_reg = ((cmd_buf[2] - '0') * 16) + (cmd_buf[3] - '0');

        uint8_t read_bytes = 0;
        for(i = 5; i < cmd_size; i++)
        {
            if(cmd_buf[i] >= '0' && cmd_buf[i] <= '9')
            {
                read_bytes = read_bytes * 10;
                read_bytes += (cmd_buf[i] - '0');
            }
        }
        uint8_t* read_data = malloc(read_bytes * sizeof(uint8_t));
        ESP_LOGI(TAG, "Reading %d bytes from register 0x%x", read_bytes, read_reg);
        TOF_READ_WRITE(read_data, read_bytes, &read_reg, 1);
        ESP_LOGI(TAG, "Read the following bytes: ");
        for(i = 0; i < read_bytes; i++)
        {
            ESP_LOGI(TAG, "%x", read_data[i]);
        }
    }
    else if(strncmp((char*) cmd_buf, (const char*) "w", 1) == 0)
    {
        uint8_t write_cnt = 0;
        uint8_t write_bytes[20] = {0};
        bool is_lsb = false;
        for(i = 1; i < cmd_size; i++)
        {
            if(cmd_buf[i] >= '0' && cmd_buf[i] <= '9')
            {
                if(is_lsb)
                {
                    write_bytes[write_cnt] += ((cmd_buf[i] - '0'));
                    is_lsb = false;
                    write_cnt++;
                }
                else
                {
                    write_bytes[write_cnt] = ((cmd_buf[i] - '0') * 16);
                    is_lsb = true;
                }
            }
        }
        write_bytes[write_cnt] = 0;
        ESP_LOGI(TAG, "Writing %d bytes to register 0x%x: ", write_cnt, write_bytes[0]);
        for(i = 0; i < write_cnt; i++)
        {
            ESP_LOGI(TAG, "%x", write_bytes[i]);
        }
        TOF_WRITE(write_bytes, write_cnt);
    }
}

static void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    /* initialization */
    size_t rx_size = 0;

    /* read */
    esp_err_t ret = tinyusb_cdcacm_read(itf, buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
    if (ret == ESP_OK) {
        buf[rx_size] = '\0';
        ESP_LOGI(TAG, "Got data (%d bytes): %s", rx_size, buf);
    } else {
        ESP_LOGE(TAG, "Read error");
    }

    // write back
    tinyusb_cdcacm_write_queue(itf, buf, rx_size);
    tinyusb_cdcacm_write_flush(itf, 0);

    UART_RUN_CMD(buf, rx_size);
}

static void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    int rst = event->line_state_changed_data.rts;
    ESP_LOGI(TAG, "Line state changed! dtr:%d, rst:%d", dtr, rst);
}

void UART_INIT(void)
{
    ESP_LOGI(TAG, "USB initialization");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
        .configuration_descriptor = NULL,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,
        .callback_rx = &tinyusb_cdc_rx_callback, // the first way to register a callback
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };

    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
    /* the second way to register a callback */
    ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
                        TINYUSB_CDC_ACM_0,
                        CDC_EVENT_LINE_STATE_CHANGED,
                        &tinyusb_cdc_line_state_changed_callback));

    ESP_ERROR_CHECK(esp_tusb_init_console(0));

    ESP_LOGI(TAG, "USB initialization DONE");
}
