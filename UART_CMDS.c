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

#define UART_MAX_ARGS 10
#define UART_INVALID_CHARACTER 100

static const char *TAG = "USB_UART";
static uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];

// helper functions

static uint8_t uart_get_hex_from_char(char to_convert);

// internal functions

static void uart_tof_cmds(uint8_t argc, const char** argv)
{
    if(argc < 2)
    {
        ESP_LOGE(TAG, "incorrect number of args");
        return;
    }
    if(strcmp((char*) argv[1], (const char*) "load_config") == 0)
    {
        uint8_t config_type = 0;
        if(argc == 3)
        {
            config_type = (uint8_t) (argv[2][0] - "0");
        }
        TOF_LOAD_CONFIG(config_type);
    }
    else if(strcmp((char*) argv[1], (const char*) "reset_tof") == 0)
    {
        TOF_RESET();
    }
    else if(strcmp((char*) argv[1], (const char*) "read_i2c") == 0)
    {
        if(argc < 4)
        {
            ESP_LOGE(TAG, "Incorrect size args");
            return;
        }

        uint8_t read_reg = 0;
        if((uart_get_hex_from_char(argv[2][0]) == UART_INVALID_CHARACTER) || (uart_get_hex_from_char(argv[2][1]) == UART_INVALID_CHARACTER))
        {
            ESP_LOGE(TAG, "invalid read register");
            return;
        }
        read_reg = (uart_get_hex_from_char(argv[2][0]) * 16) + (uart_get_hex_from_char(argv[2][1]));

        uint8_t read_bytes = 0;
        for(uint8_t i = 0; i < strlen(argv[3]); i++)
        {
            if(argv[3][i] >= '0' && argv[3][i] <= '9')
            {
                read_bytes = read_bytes * 10;
                read_bytes += (uint8_t) (argv[3][i] - '0');
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
    else if(strcmp((char*) argv[1], (const char*) "write_i2c") == 0)
    {
        uint8_t write_cnt = 0;
        uint8_t write_bytes[UART_MAX_ARGS - 2] = {0};

        if(argc < 3)
        {
            ESP_LOGE(TAG, "Incorrect size args");
            return;
        }

        for(uint8_t i = 2; (i < argc) && (i < UART_MAX_ARGS); i++)
        {
            if(uart_get_hex_from_char(argv[i][0]) == UART_INVALID_CHARACTER)
            {
                ESP_LOGE(TAG, "invalid write at byte %u", );
                return;
            }

            write_bytes[write_cnt] = (uart_get_hex_from_char(argv[2][0]) * 16);

            if(uart_get_hex_from_char(argv[i][1]) == UART_INVALID_CHARACTER)
            {
                ESP_LOGE(TAG, "invalid write at byte %u", );
                return;
            }

            write_bytes[write_cnt] += uart_get_hex_from_char(argv[2][1])

            write_cnt++;
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

static void uart_flash_cmds(uint8_t argc, const char** argv)
{
    //Implement Flash Commands
}

static void uart_msg_queue_cmds(uint8_t argc, const char** argv)
{
    //Implement Message Queue Commands
}

static uint8_t uart_get_hex_from_char(char to_convert)
{
    if(to_convert >= '0' && to_convert <= '9')
    {
        return (uint8_t) (to_convert - '0');
    }
    else if(to_convert >= 'a' && to_convert <= 'f')
    {
        return (uint8_t) (to_convert - 'a' + 10);
    }
    else if(to_convert >= 'A' && to_convert <= 'F')
    {
        return (uint8_t) (to_convert - 'A' + 10);
    }
    else
    {
        return UART_INVALID_CHARACTER;
    }
}

static dispatcher_type_t uart_get_dispatcher(const char * disp_str)
{
    if(disp_str == NULL)
    {
        return error;
    }
    else if(strcmp((char*) cmd_buf, (const char*) "flash") == 0)
    {
        return flash;
    }
    else if(strcmp((char*) cmd_buf, (const char*) "msg_queue") == 0)
    {
        return msg_queue;
    }
    else if(strcmp((char*) cmd_buf, (const char*) "tof") == 0)
    {
        return tof;
    }
    else if(strcmp((char*) cmd_buf, (const char*) "imu") == 0)
    {
        return imu;
    }
    else if(strcmp((char*) cmd_buf, (const char*) "motor") == 0)
    {
        return motor;
    }
    else if(strcmp((char*) cmd_buf, (const char*) "led") == 0)
    {
        return led;
    }
    else if(strcmp((char*) cmd_buf, (const char*) "mesh") == 0)
    {
        return mesh;
    }
    else
    {
        return not_specified;
    }
}

static uint8_t uart_convert_str_to_args(const uint8_t * cmd_buf, char** argv_ptr, uint8_t argv_max)
{
    uint8_t argc = 0;

    char *p2 = strtok((char *)cmd_buf, " ");
    while (p2 && argc < argv_max-1)
    {
        argv_ptr[argc++] = p2;
        p2 = strtok(0, " ");
    }
    argv_ptr[argc] = 0;

    return argc;
}

static void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    /* initialization */
    size_t rx_size = 0;
    char *argv[UART_MAX_ARGS] = {0};
    uint8_t argc = 0;

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

    argc = uart_convert_str_to_args(buf, argv);

    if(argc == 0)
    {
        ESP_LOGE(TAG, "no dispatcher specified");
        return;
    }

    switch(uart_get_dispatcher(argv[0]))
    {
        case flash:
        {
            uart_flash_cmds(argc, argv);
            break;
        }
        case msg_queue:
        {
            uart_msg_queue_cmds(argc, argv)
            break;
        }
        case tof:
        {
            uart_tof_cmds(argc, argv);
            break;
        }
        case imu:
        {
            ESP_LOGI(TAG, "IMU commands not implemented");
            break;
        }
        case motor:
        {
            ESP_LOGI(TAG, "Motor commands not implemented");
            break;
        }
        case led:
        {
            ESP_LOGI(TAG, "LED commands not implemented");
            break;
        }
        case mesh:
        {
            ESP_LOGI(TAG, "Mesh commands not implemented");
            break;
        }
        default:
        {
            ESP_LOGI(TAG, "invalid specifier");
            break;
        }
    }
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
