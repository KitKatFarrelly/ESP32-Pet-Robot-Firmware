#include <stdint.h>

#ifdef FUNCTIONAL_TESTS
#include "mocked_functions.h"
#else
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "tusb_console.h"
#include "tusb_cdc_acm.h"
#include "sdkconfig.h"
#endif

#include "UART_CMDS.h"
#include "ToF_I2C.h"
#include "FLASH_SPI.h"
#include "MESSAGE_QUEUE.h"

#define UART_MAX_ARGS 10
#define UART_INVALID_CHARACTER 100

static const char *TAG = "USB_UART";
static uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];

// static variables
static component_handle_t s_uart_component_handle = 0;
static bool s_has_component_handle = false;
static callback_handle_t UART_callback_handles[dispatcher_max] = {0};

// helper functions

static uint8_t uart_get_hex_from_char(char to_convert);
static dispatcher_type_t uart_get_dispatcher(char * disp_str);
static uint8_t uart_convert_str_to_args(const uint8_t * cmd_buf, char** argv_ptr, uint8_t argv_max);

// usb functions

static void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event);
static void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event);

// message queue functions

static void uart_msg_queue_handler(component_handle_t component_type, uint8_t message_type, void* message_data);
static char* uart_return_string_from_dispatcher(dispatcher_type_t dispatcher);
static dispatcher_type_t uart_get_dispatcher_from_component(component_handle_t component);
static component_handle_t uart_get_component_handle_from_dispatcher(dispatcher_type_t dispatcher);
static bool uart_does_component_have_a_handle(dispatcher_type_t dispatcher);

// uart command lists

static void uart_tof_cmds(uint8_t argc, char** argv);
static void uart_flash_cmds(uint8_t argc, char** argv);
static void uart_msg_queue_cmds(uint8_t argc, char** argv);

// function defs

static void uart_tof_cmds(uint8_t argc, char** argv)
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
            config_type = (uint8_t) (argv[2][0] - '0');
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
        for(uint8_t i = 0; i < read_bytes; i++)
        {
            ESP_LOGI(TAG, "%x", read_data[i]);
        }
        free(read_data);
    }
    else if(strcmp((char*) argv[1], (const char*) "write_i2c") == 0)
    {
        uint8_t write_cnt = 0;
        uint8_t write_bytes[UART_MAX_ARGS - 2] = {0};
        uint8_t i = 0;
        if(argc < 3)
        {
            ESP_LOGE(TAG, "Incorrect size args");
            return;
        }

        for(i = 2; (i < argc) && (i < UART_MAX_ARGS); i++)
        {
            if(uart_get_hex_from_char(argv[i][0]) == UART_INVALID_CHARACTER)
            {
                ESP_LOGE(TAG, "invalid write at byte %u", i);
                return;
            }

            write_bytes[write_cnt] = (uart_get_hex_from_char(argv[2][0]) * 16);

            if(uart_get_hex_from_char(argv[i][1]) == UART_INVALID_CHARACTER)
            {
                ESP_LOGE(TAG, "invalid write at byte %u", i);
                return;
            }

            write_bytes[write_cnt] += uart_get_hex_from_char(argv[2][1]);

            write_cnt++;
        }
        if(i < UART_MAX_ARGS)
        {
            write_bytes[write_cnt] = 0;
        }
        ESP_LOGI(TAG, "Writing %d bytes to register 0x%x: ", write_cnt, write_bytes[0]);
        for(i = 0; i < write_cnt; i++)
        {
            ESP_LOGI(TAG, "%x", write_bytes[i]);
        }
        TOF_WRITE(write_bytes, write_cnt);
    }
}

static void uart_flash_cmds(uint8_t argc, char** argv)
{
    if(argc < 2)
    {
        ESP_LOGE(TAG, "incorrect number of args");
        return;
    }
    if(strcmp((char*) argv[1], (const char*) "write_flash") == 0)
    {
        uint8_t write_cnt = 0;
        uint8_t write_bytes[UART_MAX_ARGS - 3] = {0};
        uint8_t i = 0;
        if(argc < 4)
        {
            ESP_LOGE(TAG, "Incorrect size args");
            return;
        }
        for(i = 3; (i < argc) && (i < UART_MAX_ARGS); i++)
        {
            if(uart_get_hex_from_char(argv[i][0]) == UART_INVALID_CHARACTER)
            {
                ESP_LOGE(TAG, "invalid write at byte %u", i);
                return;
            }

            write_bytes[write_cnt] = (uart_get_hex_from_char(argv[2][0]) * 16);

            if(uart_get_hex_from_char(argv[i][1]) == UART_INVALID_CHARACTER)
            {
                ESP_LOGE(TAG, "invalid write at byte %u", i);
                return;
            }

            write_bytes[write_cnt] += uart_get_hex_from_char(argv[2][1]);

            write_cnt++;
        }
        if(i < UART_MAX_ARGS)
        {
            write_bytes[write_cnt] = 0;
        }
        ESP_LOGI(TAG, "Writing %d bytes to key %s: ", write_cnt, argv[2]);
        FLASH_WRITE_TO_BLOB("factory", "flash", argv[2], write_bytes, write_cnt);
    }
    else if(strcmp((char*) argv[1], (const char*) "read_flash") == 0)
    {
        if(argc < 4)
        {
            ESP_LOGE(TAG, "Incorrect size args");
            return;
        }

        uint8_t read_bytes = 0;
        for(uint8_t i = 0; i < strlen(argv[3]); i++)
        {
            if(argv[3][i] >= '0' && argv[3][i] <= '9')
            {
                read_bytes = read_bytes * 10;
                read_bytes += (uint8_t) (argv[3][i] - '0');
            }
        }
        ESP_LOGI(TAG, "Reading %d bytes from key %s", read_bytes, argv[2]);
        uint8_t* read_data = FLASH_READ_FROM_BLOB("factory", "flash", argv[2], read_bytes);
        ESP_LOGI(TAG, "Read the following bytes: ");
        for(uint8_t i = 0; i < read_bytes; i++)
        {
            ESP_LOGI(TAG, "%x", read_data[i]);
        }
        free(read_data);
    }
    else if(strcmp((char*) argv[1], (const char*) "clear_partition") == 0)
    {
        uint8_t error_code = 0;
        if(argc < 3)
        {
            error_code = FLASH_ERASE_PARTITION(MAIN_PARTITION);
        }
        else
        {
            error_code = FLASH_ERASE_PARTITION(argv[2]);
        }
        ESP_LOGI(TAG, "cleared partition with error %u", error_code);
    }
    else if(strcmp((char*) argv[1], (const char*) "partition_info") == 0)
    {
        PARTITION_INFO_t partition_info;
        if(argc < 3)
        {
            partition_info = FLASH_GET_PARTITION_INFO(MAIN_PARTITION);
            ESP_LOGI(TAG, "info for default partition:");
        }
        else
        {
            partition_info = FLASH_GET_PARTITION_INFO(argv[2]);
            ESP_LOGI(TAG, "info for %s partition:", argv[2]);
        }
        if(partition_info.number_of_entries == 0)
        {
            ESP_LOGE(TAG, "failed to retrieve info!");
            return;
        }
        ESP_LOGI(TAG, "number of entries %zu", partition_info.number_of_entries);
        ESP_LOGI(TAG, "in use entries entries %zu", partition_info.in_use_entries);
        ESP_LOGI(TAG, "free entries %zu", partition_info.free_entries);
    }
    else if(strcmp((char*) argv[1], (const char*) "find_key") == 0)
    {
        size_t key_size = 0;
        if(argc < 3)
        {
            ESP_LOGE(TAG, "Incorrect size args");
            return;
        }
        if(argc < 4)
        {
            key_size = FLASH_DOES_KEY_EXIST(MAIN_PARTITION, "flash", argv[2]);
            ESP_LOGI(TAG, "info for %s key in default namespace in default partition:", argv[2]);
        }
        else if(argc < 5)
        {
            key_size = FLASH_DOES_KEY_EXIST(argv[2], "flash", argv[3]);
            ESP_LOGI(TAG, "info for %s key in default namespace in %s partition:", argv[3], argv[2]);
        }
        else
        {
            key_size = FLASH_DOES_KEY_EXIST(argv[2], argv[3], argv[4]);
            ESP_LOGI(TAG, "info for %s key in %s namespace in %s partition:", argv[4], argv[3], argv[2]);
        }
        if(key_size == 0)
        {
            ESP_LOGE(TAG, "failed to retrieve key data!");
            return;
        }
        ESP_LOGI(TAG, "key exists with size %zu", key_size);
    }
}

static void uart_msg_queue_cmds(uint8_t argc, char** argv)
{
    if(argc < 2)
    {
        ESP_LOGE(TAG, "incorrect number of args");
        return;
    }
    if(strcmp((char*) argv[1], (const char*) "msg_create_handle") == 0)
    {
        if(!s_has_component_handle)
        {
            uint8_t error = 0;
            if((error = create_handle_for_component(&s_uart_component_handle)) > 0)
            {
                ESP_LOGE(TAG, "failed to register UART to message queue with error %u", error);
            }
            else
            {
                s_has_component_handle = true;
                ESP_LOGI(TAG, "new handle for UART: %u", s_uart_component_handle);
            }
        }
        else
        {
            ESP_LOGI(TAG, "handle for UART already exists: %u", s_uart_component_handle);
        }
    }
    else if(strcmp((char*) argv[1], (const char*) "msg_delete_handle") == 0)
    {
        if(s_has_component_handle)
        {
            uint8_t error = 0;
            if((error = create_handle_for_component(&s_uart_component_handle)) > 0)
            {
                ESP_LOGE(TAG, "failed to delete UART from message queue with error %u", error);
            }
            else
            {
                s_has_component_handle = false;
                ESP_LOGI(TAG, "successfully deleted UART from message queue.");
            }
        }
        else
        {
            ESP_LOGI(TAG, "handle for UART does not exist.");
        }
    }
    else if(strcmp((char*) argv[1], (const char*) "msg_register_cb") == 0)
    {
        if(argc < 3)
        {
            ESP_LOGE(TAG, "Incorrect size args");
            return;
        }
        dispatcher_type_t callback_index = uart_get_dispatcher(argv[2]);
        if(!uart_does_component_have_a_handle(callback_index))
        {
            ESP_LOGE(TAG, "component handle for %s does not exist.", uart_return_string_from_dispatcher(callback_index));
            return;
        }
        component_handle_t component = uart_get_component_handle_from_dispatcher(callback_index);
        ESP_LOGI(TAG, "registering UART handler for %s messages:", uart_return_string_from_dispatcher(callback_index));
        UART_callback_handles[callback_index] = register_component_handler_for_messages(uart_msg_queue_handler, component);
        if(UART_callback_handles[callback_index]) //returning 0 means no callback handle was generated.
        {
            ESP_LOGI(TAG, "registration successful!");
        }
        else
        {
            ESP_LOGE(TAG, "registration failed!");
        }
    }
    else if(strcmp((char*) argv[1], (const char*) "msg_unregister_cb") == 0)
    {
        if(argc < 3)
        {
            ESP_LOGE(TAG, "Incorrect size args");
            return;
        }
        dispatcher_type_t callback_index = uart_get_dispatcher(argv[2]);
        if(!uart_does_component_have_a_handle(callback_index))
        {
            ESP_LOGE(TAG, "failed to unregister: component handle does not exist.");
            return;
        }
        component_handle_t component = uart_get_component_handle_from_dispatcher(callback_index);
        if(UART_callback_handles[callback_index] == 0)
        {
            ESP_LOGE(TAG, "failed to unregister: callback handle does not exist.");
            return;
        }
        ESP_LOGI(TAG, "unregistering UART handler from %s messages:", uart_return_string_from_dispatcher(callback_index));
        if(unregister_component_handler_for_messages(component, UART_callback_handles[callback_index]))
        {
            ESP_LOGE(TAG, "unregistration failed!");
        }
        else
        {
            ESP_LOGI(TAG, "unregistration successful!");
            UART_callback_handles[callback_index] = 0;
        }
    }
    else if(strcmp((char*) argv[1], (const char*) "msg_send_message") == 0)
    {
        if(s_has_component_handle)
        {
            if(argc < 4)
            {
                ESP_LOGE(TAG, "Incorrect size args");
                return;
            }
            message_info_t message;
            message.component_handle = s_uart_component_handle;
            message.message_type = 0;
            message.message_data = malloc(sizeof(char*) * (strlen(argv[3]) + 1));
            memcpy(message.message_data, argv[3], sizeof(char*) * (strlen(argv[3]) + 1));
            if(strcmp((char*) argv[2], (const char*) "priority") == 0)
            {
                if(check_is_queue_active(1))
                {
                    send_message_to_priority_queue(message);
                    ESP_LOGI(TAG, "sent priority queue message with payload %s.", argv[3]);
                }
                else
                {
                    ESP_LOGE(TAG, "priority queue is inactive.");
                }
            }
            else if(strcmp((char*) argv[2], (const char*) "normal") == 0)
            {
                if(check_is_queue_active(0))
                {
                    send_message_to_normal_queue(message);
                    ESP_LOGI(TAG, "sent normal queue message with payload %s.", argv[3]);
                }
                else
                {
                    ESP_LOGE(TAG, "normal queue is inactive.");
                }
            }
            else
            {
                ESP_LOGE(TAG, "send failed, must set priority");
            }
        }
        else
        {
            ESP_LOGE(TAG, "UART has no component handle!!!");
        }
    }
    else if(strcmp((char*) argv[1], (const char*) "msg_clear_handles") == 0)
    {
        if(clear_all_handles())
        {
            ESP_LOGI(TAG, "failed to delete all component handles.");
        }
        else
        {
            ESP_LOGI(TAG, "successfully deleted all queue component handles.");
        }
    }
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

static dispatcher_type_t uart_get_dispatcher(char * disp_str)
{
    if(disp_str == NULL)
    {
        return error;
    }
    else if(strcmp(disp_str, (const char*) "flash") == 0)
    {
        return flash;
    }
    else if(strcmp(disp_str, (const char*) "msg_queue") == 0)
    {
        return msg_queue;
    }
    else if(strcmp(disp_str, (const char*) "tof") == 0)
    {
        return tof;
    }
    else if(strcmp(disp_str, (const char*) "imu") == 0)
    {
        return imu;
    }
    else if(strcmp(disp_str, (const char*) "motor") == 0)
    {
        return motor;
    }
    else if(strcmp(disp_str, (const char*) "led") == 0)
    {
        return led;
    }
    else if(strcmp(disp_str, (const char*) "mesh") == 0)
    {
        return mesh;
    }
    else if(strcmp(disp_str, (const char*) "uart") == 0)
    {
        return uart;
    }
    else
    {
        return not_specified;
    }
}

// Credit to sstteevvee on StackOverflow for this one
// https://stackoverflow.com/questions/1706551/parse-string-into-argv-argc
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

static void uart_msg_queue_handler(component_handle_t component_type, uint8_t message_type, void* message_data)
{
    dispatcher_type_t dispatcher = uart_get_dispatcher_from_component(component_type);
    ESP_LOGI(TAG, "message from %s with message type %u.", uart_return_string_from_dispatcher(dispatcher), message_type);
    ESP_LOGI(TAG, "message data:");
    uint8_t* output_data = (uint8_t*) message_data;
    for(int i = 0; i < sizeof(message_data); i++)
    {
        ESP_LOGI(TAG, "%x", output_data[i]);
    }

}

static char* uart_return_string_from_dispatcher(dispatcher_type_t dispatcher)
{
    switch(dispatcher)
    {
        case not_specified: return "not specified";
        case flash: return "flash";
        case msg_queue: return "msg queue";
        case tof: return "tof";
        case imu: return "imu";
        case motor: return "motor";
        case led: return "led";
        case mesh: return "mesh";
        case uart: return "uart";
        case error: return "error";
        default: return "unknown component";
    }
}

static dispatcher_type_t uart_get_dispatcher_from_component(component_handle_t component)
{
    for(dispatcher_type_t i = 0; i < dispatcher_max; i++)
    {
        if(UART_callback_handles[i] == component)
        {
            return i;
        }
    }
    return error;
}

static component_handle_t uart_get_component_handle_from_dispatcher(dispatcher_type_t dispatcher)
{
    switch(dispatcher)
    {
        case uart: return s_uart_component_handle;
        default: return 0;
    }
}
static bool uart_does_component_have_a_handle(dispatcher_type_t dispatcher)
{
    switch(dispatcher)
    {
        case uart: return s_has_component_handle;
        default: return false;
    }
}

#ifndef FUNCTIONAL_TESTS

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

    argc = uart_convert_str_to_args(buf, argv, UART_MAX_ARGS);

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
            uart_msg_queue_cmds(argc, argv);
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
        case uart:
        {
            ESP_LOGI(TAG, "UART commands not implemented");
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

#endif

void UART_INIT(void)
{
    ESP_LOGI(TAG, "USB initialization");

#ifndef FUNCTIONAL_TESTS

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

#endif

    memset(UART_callback_handles, 0, sizeof(UART_callback_handles));

    ESP_LOGI(TAG, "USB initialization DONE");
}
