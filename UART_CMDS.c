#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>

#ifdef FUNCTIONAL_TESTS
#include "mocked_functions.h"
#else
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#endif

#include "UART_CMDS.h"
#include "ToF_I2C.h"
#include "FLASH_SPI.h"
#include "IMU_SPI.h"
#include "MESSAGE_QUEUE.h"
#include "MTR_DRVR.h"

#define UART_MAX_ARGS 10
#define UART_INVALID_CHARACTER 100
#define UART_SERIAL_MAX 200

static const char *TAG = "USB_UART";

// static variables
static component_handle_t s_uart_component_handle = 0;
static bool s_has_component_handle = false;
static bool s_serialize = false;
static callback_handle_t UART_callback_handles[dispatcher_max] = {0};
static callback_handle_t s_ToF_callback_handle;
static callback_handle_t s_imu_callback_handle;
static uint8_t s_serial_out[UART_SERIAL_MAX] = {0};

// helper functions

static uint8_t uart_get_hex_from_char(char to_convert);
static dispatcher_type_t uart_get_dispatcher(char * disp_str);
static uint8_t uart_convert_str_to_args(char * cmd_buf, char** argv_ptr, uint8_t argv_max);
static uint8_t uart_convert_str_to_handedness(char * cmd_buf);
static mtr_direction_t uart_convert_str_to_direction(char * cmd_buf);
static void run_command(uint8_t rx_size, char *buf);

// message queue functions

static void uart_msg_queue_handler(component_handle_t component_type, uint8_t message_type, void* message_data, size_t message_size);
static char* uart_return_string_from_dispatcher(dispatcher_type_t dispatcher);
static dispatcher_type_t uart_get_dispatcher_from_component(component_handle_t component);
static component_handle_t uart_get_component_handle_from_dispatcher(dispatcher_type_t dispatcher);
static bool uart_does_component_have_a_handle(dispatcher_type_t dispatcher);

// uart command lists

static void uart_tof_cmds(uint8_t argc, char** argv);
static void uart_flash_cmds(uint8_t argc, char** argv);
static void uart_msg_queue_cmds(uint8_t argc, char** argv);
static void uart_imu_cmds(uint8_t argc, char** argv);
static void uart_mtr_cmds(uint8_t argc, char** argv);
static void uart_serial_cmds(uint8_t argc, char** argv);

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

            write_bytes[write_cnt] = (uart_get_hex_from_char(argv[i][0]) * 16);

            if(uart_get_hex_from_char(argv[i][1]) == UART_INVALID_CHARACTER)
            {
                ESP_LOGE(TAG, "invalid write at byte %u", i);
                return;
            }

            write_bytes[write_cnt] += uart_get_hex_from_char(argv[i][1]);

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
    else if(strcmp((char*) argv[1], (const char*) "factory_calibrate") == 0)
    {
        //factory calibration
        uint8_t err = TOF_FACTORY_CALIBRATION();
        ESP_LOGI(TAG, "Error code is: %u", err);

        err = TOF_RETURN_CALIBRATION_STATUS();
        ESP_LOGI(TAG, "calibration status is is: %x", err);
    }
    else if(strcmp((char*) argv[1], (const char*) "store_calibration") == 0)
    {
        //store calibration
        uint8_t err = TOF_STORE_FACTORY_CALIBRATION();
        ESP_LOGI(TAG, "Error code is: %u", err);
    }
    else if(strcmp((char*) argv[1], (const char*) "load_calibration") == 0)
    {
        //load calibration
        uint8_t err = TOF_LOAD_FACTORY_CALIBRATION();
        ESP_LOGI(TAG, "Error code is: %u", err);
    }
    else if(strcmp((char*) argv[1], (const char*) "read_cal_flash") == 0)
    {
        //read calibration from flash
        if(argc < 3)
        {
            ESP_LOGE(TAG, "Incorrect size args");
            return;
        }

        uint8_t blob_val = (argv[2][0] - '0');

        char fac_cal_blob_name[20] = {0};

        size_t fac_cal_strlen = 0;

        switch(blob_val)
        {
            case 0:
            {
                fac_cal_strlen = strlen(tmf8828_fac_cal_1);
				memcpy(fac_cal_blob_name, tmf8828_fac_cal_1, fac_cal_strlen);
				fac_cal_blob_name[fac_cal_strlen] = '_';
				fac_cal_blob_name[fac_cal_strlen + 1] = ('0');
				fac_cal_blob_name[fac_cal_strlen + 2] = '\0';
                break;
            }
            case 1:
            {
                fac_cal_strlen = strlen(tmf8828_fac_cal_1);
				memcpy(fac_cal_blob_name, tmf8828_fac_cal_1, fac_cal_strlen);
				fac_cal_blob_name[fac_cal_strlen] = '_';
				fac_cal_blob_name[fac_cal_strlen + 1] = ('0');
				fac_cal_blob_name[fac_cal_strlen + 2] = '\0';
                break;
            }
            case 2:
            {
                fac_cal_strlen = strlen(tmf8828_fac_cal_1);
				memcpy(fac_cal_blob_name, tmf8828_fac_cal_1, fac_cal_strlen);
				fac_cal_blob_name[fac_cal_strlen] = '_';
				fac_cal_blob_name[fac_cal_strlen + 1] = ('0');
				fac_cal_blob_name[fac_cal_strlen + 2] = '\0';
                break;
            }
            case 3:
            {
                fac_cal_strlen = strlen(tmf8828_fac_cal_1);
				memcpy(fac_cal_blob_name, tmf8828_fac_cal_1, fac_cal_strlen);
				fac_cal_blob_name[fac_cal_strlen] = '_';
				fac_cal_blob_name[fac_cal_strlen + 1] = ('0');
				fac_cal_blob_name[fac_cal_strlen + 2] = '\0';
                break;
            }
            case 4:
            {
                fac_cal_strlen = strlen(tmf8828_fac_cal_1);
				memcpy(fac_cal_blob_name, tmf8828_fac_cal_1, fac_cal_strlen);
				fac_cal_blob_name[fac_cal_strlen] = '_';
				fac_cal_blob_name[fac_cal_strlen + 1] = ('0');
				fac_cal_blob_name[fac_cal_strlen + 2] = '\0';
                break;
            }
            default:
            {
                ESP_LOGE(TAG, "blob val must be between 0 and 4");
                return;
            }
        }

        if(FLASH_DOES_KEY_EXIST(MAIN_PARTITION, "tof", fac_cal_blob_name) != 0xC0)
        {
            ESP_LOGE(TAG, "selected calibration blob is not saved");
            return;
        }
        uint8_t* read_data = FLASH_READ_FROM_BLOB(MAIN_PARTITION, "tof", fac_cal_blob_name, 0xC0);
        ESP_LOGI(TAG, "Read the following bytes: ");
        for(uint8_t i = 0; i < 0x0C; i++)
        {
            ESP_LOGI(TAG, "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x", 
                    read_data[(i * 16) +  0], read_data[(i * 16) +  1], read_data[(i * 16) +  2], read_data[(i * 16) +  3],
                    read_data[(i * 16) +  4], read_data[(i * 16) +  5], read_data[(i * 16) +  6], read_data[(i * 16) +  7],
                    read_data[(i * 16) +  8], read_data[(i * 16) +  9], read_data[(i * 16) + 10], read_data[(i * 16) + 11],
                    read_data[(i * 16) + 12], read_data[(i * 16) + 13], read_data[(i * 16) + 14], read_data[(i * 16) + 15]);
        }
        free(read_data);
    }
    else if(strcmp((char*) argv[1], (const char*) "start_measurements") == 0)
    {
        //start taking measurements from sensor
        s_ToF_callback_handle = register_priority_handler_for_messages(uart_msg_queue_handler, ToF_public_component);
        uint8_t err = TOF_START_MEASUREMENTS();
        ESP_LOGI(TAG, "Error code is: %u", err);
    }
    else if(strcmp((char*) argv[1], (const char*) "stop_measurements") == 0)
    {
        //stop taking measurements from sensor
        uint8_t err = TOF_STOP_MEASUREMENTS();
        ESP_LOGI(TAG, "Error code is: %u", err);
        err = unregister_priority_handler_for_messages(ToF_public_component, s_ToF_callback_handle);
        ESP_LOGI(TAG, "Unreigster error code is: %u", err);
        s_ToF_callback_handle = 0;
    }
    else if(strcmp((char*) argv[1], (const char*) "set_tof_mode") == 0)
    {
        //switch between tof8821 mode and tof8828 mode
        if(argc < 3)
        {
            ESP_LOGE(TAG, "Incorrect size args");
            return;
        }
        bool set_mode = false;
        if(argv[2][0] == '1')
        {
            set_mode = true;
        }
        bool mode_val = TOF_SET_TMF8828_MODE(set_mode);
        if(mode_val)
        {
            ESP_LOGI(TAG, "Mode is now tmf8828.");
        }
        else
        {
            ESP_LOGI(TAG, "Mode is now tmf8821.");
        }
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

            write_bytes[write_cnt] = (uart_get_hex_from_char(argv[i][0]) * 16);

            if(uart_get_hex_from_char(argv[i][1]) == UART_INVALID_CHARACTER)
            {
                ESP_LOGE(TAG, "invalid write at byte %u", i);
                return;
            }

            write_bytes[write_cnt] += uart_get_hex_from_char(argv[i][1]);

            write_cnt++;
        }
        if(i < UART_MAX_ARGS)
        {
            write_bytes[write_cnt] = 0;
        }
        ESP_LOGI(TAG, "Writing %d bytes to key %s: ", write_cnt, argv[2]);
        FLASH_WRITE_TO_BLOB(MAIN_PARTITION, "flash", argv[2], write_bytes, write_cnt);
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
        uint8_t* read_data = FLASH_READ_FROM_BLOB(MAIN_PARTITION, "flash", argv[2], read_bytes);
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
            message.is_pointer = false;
            char* uart_msg = malloc(sizeof(char) * (strlen(argv[3]) + 1));
            memcpy(uart_msg, argv[3], sizeof(char) * strlen(argv[3]));
            uart_msg[strlen(argv[3])] = '\0';
            message.message_data = (void*) uart_msg;
            message.message_size = sizeof(char) * (strlen(argv[3]) + 1);
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
    else if(strcmp((char*) argv[1], (const char*) "init_normal_queue") == 0)
    {
        MESSAGE_QUEUE_INIT();
        bool is_active = check_is_queue_active(0);
        bool is_priority_active = check_is_queue_active(1);
        ESP_LOGI(TAG, "normal queue active: %u. priority queue active: %u.", is_active, is_priority_active);
    }
}

static void uart_imu_cmds(uint8_t argc, char** argv)
{
    if(argc < 2)
    {
        ESP_LOGE(TAG, "incorrect number of args");
        return;
    }
    if(strcmp((char*) argv[1], (const char*) "imu_read") == 0)
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
        IMU_READ(read_data, read_reg, read_bytes);
        ESP_LOGI(TAG, "Read the following bytes: ");
        for(uint8_t i = 0; i < read_bytes; i++)
        {
            ESP_LOGI(TAG, "%x", read_data[i]);
        }
        free(read_data);
    }
    else if(strcmp((char*) argv[1], (const char*) "imu_write") == 0)
    {
        uint8_t write_cnt = 0;
        uint8_t write_reg = 0;
        uint8_t write_bytes[UART_MAX_ARGS - 3] = {0};
        uint8_t i = 0;
        if(argc < 4)
        {
            ESP_LOGE(TAG, "Incorrect size args");
            return;
        }

        if(uart_get_hex_from_char(argv[2][0]) == UART_INVALID_CHARACTER)
        {
            ESP_LOGE(TAG, "invalid write at byte %u", i);
            return;
        }

        write_reg = (uart_get_hex_from_char(argv[2][0]) * 16);

        if(uart_get_hex_from_char(argv[2][1]) == UART_INVALID_CHARACTER)
        {
            ESP_LOGE(TAG, "invalid write at byte %u", i);
            return;
        }

        write_reg += uart_get_hex_from_char(argv[2][1]);

        for(i = 3; (i < argc) && (i < UART_MAX_ARGS); i++)
        {
            if(uart_get_hex_from_char(argv[i][0]) == UART_INVALID_CHARACTER)
            {
                ESP_LOGE(TAG, "invalid write at byte %u", i);
                return;
            }

            write_bytes[write_cnt] = (uart_get_hex_from_char(argv[i][0]) * 16);

            if(uart_get_hex_from_char(argv[i][1]) == UART_INVALID_CHARACTER)
            {
                ESP_LOGE(TAG, "invalid write at byte %u", i);
                return;
            }

            write_bytes[write_cnt] += uart_get_hex_from_char(argv[i][1]);

            write_cnt++;
        }
        if(i < UART_MAX_ARGS)
        {
            write_bytes[write_cnt] = 0;
        }
        ESP_LOGI(TAG, "Writing %d bytes to register 0x%x: ", write_cnt, write_reg);
        for(i = 0; i < write_cnt; i++)
        {
            ESP_LOGI(TAG, "%x", write_bytes[i]);
        }
        IMU_WRITE(write_bytes, write_reg, write_cnt);
    }
    else if(strcmp((char*) argv[1], (const char*) "start_measurements") == 0)
    {
        //start taking measurements from sensor
        s_imu_callback_handle = register_priority_handler_for_messages(uart_msg_queue_handler, imu_public_component);
        imu_accel_config();
        imu_gyro_config();
        imu_set_interrupts();
        uint8_t err = imu_start();
        ESP_LOGI(TAG, "Error code is: %u", err);
    }
    else if(strcmp((char*) argv[1], (const char*) "stop_measurements") == 0)
    {
        //stop taking measurements from sensor
        uint8_t err = imu_stop();
        ESP_LOGI(TAG, "Error code is: %u", err);
        err = unregister_priority_handler_for_messages(imu_public_component, s_imu_callback_handle);
        ESP_LOGI(TAG, "Unreigster error code is: %u", err);
        s_imu_callback_handle = 0;
    }
    else if(strcmp((char*) argv[1], (const char*) "reset") == 0)
    {
        //soft reset sensor
        uint8_t err = imu_reset();
        ESP_LOGI(TAG, "Error code is: %u", err);
    }
    else if(strcmp((char*) argv[1], (const char*) "check_status") == 0)
    {
        //check imu status register
        uint8_t err = imu_check_status();
        ESP_LOGI(TAG, "Status register is: 0x%x", err);
    }
    else if(strcmp((char*) argv[1], (const char*) "check_error") == 0)
    {
        //check imu error register
        uint8_t err = imu_check_error();
        ESP_LOGI(TAG, "Error register is: 0x%x", err);
    }
    else if(strcmp((char*) argv[1], (const char*) "check_events") == 0)
    {
        //check imu event register
        uint8_t err = imu_check_events();
        ESP_LOGI(TAG, "Event register is: 0x%x", err);
    }
}

static void uart_mtr_cmds(uint8_t argc, char** argv)
{
    if(argc < 2)
    {
        ESP_LOGE(TAG, "incorrect number of args");
        return;
    }
    if(strcmp((char*) argv[1], (const char*) "set_direction") == 0)
    {
        if(argc < 4)
        {
            ESP_LOGE(TAG, "Incorrect size args");
            return;
        }
        uint8_t handedness_val = uart_convert_str_to_handedness((char*) argv[2]);
        mtr_direction_t is_direction = uart_convert_str_to_direction((char*) argv[3]);
        bool is_right = (handedness_val > 2);
        if(!handedness_val)
        {
            ESP_LOGE(TAG, "handedness not set");
            return;
        }
        if(is_direction == MTR_DIR_INVALID)
        {
            ESP_LOGE(TAG, "invalid direction");
            return;
        }
        mtr_set_direction(is_right, is_direction);
        ESP_LOGI(TAG, "set motor %u to direction %u", is_right, is_direction);
    }
    if(strcmp((char*) argv[1], (const char*) "set_duty") == 0)
    {
        if(argc < 4)
        {
            ESP_LOGE(TAG, "Incorrect size args");
            return;
        }
        uint8_t handedness_val = uart_convert_str_to_handedness((char*) argv[2]);
        bool is_right = (handedness_val > 2);
        if(!handedness_val)
        {
            ESP_LOGE(TAG, "handedness not set");
            return;
        }
        uint16_t read_duty = 0;
        for(uint16_t i = 0; i < strlen(argv[3]); i++)
        {
            if(argv[3][i] >= '0' && argv[3][i] <= '9')
            {
                read_duty = read_duty * 10;
                read_duty += (uint16_t) (argv[3][i] - '0');
            }
        }
        if(read_duty > 8191)
        {
            ESP_LOGE(TAG, "invalid duty cycle");
            return;
        }
        mtr_set_duty(is_right, read_duty);
        ESP_LOGI(TAG, "set motor %u to duty cycle %u of 8192", is_right, read_duty);
    }
    if(strcmp((char*) argv[1], (const char*) "set_standby") == 0)
    {
        if(argc < 3)
        {
            ESP_LOGE(TAG, "Incorrect size args");
            return;
        }
        bool is_standby = false;
        if(strcmp((char*) argv[2], (const char*) "on"))
        {
            is_standby = true;
        }
        else if(!strcmp((char*) argv[2], (const char*) "off"))
        {
            ESP_LOGE(TAG, "invalid standby state");
            return;
        }
        mtr_set_standby(is_standby);
        ESP_LOGI(TAG, "set standby state to %u", is_standby);
    }
    if(strcmp((char*) argv[1], (const char*) "get_direction") == 0)
    {
        if(argc < 3)
        {
            ESP_LOGE(TAG, "Incorrect size args");
            return;
        }
        uint8_t handedness_val = uart_convert_str_to_handedness((char*) argv[2]);
        bool is_right = (handedness_val > 2);
        if(!handedness_val)
        {
            ESP_LOGE(TAG, "handedness not set");
            return;
        }
        ESP_LOGI(TAG, "motor %u is in direction %u", is_right, mtr_get_direction(is_right));
    }
    if(strcmp((char*) argv[1], (const char*) "get_duty") == 0)
    {
        if(argc < 3)
        {
            ESP_LOGE(TAG, "Incorrect size args");
            return;
        }
        uint8_t handedness_val = uart_convert_str_to_handedness((char*) argv[2]);
        bool is_right = (handedness_val > 2);
        if(!handedness_val)
        {
            ESP_LOGE(TAG, "handedness not set");
            return;
        }
        ESP_LOGI(TAG, "motor %u is in duty %u", is_right, mtr_get_duty(is_right));
    }
    if(strcmp((char*) argv[1], (const char*) "get_standby") == 0)
    {
        ESP_LOGI(TAG, "standby state is %u", mtr_get_standby());
    }
}

static void uart_serial_cmds(uint8_t argc, char** argv)
{
    if(argc < 3)
    {
        ESP_LOGE(TAG, "incorrect number of args");
        return;
    }
    if(strcmp((char*) argv[1], (const char*) "set_serialize") == 0)
    {
        if(strcmp((char*) argv[1], (const char*) "true") == 0)
        {
            s_serialize = true;
        }
        else if(strcmp((char*) argv[1], (const char*) "false") == 0)
        {
            s_serialize = false; 
        }
        ESP_LOGI(TAG, "serial value set to %d", s_serialize);
    }
}

static uint8_t uart_convert_str_to_handedness(char * cmd_buf)
{
    if(strcmp(cmd_buf, (const char*) "right"))
    {
        return 2;
    }
    else if(strcmp(cmd_buf, (const char*) "left"))
    {
        return 1;
    }
    return 0;
}

static mtr_direction_t uart_convert_str_to_direction(char * cmd_buf)
{
    if(strcmp(cmd_buf, (const char*) "stopped"))
    {
        return MTR_DIR_STOPPED;
    }
    else if(strcmp(cmd_buf, (const char*) "forward"))
    {
        return MTR_DIR_FORWARD;
    }
    else if(strcmp(cmd_buf, (const char*) "reverse"))
    {
        return MTR_DIR_REVERSE;
    }
    else if(strcmp(cmd_buf, (const char*) "standby"))
    {
        return MTR_DIR_NOT_SET;
    }
    return MTR_DIR_INVALID;
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
static uint8_t uart_convert_str_to_args(char * cmd_buf, char** argv_ptr, uint8_t argv_max)
{
    uint8_t argc = 0;

    char *p2 = strtok(cmd_buf, " ");
    while (p2 && argc < argv_max-1)
    {
        argv_ptr[argc++] = p2;
        p2 = strtok(0, " ");
    }
    argv_ptr[argc] = 0;

    return argc;
}

static void uart_msg_queue_handler(component_handle_t component_type, uint8_t message_type, void* message_data, size_t message_size)
{
    dispatcher_type_t dispatcher = uart_get_dispatcher_from_component(component_type);
    uint8_t checksum = 0;
    s_serial_out[0] = 254;
    s_serial_out[1] = 254;
    s_serial_out[2] = 254;
    s_serial_out[3] = 0; //invalid size
    s_serial_out[4] = 0; //invalid type
    if(!s_serialize)
    {
        ESP_LOGI(TAG, "message from %s with message type %u and size %u.", uart_return_string_from_dispatcher(dispatcher), message_type, message_size);
    }
    if(component_type == ToF_public_component && message_type == TOF_MSG_NEW_DEPTH_ARRAY)
    {
        //write TOF_DATA_t to console
        TOF_DATA_t* tof_data = (TOF_DATA_t*) message_data;
        uint8_t h_size = tof_data->horizontal_size;
        uint8_t v_size = tof_data->vertical_size;
        uint16_t** array_ptr = tof_data->depth_pixel_field;
        if(s_serialize)
        {
            //write header data to s_serial_out
            s_serial_out[0] = 254;
            s_serial_out[1] = 254;
            s_serial_out[2] = 254;
            if(h_size == 8)
            {
                s_serial_out[3] = 128; //128 bytes of data
            }
            else
            {
                s_serial_out[3] = 32; //32 bytes of data
            }
            s_serial_out[4] = 4; //data type is ToF
        }
        else
        {
            ESP_LOGI(TAG, "outputting %ux%u depth array:", h_size, v_size);
        }
        for(uint8_t j = 0; j < v_size; j++)
        {
            if(h_size == 8)
            {
                if(s_serialize)
                {
                    //serialize 128 bytes of data
                    for(uint8_t k = 0; k < v_size; k++)
                    {
                        s_serial_out[(16*j) + (k*2) + 5] = array_ptr[j][k] & 0xFF;
                        s_serial_out[(16*j) + (k*2) + 6] = (array_ptr[j][k] >> 8);
                    }
                }
                else
                {
                    ESP_LOGI(TAG, "%04u %04u %04u %04u %04u %04u %04u %04u", 
                        array_ptr[j][0], array_ptr[j][1], array_ptr[j][2], array_ptr[j][3],
                        array_ptr[j][4], array_ptr[j][5], array_ptr[j][6], array_ptr[j][7]);
                }
            }
            else if(h_size == 4)
            {
                if(s_serialize)
                {
                    //serialize 32 bytes of data
                    for(uint8_t k = 0; k < v_size; k++)
                    {
                        s_serial_out[(8*j) + (k*2) + 5] = array_ptr[j][k] & 0xFF;
                        s_serial_out[(8*j) + (k*2) + 6] = (array_ptr[j][k] >> 8);
                    }
                }
                else
                {
                    ESP_LOGI(TAG, "%04u %04u %04u %04u", 
                        array_ptr[j][0], array_ptr[j][1], array_ptr[j][2], array_ptr[j][3]);
                }
            }
            
        }
    }
    else if (component_type == imu_public_component && message_type == IMU_MSG_RAW_DATA)
    {
        IMU_DATA_RAW_t *imu_data = (IMU_DATA_RAW_t *) message_data;
        uint32_t timestamp = (imu_data->timestamp[2] << 16) + (imu_data->timestamp[1] << 8) + imu_data->timestamp[0];
        if(s_serialize)
        {
            //write header data to s_serial_out
            s_serial_out[0] = 254;
            s_serial_out[1] = 254;
            s_serial_out[2] = 254;
            s_serial_out[3] = 3;
            s_serial_out[4] = imu_data->flags; //data type is acc (1), gyro (2), or both (3)
            s_serial_out[5] = imu_data->timestamp[0];
            s_serial_out[6] = imu_data->timestamp[1];
            s_serial_out[7] = imu_data->timestamp[2];
            if(imu_data->flags & 0x01)
            {
                s_serial_out[5 + s_serial_out[3]] = imu_data->acc_data[0];
                s_serial_out[6 + s_serial_out[3]] = imu_data->acc_data[1];
                s_serial_out[7 + s_serial_out[3]] = imu_data->acc_data[2];
                s_serial_out[8 + s_serial_out[3]] = imu_data->acc_data[3];
                s_serial_out[9 + s_serial_out[3]] = imu_data->acc_data[4];
                s_serial_out[10 + s_serial_out[3]] = imu_data->acc_data[5];
                s_serial_out[3] += 6;
            }
            if(imu_data->flags & 0x02)
            {
                s_serial_out[5 + s_serial_out[3]] = imu_data->gyr_data[0];
                s_serial_out[6 + s_serial_out[3]] = imu_data->gyr_data[1];
                s_serial_out[7 + s_serial_out[3]] = imu_data->gyr_data[2];
                s_serial_out[8 + s_serial_out[3]] = imu_data->gyr_data[3];
                s_serial_out[9 + s_serial_out[3]] = imu_data->gyr_data[4];
                s_serial_out[10 + s_serial_out[3]] = imu_data->gyr_data[5];
                s_serial_out[3] += 6;
            }
        }
        else
        {
            ESP_LOGI(TAG, "timestamp %lu imu data:", timestamp);
            for(uint8_t i = 0; i < 3; i++)
            {
                uint16_t raw_accel = (imu_data->acc_data[(2*i) + 1] << 8) + imu_data->acc_data[(2*i)];
                uint16_t raw_gyro = (imu_data->gyr_data[(2*i) + 1] << 8) + imu_data->gyr_data[(2*i)];
                ESP_LOGI(TAG, "%u: accel %04x, gyro %04x", i, raw_accel, raw_gyro);
            }
        }
    }
    else
    {
        if(message_size > 200)
        {
            ESP_LOGE(TAG, "message size too large:");
            return;
        }
        ESP_LOGI(TAG, "message data:");
        char* output_data = (char*) message_data;
        ESP_LOGI(TAG, "%s", output_data);
    }
    if(s_serialize)
    {
        for(uint8_t check_size = 0; check_size < (s_serial_out[3] + 5); check_size++)
        {
            checksum = checksum ^ s_serial_out[check_size];
        }
        s_serial_out[5 + s_serial_out[3]] = checksum; //checksum
        s_serial_out[6 + s_serial_out[3]] = 0;
        //write data out via UART
        fwrite(s_serial_out, sizeof(uint8_t), 6 + s_serial_out[3], stdout);
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

static void poll_stdin(void* args)
{
    char buf[128] = {0};
    char newchar = 0;
    uint8_t readlen = 0;

    while(true)
    {
        newchar = fgetc(stdin);
        if(newchar == 0x08 || newchar == 0xff)
        {
            readlen--;
            buf[readlen] = '\0';
        }
        if(newchar >= ' ')
        {
            buf[readlen] = newchar;
            readlen++;
        }
        if(readlen > 127 || newchar == '\r' || newchar == '\n')
        {
            buf[readlen] = '\0';
            run_command(readlen, buf);
            memset(buf, 0, 128);
            readlen = 0;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

static void run_command(uint8_t rx_size, char *buf)
{
    /* initialization */
    char *argv[UART_MAX_ARGS] = {0};
    uint8_t argc = 0;

    ESP_LOGI(TAG, "Got data (%d bytes): %s", rx_size, buf);
    
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
            uart_imu_cmds(argc, argv);
            break;
        }
        case motor:
        {
            uart_mtr_cmds(argc, argv);
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
            uart_serial_cmds(argc, argv);
            break;
        }
        default:
        {
            ESP_LOGI(TAG, "invalid specifier");
            break;
        }
    }
}

#endif

void UART_INIT(void)
{
    ESP_LOGI(TAG, "USB initialization");

#ifndef FUNCTIONAL_TESTS

    setvbuf(stdin, NULL, _IONBF, 0);
    fcntl(fileno(stdout), F_SETFL, 0);
    fcntl(fileno(stdin), F_SETFL, 0);
    xTaskCreate(poll_stdin, "poll_stdin", 16384, NULL, 0, NULL);

#endif

    memset(UART_callback_handles, 0, sizeof(UART_callback_handles));

    ESP_LOGI(TAG, "USB initialization DONE");
}
