#include "NAV_ALGO.h"
#include "MESSAGE_QUEUE.h"
#include "ToF_I2C.h"
#include "IMU_SPI.h"
#include "FLASH_SPI.h"

#define MAX_FEATURES_PER_TOF_ARRAY 10

typedef struct
{
    NAV_POINT_T features[MAX_FEATURES_PER_TOF_ARRAY];
    uint8_t number_of_features;
} feature_extraction_t;

typedef struct
{
    NAV_POINT_T robot_pos;
    uint8_t submap_x;
    uint8_t submap_y;
} robot_position_t;

static callback_handle_t s_nav_tof_handle;
static callback_handle_t s_nav_imu_handle;
static robot_position_t s_nav_robot_position;
static NAV_MAP_T s_nav_map;
static bool s_is_navigation_enabled = false;

static const char *TAG = "NAV_ALG";

static void nav_algo_queue_handler(component_handle_t component_type, uint8_t message_type, void* message_data, size_t message_size);
static uint8_t nav_algo_convert_adjusted_confidence_value(uint16_t distance, uint8_t confidence);
//placeholder for imu kalman filter function
static void nav_algo_check_tof_array_against_map(TOF_DATA_t* tof_data);

bool nav_algo_init(void)
{
    s_nav_tof_handle = register_priority_handler_for_messages(nav_algo_queue_handler, ToF_public_component);
    s_nav_imu_handle = register_priority_handler_for_messages(nav_algo_queue_handler, imu_public_component);
    return true;
}

bool nav_algo_enable_navigation(bool enable)
{
    if(enable)
    {
        if(TOF_START_MEASUREMENTS())
        {
            return false;
        }
    }
    else
    {
        if(TOF_STOP_MEASUREMENTS())
        {
            return false;
        }
    }
    s_is_navigation_enabled = enable;
    return s_is_navigation_enabled;
}

bool nav_algo_restart_temp_map(void)
{
    return false;
}

NAV_MAP_HANDLER_T nav_algo_closest_map_to_temp_map(void)
{
    return NULL;
}

NAV_MAP_HANDLER_T nav_algo_start_writing_map(void)
{
    return NULL;
}

bool nav_algo_stop_writing_map(NAV_MAP_HANDLER_T map)
{
    return false;
}

bool nav_algo_save_map(NAV_MAP_HANDLER_T map_to_save)
{
    return false;
}

bool nav_algo_load_map(NAV_MAP_HANDLER_T map_to_load)
{
    return false;
}

NAV_MAP_HANDLER_T nav_algo_get_current_map(void)
{
    return NULL;
}

NAV_CURRENT_POS_T nav_algo_get_current_pos(void)
{
    return NULL;
}

NAV_SUBMAP_T *nav_algo_get_submap(int submap_x, int submap_y)
{
    return NULL;
}

static void nav_algo_queue_handler(component_handle_t component_type, uint8_t message_type, void* message_data, size_t message_size)
{
    if(!s_is_navigation_enabled)
    {
        ESP_LOGI(TAG, "navigation not enabled, ignoring.");
        return;
    }
    if(component_type == ToF_public_component && message_type == TOF_MSG_NEW_DEPTH_ARRAY)
    {
        nav_algo_check_tof_array_against_map((TOF_DATA_t*) message_data);
    }
    else if(component_type == imu_public_component && message_type == IMU_MSG_RAW_DATA)
    {

    }
}

static uint8_t nav_algo_convert_adjusted_confidence_value(uint16_t distance, uint8_t confidence)
{
    float base_mult = 6.0;
    float square_val = (1000.0 / base_mult); //whatever the multiplier should be at 1000 mm
    float square_dist = (float) distance;
    float multiplier = 1.0 + ((square_dist * square_dist) / (square_val * square_val * base_mult));
    uint16_t ret_val = (uint16_t) (multiplier * (float) confidence);
    return (uint8_t) (ret_val & 0xFF);
}

//perform feature extraction from tof data
//generate up to MAX_FEATURES_PER_TOF_ARRAY landmarks
static feature_extraction_t nav_algo_feature_extraction_from_tof_data(TOF_DATA_t* tof_data)
{
    feature_extraction_t return_features_list = {0};
    //as all features are considered to be planes in this design, features are extracted like so:
    //1. create a 2x2 convolution of each point to determine vertical and horizontal gradient, starting from top left
    //2. dfs to find islands of features within the convolution with similar gradients.
    //3. Combine islands that are likely the same island, then order islands from largest to smallest in terms of number of cells.
    //4. determine center of mass of each island, xyz dimensions, as well as their orientation vs the robot.
    return return_features_list;
}

//read tof map and run error vs existing map to estimate movement
//also create and adjust objects on each submap
static void nav_algo_check_tof_array_against_map(TOF_DATA_t* tof_data)
{
    //step 1: generate landmarks
    feature_extraction_t feaures_list = nav_algo_feature_extraction_from_tof_data(tof_data);

    //step 2: find 2 most confident landmarks
    if(!features_list.number_of_feaures)
    {
        //no features were extracted from the array.
        return;
    }

    //step 3: determine rotation + translation error of landmark 1 to each existing landmark on submap
    NAV_POINT_T landmark_error[MAX_POINTS_PER_SUBMAP] = {0};
    NAV_SUBMAP_T * submap_pointer = &(s_nav_map.map[s_nav_robot_position.submap_x][s_nav_robot_position.submap_y]); //maybe have a getter/setter for this...
    for(uint8_t iter = 0; iter < MAX_POINTS_PER_SUBMAP; iter++)
    {
        if(!submap_pointer->pointCloud[iter].confidence)
        {
            //break out once pointclouds start becoming invalid
            break;
        }
        //calculate error of pointCloud[iter] - features_list[0]
        //if error is sufficiently close to s_nav_robot_position for any given feature in the pointcloud, we have matched the feature.
        //otherwise, continue with step 3.1
    }

    //step 3.1: if necessary, determine rotation + translation error of landmark 2 to each existing landmark on submap

    //step 3.2: if necessary, compare each set of errors to build array of potential transforms based on lowest shared error.

    //step 3.3: if necessary, compare each potential transform against 3, 4, 5, etc landmarks until one landmark remains

    //step 4: update location

    //step 5: update submap with map landmark info
}

//Basic mapping idea:
//turn any given array into a set of landmarks and translate/rotate array according to estimated robot location
//calculate distances of one landmark in array to all landmarks in submap and calculate rotational error for each
//do the same for a second landmark in array.
//try to find minimum error between two rotational and translation vectors. Use average of the two vectors as array transform onto map
//update landmarks on map according to new landmarks from array
