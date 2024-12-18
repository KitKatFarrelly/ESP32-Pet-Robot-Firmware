#ifndef H_NAV_ALGO
#define H_NAV_ALGO

#include <stdbool.h>

#include "MESSAGE_QUEUE.h"

//128 * 8 bytes = 1kB per submap.
#define MAX_POINTS_PER_SUBMAP 10 //128 undo this after testing

#define MAX_SUBMAPS_PER_DIRECTION 10

extern component_handle_t nav_algo_public_component;

typedef enum
{
    NAV_RAW_FEATURE_DATA,
    NAV_TRANSFORM_DATA,
    NAV_MAP_DATA,
    NAV_MSG_MAX,
} NAV_MESSAGE_TYPES_t;

typedef uint32_t NAV_MAP_HANDLER_T;

//nav_point_t consists of 4 fields position, size, rotation, confidence
//in total 64 bits of info per point, or 8 bytes.
typedef struct
{
    //10 bits per x, y, z. Should be millimeter accuracy
    uint32_t xyz_pos;
    //8 bits per width, height. 0 in height field means infinite height
    uint8_t width;
    uint8_t height;
    //8 bits for angle, about 1.4 degrees per bit
    uint8_t rotation;
    //adjusted confidence value based on depth data
    uint8_t confidence;
} NAV_POINT_T;

typedef struct
{
    NAV_POINT_T pointCloud[MAX_POINTS_PER_SUBMAP]; //Change this to dynamic allocation maybe
} NAV_SUBMAP_T;

typedef struct
{
    NAV_SUBMAP_T map[MAX_SUBMAPS_PER_DIRECTION][MAX_SUBMAPS_PER_DIRECTION]; //Change this to dynamic allocation maybe
    NAV_MAP_HANDLER_T handler;
} NAV_MAP_T;

typedef struct
{
    uint8_t number_of_nodes_in_feature;
    uint8_t min_x;
    uint8_t max_x;
    uint8_t min_y;
    uint8_t max_y;
    int16_t average_angle;
    uint16_t average_distance;
    uint16_t average_confidence;
} dfs_feature_details_t;

bool nav_algo_init(void);

bool nav_algo_enable_navigation(bool enable);

bool nav_algo_restart_temp_map(void);

NAV_MAP_HANDLER_T nav_algo_closest_map_to_temp_map(void);

NAV_MAP_HANDLER_T nav_algo_start_writing_map(void);

bool nav_algo_stop_writing_map(NAV_MAP_HANDLER_T map);

bool nav_algo_save_map(NAV_MAP_HANDLER_T map_to_save);

bool nav_algo_load_map(NAV_MAP_HANDLER_T map_to_load);

NAV_MAP_HANDLER_T nav_algo_get_current_map(void);

NAV_SUBMAP_T *nav_algo_get_submap(int16_t submap_x, int16_t submap_z);

bool nav_algo_enable_debug_messages(bool enable);

#endif