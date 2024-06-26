#ifndef H_NAV_ALGO
#define H_NAV_ALGO

//512 * 8 bytes = 4kB per submap.
#define MAX_POINTS_PER_SUBMAP 512

//nav_point_t consists of 4 fields position, size, rotation, confidence
//in total 64 bits of info per point, or 8 bytes.
typedef struct
{
    //10 bits per x, y, z. Should be millimeter accuracy
    uint32_t xyz_pos;
    //8 bits per width, height. 0 in height field means infinite height
    uint16_t width_height_size;
    //8 bits for angle, about 1.4 degrees per bit
    uint8_t rotation;
    //adjusted confidence value based on depth data
    uint8_t confidence;
} NAV_POINT_T;

typedef struct
{
    NAV_POINT_T pointCloud[MAX_POINTS_PER_SUBMAP];
    int submap_x;
    int submap_y;
} NAV_SUBMAP_T;

typedef struct
{
    uint32_t handler;
} NAV_MAP_T;

typedef struct 
{
    int submap_x;
    int submap_y;
    uint16_t cur_x;
    uint16_t cur_y;
    uint16_t cur_z;
    uint16_t cur_rotation;
} NAV_CURRENT_POS_T;

bool nav_algo_init(void);

bool nav_algo_enable_navigation(bool enable);

bool nav_algo_restart_temp_map(void);

NAV_MAP_T nav_algo_closest_map_to_temp_map(void);

NAV_MAP_T nav_algo_start_writing_map(void);

bool nav_algo_stop_writing_map(NAV_MAP_T map);

bool nav_algo_save_map(NAV_MAP_T map_to_save);

bool nav_algo_load_map(NAV_MAP_T map_to_load);

NAV_MAP_T nav_algo_get_current_map(void);

NAV_CURRENT_POS_T nav_algo_get_current_pos(void);

NAV_SUBMAP_T *nav_algo_get_submap(int submap_x, int submap_y);

#endif