#ifndef H_NAV_ALGO
#define H_NAV_ALGO

//512 * 8 bytes = 4kB per submap.
#define MAX_POINTS_PER_SUBMAP 512

//nav_point_t consists of 5 fields x, y, z, size, confidence
//in total 64 bits of info per point, or 8 bytes.
typedef struct
{
    uint16_t x;
    uint16_t y;
    uint16_t z;
    uint8_t size;
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
    int lowest_submap_x;
    int highest_submap_x;
    int lowest_submap_y;
    int highest_submap_y;
} NAV_MAP_SIZE_T;

typedef struct
{
    uint32_t handler;
} NAV_MAP_T;

bool nav_algo_init(void);

bool nav_algo_enable_navigation(bool enable);

bool nav_algo_restart_temp_map(void);

NAV_MAP_T nav_algo_closest_map_to_temp_map(void);

NAV_MAP_T nav_algo_start_writing_map(void);

bool nav_algo_stop_writing_map(NAV_MAP_T map);

bool nav_algo_save_map(NAV_MAP_T map_to_save);

bool nav_algo_load_map(NAV_MAP_T map_to_load);

NAV_MAP_T nav_algo_get_current_map(void);

NAV_MAP_SIZE_T nav_algo_get_current_size(void);

NAV_SUBMAP_T *nav_algo_get_submap(int submap_x, int submap_y);

#endif