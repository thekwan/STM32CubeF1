#ifndef __MAP_H__
#define __MAP_H__

#include <iostream>
#include <vector>
#include <cmath>

typedef unsigned short u16;
typedef unsigned char  u8;

typedef struct _distance_frame {
    u16 speed_rpm;
    float start_angle;
    float end_angle;
    u16 distance[8]; // [29:6]
    u8  quality[8];
} dist_frame_t;

typedef struct _point2_t {
    float x, y;
    int  quality;
} point2_t;

typedef struct _map_frame_t{
    std::vector<point2_t>  pts;
} map_frame_t;

class MapManager {
public:
    MapManager();
    ~MapManager();
    void push_map_point(dist_frame_t &df);
    std::vector<point2_t> get_map_point(void);
    std::vector<point2_t> get_map_point(int);
    void check_all_map_points(void);
    int get_map_point_num(void);
    int get_map_num(void);
    void push_frame_data(const u8 *fdata);
private:
    float end_angle_last;
    float start_angle_last;

    map_frame_t               _map_temp;
    std::vector<map_frame_t>  _map_list;
    std::vector<point2_t>  _point_q;
};

#endif  // __MAP_H__
