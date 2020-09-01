#include "map.h"

MapManager::MapManager(void) {
}

MapManager::~MapManager(void) {
}

void MapManager::push_map_point(dist_frame_t *df) {
    float angle_step = df->end_angle - df->start_angle;
    float angle = df->start_angle;
    if (angle_step < 0) {
        angle_step += 360.0;
    }
    angle_step /= 8.0;
    angle_step = (angle_step * M_PI) / 180.0;
    angle = (angle * M_PI) / 180.0;

    if (angle_step < 0) {
        // WA-code: give-up due to invalid data frame.
        return;
    }

    //std::cout << "start_angle = " << df->start_angle << std::endl;
    //std::cout << "end_angle   = " << df->end_angle << std::endl;
    //std::cout << "s.angle(FLP)= " << angle << std::endl;
    //std::cout << "angle_step  = " << angle_step << std::endl;

    for(int i = 0; i < 8; i++) {
        point2_t pt;
        pt.x = (float)df->distance[i] * cos(angle);
        pt.y = (float)df->distance[i] * sin(angle);
        pt.quality = df->quality[i];
        angle += angle_step;

        //std::cout << "distance[" << i << "] = " << (int)df->distance[i] << std::endl;
        //std::cout << "pt[" << i << "] = " << pt.x << "," << pt.y << std::endl;
        std::cout << "quality = " << (u16)df->quality[i] << std::endl;

        //if (df->quality[i] > 0 ) {
            point_q.push_back(pt);
        //}
    }

    //std::cout << std::endl;
}

std::vector<point2_t> MapManager::get_map_point(void) {
    return point_q;
}

int MapManager::get_map_point_num(void) {
    return point_q.size();
}

void MapManager::check_all_map_points(void) {
    int q_size = point_q.size();
    std::cout << "/*************************************************\n";
    std::cout << " * Point size = " << q_size << std::endl;
    std::cout << " *************************************************/\n";

    for(int i = 0; i < 400; i++) {
        std::cout << point_q.at(i).x << "\t" << point_q.at(i).y << std::endl;
    }
}
