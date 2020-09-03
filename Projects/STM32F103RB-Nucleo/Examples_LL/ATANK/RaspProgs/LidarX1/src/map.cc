#include "map.h"

MapManager::MapManager(void) {
    end_angle_last = 0.0;
    start_angle_last = 0.0;
}

MapManager::~MapManager(void) {
}

void MapManager::push_map_point(dist_frame_t &df) {
    float angle_step = df.end_angle - df.start_angle;
    float angle = df.start_angle;
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

    //std::cout << "start_angle = " << df.start_angle << std::endl;
    //std::cout << "end_angle   = " << df.end_angle << std::endl;
    //std::cout << "s.angle(FLP)= " << angle << std::endl;
    //std::cout << "angle_step  = " << angle_step << std::endl;

    for(int i = 0; i < 8; i++) {
        point2_t pt;
        pt.x = (float)df.distance[i] * cos(angle);
        pt.y = (float)df.distance[i] * sin(angle);
        pt.quality = df.quality[i];
        angle += angle_step;

        //std::cout << "distance[" << i << "] = " << (int)df.distance[i] << std::endl;
        //std::cout << "pt[" << i << "] = " << pt.x << "," << pt.y << std::endl;
        //std::cout << "quality = " << (u16)df.quality[i] << std::endl;

        if (df.quality[i] > 0 ) {
            _point_q.push_back(pt);
            _map_temp.pts.push_back(pt);
        }
    }

    //std::cout << std::endl;
}

std::vector<point2_t> MapManager::get_map_point(void) {
    return _point_q;
}

std::vector<point2_t> MapManager::get_map_point(int index) {
    return _map_list[index].pts;
}

int MapManager::get_map_point_num(void) {
    return _point_q.size();
}

int MapManager::get_map_num(void) {
    return _map_list.size();
}

void MapManager::check_all_map_points(void) {
    int q_size = _point_q.size();
    std::cout << "/*************************************************\n";
    std::cout << " * Point size = " << q_size << std::endl;
    std::cout << " *************************************************/\n";

    for(int i = 0; i < 400; i++) {
        std::cout << _point_q.at(i).x << "\t" << _point_q.at(i).y << std::endl;
    }
}

void MapManager::push_frame_data(const u8 *fdata) {
    dist_frame_t dframe;

    u16 speed_rpm;
    u16 start_angle, end_angle;
    
    // angular speed (rpm)
    dframe.speed_rpm   = ((u16)fdata[1]) << 8 | (u16)fdata[0];

    // start-angle calculation
    start_angle = ((u16)fdata[3]) << 8 | (u16)fdata[2];
    dframe.start_angle = (float)start_angle/64.0 - 640.0;

    // end-angle calculation
    end_angle = ((u16)fdata[29]) << 8 | (u16)fdata[28];
    dframe.end_angle = (float)end_angle/64.0 - 640.0;

    if (dframe.end_angle < 0) {
        dframe.end_angle += 360.0;
    }
    if (dframe.end_angle < 0) {
        // exception case: give up the map data.
        return;
    }


    if (dframe.start_angle < end_angle_last ||
        dframe.start_angle < start_angle_last ) {
        //std::cout <<"[INFO] Detect the end of frame !!!!!!!\n";
        _map_list.push_back(_map_temp);
        _map_temp.pts.clear();
        std::cout << "------- map_frame.num = " << _map_list.size() << " ------\n";
    }
    end_angle_last = dframe.end_angle;
    start_angle_last = dframe.start_angle;


    for(int i = 0; i < 8; i++) {
        dframe.distance[i] = ((u16)fdata[5+3*i]) << 8 | (u16)fdata[4+3*i];
        dframe.quality[i]  = (u8)fdata[6+3*i];
    }

    // no parity check
    
    push_map_point(dframe);

    //std::cout << "speed_rpm   = " << speed_rpm << std::endl;
    //std::cout << "start_angle = " << start_angle << std::endl;
    //std::cout << "end_angle   = " << (u16)end_angle << std::endl;
    std::cout << "angle = " << dframe.start_angle;
    std::cout << " ~ " << dframe.end_angle;
    //std::cout << "start_angle[0] = " << std::hex << (u16) fdata[2] << std::endl;
    //std::cout << "start_angle[1] = " << std::hex << (u16) fdata[3] << std::endl;
    //for(int i = 0; i < 8; i++) {
    //    std::cout << "distance[" << i << "] = " << distance[i];
    //    std::cout << "\tquality = " << (u16)quality[i] << std::endl;
    //}
    std::cout << std::endl;
}
