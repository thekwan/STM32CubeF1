#include "map.h"

#define PI 3.1415926535

MapManager::MapManager(std::string fileName) {
    std::cout << "[INFO] read file: '" << fileName << "\n";

    std::ifstream ifs;
    ifs.open(fileName, std::ios::binary | std::ios::in);
    if (!ifs.is_open()) {
        std::cerr << "[ERROR] Can't open the file.\n";
    }

    int frame_count = 0;

    while (!ifs.eof()) {
        LidarFrame frame;
        int frame_length;
        uint8_t qual;
        float dist, angle;
        ifs.read((char*) &frame_length, sizeof(int));

        std::cout << "[INFO] frame[" << std::to_string(frame_count++) << "]"
            << "[" << std::to_string(frame_length) << "]\n";

        frame.points_.clear();

        for (int i = 0; i < frame_length; i++) {
            ifs.read((char*)&qual, sizeof(uint8_t));
            ifs.read((char*)&dist, sizeof(float));
            ifs.read((char*)&angle, sizeof(float));

            frame.points_.emplace_back((int)qual, dist, angle, 
                    dist*cos(angle*PI/360.0), dist*sin(angle*PI/360.0));
        }

        frames_.push_back(frame);
    }

    std::cout << "[INFO] Total lidar frames: " << frames_.size() << "\n";
}

MapManager::~MapManager(void) {
}

int MapManager::getMapMaxIndex(void) {
    return frames_.size() - 1;
}

LidarFrame *MapManager::getLidarFrame(int frame_index) {
    if (frame_index < 0) {
        return &frames_[0];
    }
    else if (frame_index > getMapMaxIndex()) {
        return &frames_.back();
    }

    return &frames_[frame_index];
}
