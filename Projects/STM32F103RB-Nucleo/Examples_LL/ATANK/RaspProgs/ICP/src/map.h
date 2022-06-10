#ifndef __MAP_H__
#define __MAP_H__

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

typedef struct _LidarPoint {
    int qual;
    float dist;
    float angle;
    float cx, cy;
    _LidarPoint (int qual_, float dist_, float angle_, float cx_, float cy_) :
        qual(qual_), dist(dist_), angle(angle_), cx(cx_), cy(cy_) {}
} LidarPoint;

typedef struct _LidarFrame {
    std::vector<LidarPoint> points_;
} LidarFrame;

class MapManager {
public:
    MapManager(std::string fileName);
    ~MapManager();
    int getMapMaxIndex(void);
    LidarFrame *getLidarFrame(int frame_index);
private:
    std::vector<LidarFrame>  frames_;
};

#endif  // __MAP_H__
