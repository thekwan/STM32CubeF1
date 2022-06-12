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
    int8_t getQualThreshold(void);
    void dumpPointData(int frame_index);
    bool isReady(void) { return isInitialized_; }
    void checkFrameDistance(int frame_index);
    int findAngleOffset(std::vector<LidarPoint> &frame, float angle);
    float calcNormDist( std::vector<LidarPoint> &fa, 
            std::vector<LidarPoint> &fb, int offset);
    float calcNormDist(LidarPoint &a, LidarPoint &b);
private:
    bool isInitialized_;
    int8_t qualThreshold_;
    std::vector<LidarFrame>  frames_;
};

#endif  // __MAP_H__
