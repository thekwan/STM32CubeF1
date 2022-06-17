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

typedef struct _point2f {
    _point2f(float a, float b) : x(a), y(b) {}
    _point2f() : x(0), y(0) {}
    float x;
    float y;
} Point2f;

typedef std::pair<Point2f,Point2f>  Point2fPair;

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
    void icpProc(int map_index);
    std::vector<Point2f> getPointsGoodQual(std::vector<LidarPoint> &fr);
    std::vector<Point2fPair> findAngleMatchedPoints(
            std::vector<LidarPoint> &fA, std::vector<LidarPoint> &fB);
    float calcNormDist(std::vector<Point2fPair> &ppair);
    float calcNormDist(Point2f &a, Point2f &b);
    float calcAngleDist(LidarPoint &a, LidarPoint &b);
private:
    bool isInitialized_;
    int8_t qualThreshold_;
    std::vector<LidarFrame>  frames_;
};

#endif  // __MAP_H__
