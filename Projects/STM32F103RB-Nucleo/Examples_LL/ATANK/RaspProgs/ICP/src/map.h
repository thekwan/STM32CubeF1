#ifndef __MAP_H__
#define __MAP_H__

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <glog/logging.h>

typedef struct _point2f {
    _point2f(float a, float b) : x(a), y(b) {}
    _point2f() : x(0), y(0) {}
    float x;
    float y;
    float norm(void) {
        return sqrt(x*x + y*y);
    }
} Point2f;

typedef struct _LidarPoint {
    int qual;
    float dist;
    float angle;
    Point2f point;
    _LidarPoint (void) {}
    _LidarPoint (int qual_, float dist_, float angle_, float cx_, float cy_) :
        qual(qual_), dist(dist_), angle(angle_), point(cx_, cy_) {}
} LidarPoint;

typedef struct _LidarFrame {
    std::vector<LidarPoint> points_;
} LidarFrame;

typedef struct _Point2fPair {
    std::pair<Point2f, Point2f> points;
    bool outlier;
    float distance;

    _Point2fPair(Point2f a, Point2f b) : points(a,b), outlier(false), distance(0) {}
} Point2fPair;

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
    void findOptimalRotation(int frame_index);
    void findOptimalTranslation(int frame_index);
    void icpProc(int map_index);
    //void icpProc(std::vector<Point2fPair> &ppair);
    //void icpCore(void);
    std::vector<Point2f> getPointsGoodQual(std::vector<LidarPoint> &fr);
    std::vector<Point2fPair> findAngleMatchedPoints(
            std::vector<LidarPoint> &fA, std::vector<LidarPoint> &fB);
    std::vector<Point2fPair> findClosestPoints(
            std::vector<LidarPoint> &fA, std::vector<LidarPoint> &fB);
    float calcNormDist(std::vector<Point2fPair> &ppair);
    float calcNormDist(Point2f &a, Point2f &b);
    float calcAngleDist(LidarPoint &a, LidarPoint &b);
    void getFrameDistance(std::vector<LidarPoint> &a, 
            std::vector<LidarPoint> &b, int offset, float *dist, float *angle);
    std::vector<Point2fPair>* getPointPairs(void);
    Point2f getCentroidOfPoints(std::vector<LidarPoint> &pts);
    void removeOutlierFromPairList(std::vector<Point2fPair> &pairs, float thr_scale);
    Point2f getTranslationFromPair(std::vector<Point2fPair> &pairs);
private:
    bool isInitialized_;
    int8_t qualThreshold_;
    std::vector<LidarFrame>  frames_;
    std::vector<Point2fPair> pairList_;
};

#endif  // __MAP_H__
