#ifndef __MAP_H__
#define __MAP_H__

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <glog/logging.h>

#include "lidar.h"

class MapManager {
public:
    MapManager(const std::string& fileName, const int qualThreshold = 80);
    ~MapManager();

    // Write a CSV file for lidar frames.
    void writeLidarFrameData(void);
    void readLidarDataFromFile(const std::string& fname);

    std::vector<LidarPoint>& getLidarFramePoints(int frame_index);
    bool isReady(void) { return isInitialized_; }
    int8_t getQualThreshold(void) { return qualThreshold_; }
    int getMapMaxIndex(void) { return frames_.size() - 1; }

    void printFrameInfo(void);
 #if 0
    LidarFrame *getLidarFrame(int frame_index);
    void dumpPointData(int frame_index);
    float getEstAngle(void) { return estRotateAngle_; }

    void checkFrameDistance(int frame_index);
    /* Rotation estimation functions
     */
    void findOptimalRotation(int frame_index);
    void getDistBasedRotationAngle(int frame_index);
    float getIterativeBasedRotationAngle(int frame_index);
    float calcErrorGivenAngle(LidarFrame &a, LidarFrame &b, float angle);

    /* Translation estimation functions
     */
    void findOptimalTranslation(int frame_index);
    Point2f getCentroidOfPoints(std::vector<LidarPoint> &pts);
    void removeOutlierFromPairList(std::vector<Point2fPair> &pairs, float thr_scale);
    void getTranslationFromPair(std::vector<Point2fPair> &pairs,
            Point2f &motionVector, float &err);

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
#endif
private:
    bool isInitialized_;
    int8_t qualThreshold_;
    std::vector<LidarFrame>  frames_;
    //std::vector<Point2fPair> pairList_;

    float estRotateAngle_;
    Point2f estTranslation_;
};

#endif  // __MAP_H__
