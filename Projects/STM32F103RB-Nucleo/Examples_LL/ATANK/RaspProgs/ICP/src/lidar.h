#ifndef __LIDAR_H__
#define __LIDAR_H__

#include <cassert>
#include <vector>
#include <cmath>
#include <cstdint>

class Point2f {
    float x_;
    float y_;
public:
    Point2f(float x, float y) : x_(x), y_(y) {}
    Point2f() : x_(0), y_(0) {}
    void setFromDistAngle(float dist, float angle) {
        x_ = dist * cos((angle / 180.0) * M_PI);
        y_ = dist * sin((angle / 180.0) * M_PI);
    }
    void setXY(float x, float y) {
        x_ = x;
        y_ = y;
    }
    float norm(void) {
        return sqrt(x_*x_ + y_*y_);
    }
    float getX() {return x_;}
    float getY() {return y_;}
    Point2f operator+(const Point2f& b) {
        Point2f a = *this;
        a.x_ += b.x_;
        a.y_ += b.y_;
        return a;
    }
    Point2f operator-(const Point2f& b) {
        Point2f a = *this;
        a.x_ -= b.x_;
        a.y_ -= b.y_;
        return a;
    }
    Point2f operator*(const float& b) {
        Point2f a = *this;
        a.x_ *= b;
        a.y_ *= b;
        return a;
    }
    float distance(const Point2f& a) {
        return sqrt(pow((a.x_ - x_),2) + pow((a.y_ - y_),2));
    }
};

class LidarPoint {
    const uint8_t qual_;
    const float dist_;
    const float angle_;
    Point2f point_;
public:
    LidarPoint(int qual, float dist, float angle) :
        qual_(qual), dist_(dist), angle_(angle) {
        point_.setFromDistAngle(dist, angle);
    }
    const uint8_t getQual() const { return qual_; }
    const float getDistance() const { return dist_; }
    const float getAngle() const { return angle_; }
    Point2f getPoint2f() const { return point_; }
};

class LidarFrame {
    std::vector<LidarPoint> lpoints_;
    Point2f comp_tx_;       // global position
    std::vector<Point2f> qpoints_;   //qualified 2d points
public:
    int getPointSize() { return lpoints_.size(); }
    LidarPoint& getLidarPoint(int i) {
        assert(i < lpoints_.size());
        return lpoints_[i];
    }
    std::vector<Point2f>& getQualPoint2f() {
        return qpoints_;
    }
    void makePoint2fList(int qual) {
        for (auto& lp : lpoints_) {
            if (lp.getQual() >= qual) {
                qpoints_.push_back(lp.getPoint2f());
            }
        }
    }
    void addLidarPoint(LidarPoint p) { lpoints_.push_back(p); }
    std::vector<LidarPoint>& getLidarPoints(void) { return lpoints_; }
    void set_delta_tx(Point2f tx) {
        comp_tx_ = tx;
    }
};

typedef struct _Point2fPair {
    std::pair<Point2f, Point2f> points;
    bool outlier;
    float distance;

    _Point2fPair(Point2f a, Point2f b) : points(a,b), outlier(false), distance(0) {}
} Point2fPair;
#endif
