#ifndef __LIDAR_H__
#define __LIDAR_H__

#include <glog/logging.h>
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
    Point2f operator/=(const float b) {
        this->x_ /= b;
        this->y_ /= b;
        return *this;
    }
    Point2f operator*=(const float b) {
        this->x_ *= b;
        this->y_ *= b;
        return *this;
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
    Point2f rotate(const float angle) {
        float theta = (angle / 180.0) * M_PI;
        float x = x_ * cos(theta) - y_ * sin(theta);
        float y = x_ * sin(theta) + y_ * cos(theta);
        return Point2f(x, y);
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
    Point2f tx_to_prevframe_;       // translation to previous frame
    float rot_to_prevframe_;       // rotation angle to previous frame
    std::vector<Point2f> qpoints_;   // qualified 2d points
    std::vector<Point2f> points_;    // non-qualified 2d points
public:
    int getPointSize() { return lpoints_.size(); }
    LidarPoint& getLidarPoint(int i) {
        assert(i < lpoints_.size());
        return lpoints_[i];
    }
    std::vector<Point2f>& getQualPoint2f() {
        return qpoints_;
    }
    std::vector<Point2f>& getPoint2f() {
        return points_;
    }
    void makePoint2fList(uint8_t qual) {
        for (auto& lp : lpoints_) {
            points_.push_back(lp.getPoint2f());
            if (lp.getQual() >= qual) {
                qpoints_.push_back(lp.getPoint2f());
            }
        }
    }
    void addLidarPoint(LidarPoint p) { lpoints_.push_back(p); }
    std::vector<LidarPoint>& getLidarPoints(void) { return lpoints_; }
    void set_delta_tx(Point2f tx) {
        tx_to_prevframe_ = tx;
    }
    Point2f get_delta_tx(void) {
        return tx_to_prevframe_;
    }
    void set_delta_rot(float angle) {
        rot_to_prevframe_ = angle;
    }
    float get_delta_rot(void) {
        return rot_to_prevframe_;
    }
};

typedef struct _Point2fPair {
    std::pair<Point2f, Point2f> points;
    bool outlier;
    float distance;

    _Point2fPair(Point2f a, Point2f b) : points(a,b), outlier(false), distance(0) {}
} Point2fPair;
#endif
