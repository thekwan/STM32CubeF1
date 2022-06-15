#include "map.h"

#define PI 3.1415926535

#define CHECK_FRAME_RETURN(index, min, max)   {\
    if (frame_index < min || frame_index > max) { \
        return; } }while(0);

MapManager::MapManager(std::string fileName) 
    : qualThreshold_(10), isInitialized_(false)
{
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
                    dist*cos((angle/180.0)*PI), dist*sin((angle/180.0)*PI));
        }

        frames_.push_back(frame);
    }

    qualThreshold_ = 60;
    isInitialized_ = true;

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

void MapManager::dumpPointData(int frame_index) {
    CHECK_FRAME_RETURN(frame_index, 0, getMapMaxIndex());
    std::string filename("frame#"+std::to_string(frame_index)+".dat");
    std::ofstream ofp;

    ofp.open(filename, std::ios::out);

    if (ofp.is_open()) {
        ofp << "frame_index : " << frame_index << std::endl;
        ofp << "[qual]\t[angle]\t[cx]\t[cy]\n";

        for (auto &p : frames_[frame_index].points_) {
            ofp << p.qual << "\t" << p.angle << "\t"
                << p.cx << "\t" << p.cy << std::endl;
        }
        ofp.close();

        std::cout << "[INFO] dump point data #" << frame_index << std::endl;
    }

    return;
}

int8_t MapManager::getQualThreshold(void) {
    return qualThreshold_;
}

std::vector<Point2fPair> MapManager::findAngleMatchedPoints(
        std::vector<LidarPoint> &fA, std::vector<LidarPoint> &fB) {
    std::vector<Point2fPair> plist;
    auto ia = fA.begin();
    auto ib = fA.begin();

    float angleDist = calcAngleDist(*ia, *ib);
    float angleDistMax = fabs(angleDist);

    if (angleDist > 0) {
        for (; ia != fA.end() && ib != fB.end(); ia++) {
            angleDist = calcAngleDist(*ia, *ib);
            for (; ib != fB.end(); ib++) {
                if (fabs(angleDist) < angleDistMax) {
                    angleDistMax = fabs(angleDist);
                }
                else {
                    ib--;
                    plist.emplace_back(Point2fPair(
                        Point2f(ia->cx, ia->cy), Point2f(ib->cx, ib->cy)));
                    break;
                }
            }
        }
    }
    else {
        for (; ia != fA.end() && ib != fB.end(); ib++) {
            angleDist = calcAngleDist(*ib, *ia);
            for (; ia != fA.end(); ia++) {
                if (fabs(angleDist) < angleDistMax) {
                    angleDistMax = fabs(angleDist);
                }
                else {
                    ia--;
                    plist.emplace_back(Point2fPair(
                        Point2f(ia->cx, ia->cy), Point2f(ib->cx, ib->cy)));
                    break;
                }
            }
        }
    }

    return plist;
}

void MapManager::checkFrameDistance(int frame_index)  {
    CHECK_FRAME_RETURN(frame_index, 1, getMapMaxIndex());
    auto &pf = frames_[frame_index-1];  // previous frame
    auto &cf = frames_[frame_index];    // current frame

    //std::cout << "prevPts, currPts = [" << pf.points_.size() << " , "
    //    << cf.points_.size() << "]" << "\t";
    //std::cout << pf.points_[0].angle << "," << cf.points_[0].angle << std::endl;
    std::cout << frame_index << "\t\t";

    // find closest angle point for given angle(start angle of current)
    int angleOffset;
    float normDist;
    std::vector<Point2fPair> pairList;
    
    pairList = findAngleMatchedPoints(pf.points_,cf.points_);
    normDist = calcNormDist(pairList);
}

float MapManager::calcNormDist(std::vector<Point2fPair> &ppair) {
    float dist = 0;
    //std::cout << "##### = " << fa[0].angle << "," << offset << std::endl;
    //std::cout << "angle = " << fa[offset].angle << "," << fb[0].angle << std::endl;
    //std::cout << "angle = " << fa[offset+1].angle << "," << fb[1].angle << std::endl;
    //std::cout << "angle = " << fa[offset+2].angle << "," << fb[2].angle << std::endl;
    //std::cout << "angle = " << fa[offset+3].angle << "," << fb[3].angle << std::endl;
    for (auto &pair : ppair) {
        dist += calcNormDist(pair.first, pair.second);
    }

    std::cout << "NormDistFrame = " << dist/1e3 << "\t" << std::endl;
    return dist;
}

float MapManager::calcNormDist(Point2f &a, Point2f &b) {
    return sqrt(pow((a.x - b.x),2) + pow((a.y - b.y),2));
}

float MapManager::calcAngleDist(LidarPoint &a, LidarPoint &b) {
    return a.angle - b.angle;
}
