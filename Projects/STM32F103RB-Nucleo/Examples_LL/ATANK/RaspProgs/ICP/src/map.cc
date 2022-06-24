#include "map.h"

using Eigen::MatrixXf;
using Eigen::JacobiSVD;

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

    qualThreshold_ = 40;
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
                << p.point.x << "\t" << p.point.y << std::endl;
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
    auto ib = fB.begin();

    float angleDist;
    float angleDistMax;

    if (angleDist > 0) {
        for (; ia != fA.end() && ib != fB.end(); ia++) {
            angleDistMax = fabs(calcAngleDist(*ia, *ib));
            for (; ib != fB.end(); ib++) {
                angleDist = calcAngleDist(*ia, *ib);
                if (fabs(angleDist) <= angleDistMax) {
                    angleDistMax = fabs(angleDist);
                }
                else {
                    if (ib == fB.begin()) {
                        break;
                    }
                    ib--;
                    if (ia->qual > qualThreshold_ && ib->qual > qualThreshold_) {
                        plist.emplace_back(Point2fPair(ia->point, ib->point));
                    }
                    break;
                }
            }
        }
    }
    else {
        for (; ia != fA.end() && ib != fB.end(); ib++) {
            angleDistMax = fabs(calcAngleDist(*ia, *ib));
            for (; ia != fA.end(); ia++) {
                angleDist = calcAngleDist(*ib, *ia);
                if (fabs(angleDist) <= angleDistMax) {
                    angleDistMax = fabs(angleDist);
                }
                else {
                    if (ia == fA.begin()) {
                        break;
                    }
                    ia--;
                    if (ia->qual > qualThreshold_ && ib->qual > qualThreshold_) {
                        plist.emplace_back(Point2fPair(ia->point, ib->point));
                    }
                    break;
                }
            }
        }
    }

    return plist;
}

std::vector<Point2fPair> MapManager::findClosestPoints(
        std::vector<LidarPoint> &fA, std::vector<LidarPoint> &fB) {
    std::vector<Point2fPair> plist;
    std::vector<Point2f> pPoints = getPointsGoodQual(fA);
    std::vector<Point2f> cPoints = getPointsGoodQual(fB);
    std::vector<int> pIndex(pPoints.size());
    std::vector<int> cIndex(cPoints.size());

    /* Find closest points from fA's points.
     */
    int index_a = 0;
    for (auto &a : pPoints) {
        float distMax = calcNormDist(a, cPoints[0]);
        Point2f pointMax = cPoints[0];
        int index_b = 0, indexMax = 0;
        for (auto &b : cPoints) {
            float dist = calcNormDist(a, b);
            if (dist < distMax) {
                distMax = dist;
                indexMax = index_b;
            }
            index_b++;
        }
        pIndex[index_a++] = indexMax;
    }

    /* Find closest points from fB's points.
     */
    index_a = 0;
    for (auto &a : cPoints) {
        float distMax = calcNormDist(a, pPoints[0]);
        Point2f pointMax = pPoints[0];
        int index_b = 0, indexMax = 0;
        for (auto &b : pPoints) {
            float dist = calcNormDist(a, b);
            if (dist < distMax) {
                distMax = dist;
                indexMax = index_b;
            }
            index_b++;
        }
        cIndex[index_a++] = indexMax;
    }

    /* Selects correspoding points
     */
    for (int i = 0; i < cPoints.size(); i++) {
        if (pIndex[cIndex[i]] == i) {
            plist.emplace_back(cPoints[i], pPoints[cIndex[i]]);
        }
    }

    std::cout << "pPoint vs. cPoints # : " 
        << pPoints.size() << "," << cPoints.size() << std::endl;
    std::cout << "findClosestPoints finds corr: " << plist.size() << std::endl;

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
    
    //pairList_ = findAngleMatchedPoints(pf.points_,cf.points_);
    pairList_ = findClosestPoints(pf.points_,cf.points_);
    std::cout << "pairList_.size = " << pairList_.size() << "\t";
    normDist = calcNormDist(pairList_);
}

std::vector<Point2f> MapManager::getPointsGoodQual(std::vector<LidarPoint> &fr) {
    std::vector<Point2f> plist;

    for (auto &pt : fr) {
        if (pt.qual >= qualThreshold_) {
            plist.emplace_back(pt.point);
        }
    }

    return plist;
}

std::vector<Point2fPair>* MapManager::getPointPairs(void) {
    return &pairList_;
}

void MapManager::icpProc(int map_index) {
    if (map_index == 0) { return; }
    LidarFrame *pframe = getLidarFrame(map_index-1);
    LidarFrame *cframe = getLidarFrame(map_index);
    
    std::vector<Point2f> pPoints = getPointsGoodQual(pframe->points_);
    std::vector<Point2f> cPoints = getPointsGoodQual(cframe->points_);

    int minPointsNum = (pPoints.size() < cPoints.size() ?
            pPoints.size() : cPoints.size());

    std::cout << "Points(p,c) = " << pPoints.size() << " , " << cPoints.size()
        << std::endl;

    /* calculate centroid
     */
    Point2f pCent, cCent;
    for (int i = 0; i < minPointsNum; i++) {
        pCent.x += pPoints[i].x;
        pCent.y += pPoints[i].y;
        cCent.x += cPoints[i].x;
        cCent.y += cPoints[i].y;
    }
    pCent.x /= minPointsNum;
    pCent.y /= minPointsNum;
    cCent.x /= minPointsNum;
    cCent.y /= minPointsNum;
    
    double ma, mb, mc, md;
    ma = mb = mc = md = 0;

    /* Get X^T * W * Y
     * temp: W = Identity matrix temporarily
     */
    //std::vector<Point2f> ppts, cpts;  // DEBUG ONLY
    auto pp = pPoints.begin();
    auto cc = cPoints.begin();
    for (; pp != pPoints.end() && cc != cPoints.end(); pp++, cc++) {
        ma += (pp->x - pCent.x) * (cc->x - cCent.x);
        mb += (pp->x - pCent.x) * (cc->y - cCent.y);
        mc += (pp->y - pCent.y) * (cc->x - cCent.x);
        md += (pp->y - pCent.y) * (cc->y - cCent.y);

        //ppts.emplace_back(pp->x - pCent.x, pp->y - pCent.y);
        //cpts.emplace_back(cc->x - cCent.x, cc->y - cCent.y);
        Point2f a(pp->x - pCent.x, pp->y - pCent.y);
        Point2f b(cc->x - cCent.x, cc->y - cCent.y);
        std::cout << a.x << " , " << a.y << "\t\t";
        std::cout << b.x << " , " << b.y << "\n";
    }

    std::cout << "[2x2] = [" << ma << " , " << mb << std::endl;
    std::cout << "         " << mc << " , " << md << "]" << std::endl;

    MatrixXf a(2,2);
    a(0,0) = ma;
    a(0,1) = mb;
    a(1,0) = mc;
    a(1,1) = md;

    JacobiSVD<MatrixXf> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
    //std::cout << svd.singularValues();
    //std::cout << "matrix U : " << svd.matrixU() << std::endl;
    //std::cout << "matrix V : " << svd.matrixV() << std::endl;
    
    MatrixXf rotateMatrix = svd.matrixV() * svd.matrixU().transpose();
    std::cout << "Rotation Matrix = " << rotateMatrix << std::endl;
    std::cout << "angle = " << (atan2(rotateMatrix(0,0), rotateMatrix(1,0)) / PI) * 180 << std::endl;
}

//void MapManager::calcSVD2x2(k

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
