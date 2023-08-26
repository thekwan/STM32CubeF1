#include "map.h"
#include <stdio.h>
#include <iomanip>
#include <float.h>

using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using namespace std;

//#define PI 3.1415926535

char LOGI_buf[1024];
#define LOGI(format, ...) {sprintf(LOGI_buf, format, ##__VA_ARGS__);LOG(INFO) << LOGI_buf;}

//#define CHECK_FRAME_RETURN(index, min, max)   {\
//    if (frame_index < min || frame_index > max) { \
//        return; } }while(0);
//
//#define CHECK_FRAME_RETURN_X(index, min, max, ret)   {\
//    if (frame_index < min || frame_index > max) { \
//        return ret; } }while(0);


MapManager::MapManager(const std::string& fileName, const uint8_t qualThreshold)
    : qualThreshold_(qualThreshold), isInitialized_(false) {
    readLidarDataFromFile(fileName);
}

MapManager::~MapManager(void) {
}

void MapManager::readLidarDataFromFile(const std::string& fname) {
    LOG(INFO) << "read file: '" << fname << "\n";

    std::ifstream ifs;
    ifs.open(fname, std::ios::binary | std::ios::in);
    if (!ifs.is_open()) {
        LOG(FATAL) << "Can't open the file.\n";
    }

    int frame_count = 0;

    while (!ifs.eof()) {
        LidarFrame frame;
        int frame_length;
        uint8_t qual;
        float dist, angle;
        ifs.read((char*) &frame_length, sizeof(int));

        //LOG(INFO) << "frame[" << std::to_string(frame_count++) << "]"
        //    << "[" << std::to_string(frame_length) << "]\n";

        //frame.points_.clear();

        for (int i = 0; i < frame_length; i++) {
            ifs.read((char*)&qual, sizeof(uint8_t));
            ifs.read((char*)&dist, sizeof(float));
            ifs.read((char*)&angle, sizeof(float));

            frame.addLidarPoint(LidarPoint(qual, dist, angle));
        }

        frame.makePoint2fList(qualThreshold_);

        frames_.push_back(frame);
    }

    isInitialized_ = true;

    LOG(INFO) << "Total lidar frames: " << frames_.size() << "\n";
    //printFrameInfo();
}

void MapManager::printFrameInfo(void) {
    LOG(INFO) << "//----------------------------------------";
    LOG(INFO) << "# of LidarFrame  : " << frames_.size();
    LOG(INFO) << "# of LidarPoint  : " << frames_.size();
    for (auto& f : frames_) {
        LOG(INFO) << f.getPointSize();
    }
}

void MapManager::writeLidarFrameData(void) {
    std::ofstream ofs;
    ofs.open("lidar_data.cvs", std::ios::out);

    if (ofs.is_open()) {
        for (auto & frame : frames_) {
            int point_num = frame.getPointSize();
            ofs << point_num << std::endl;
            for (int i = 0; i < point_num; i++) {
                LidarPoint p = frame.getLidarPoint(i);
                ofs << p.getQual() << ", "
                    << p.getDistance() << ", "
                    << p.getAngle() << ", "
                    << p.getPoint2f().getX() << ", "
                    << p.getPoint2f().getY() << std::endl;
            }
        }
    }
    ofs.close();
}

std::vector<LidarPoint>& MapManager::getLidarFramePoints(int frame_index) {
    assert(frame_index < frames_.size());
    return frames_[frame_index].getLidarPoints();
}

void MapManager::update_delta(int cframeIdx, int pframeIdx) {
    auto& cf = frames_[cframeIdx];
    auto& pf = frames_[pframeIdx];

    // Update tx
    Point2f tx_ = estimate_tx(cf, pf);
    //
    // Update Rot
    cf.set_delta_tx(tx_);
}

int MapManager::findClosestPointIndex(
        Point2f& point, std::vector<Point2f>& plist) {
    float min_dist = FLT_MAX;
    int min_index = 0;

    for (int i = 0; i < plist.size(); i++) {
        float dist = point.distance(plist[i]);
        if (dist < min_dist) {
            min_dist = dist;
            min_index = i;
        }
    }

    return min_index;
}

std::vector<std::pair<int,int>> MapManager::getClosestPointPairIndex(
        std::vector<Point2f>& ppts, std::vector<Point2f>& cpts) {
    std::vector<int> pidx;
    std::vector<int> cidx;
    std::vector<std::pair<int, int>> pair_list;

    // get closest point's index for each point
    for (auto& p : ppts) {
        pidx.push_back(findClosestPointIndex(p, cpts));
    }

    for (auto& c : cpts) {
        cidx.push_back(findClosestPointIndex(c, ppts));
    }

    // get mutual closest point pair index
    for (int i = 0; i < cidx.size(); i++) {
        if (pidx[cidx[i]] == i) {
            pair_list.push_back(std::make_pair(i, cidx[i]));
        }
    }

    return pair_list;
}

Point2f MapManager::estimate_tx(LidarFrame& cf, LidarFrame& pf) {
    std::vector<std::pair<int,int>> pair_list;
    Point2f delta_tx_acc = Point2f(0,0);
    std::vector<Point2f> ppts(pf.getQualPoint2f());
    std::vector<Point2f> ppts_org(ppts);
    std::vector<Point2f> cpts(cf.getQualPoint2f());

    for (int i = 0; i < 16; i++) {

        // mutual closest point pair index
        pair_list = getClosestPointPairIndex(ppts, cpts);
        //LOG(INFO) << "Closest Point pair list # = " << pair_list.size();

        // get average distance of closest points
        float dist_sum = 0;
        Point2f delta_tx = Point2f(0, 0);
        for (auto& pair : pair_list) {
            dist_sum += cpts[pair.first].distance(ppts[pair.second]);
            delta_tx = delta_tx + (cpts[pair.first] - ppts[pair.second]);
        }
        dist_sum /= pair_list.size();
        delta_tx /= (float)(pair_list.size());

        //LOG(INFO) << "Total minimum dist = " << dist_sum;
        //LOG(INFO) << "Delta tx           = " << delta_tx.getX() << " , " 
        //    << delta_tx.getY();
        LOG(INFO) << "pair.#, min_dist, tx_: " << pair_list.size() << "\t"
            << dist_sum << "\t" << delta_tx.getX() << "\t" << delta_tx.getY();

        // compensates the translation movement.
        delta_tx *= 0.6;
        delta_tx_acc = delta_tx_acc + delta_tx;
        for (int i = 0; i < ppts.size(); i++) {
            ppts[i] = ppts_org[i] + delta_tx_acc;
        }
    }
    LOG(INFO) << "end";

    return delta_tx_acc;
}

#if 0
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
        ofp << "[qual]\t[angle]\t[dist]\t[cx]\t[cy]\n";

        for (auto &p : frames_[frame_index].points_) {
            ofp << p.qual << "\t" << p.angle << "\t" << p.dist << "\t"
                << p.point.x << "\t" << p.point.y << std::endl;
        }
        ofp.close();

        LOG(INFO) << "dump point data #" << frame_index << std::endl;
    }

    return;
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

    /* remove outlier pair by checking global motion vector.
     */
    // i) get global motion vector.
    Point2f gmv(0,0);
    std::vector<Point2f> mv;
    for (auto &pair : plist) {
        mv.emplace_back((pair.points.first.x - pair.points.second.x), 
                (pair.points.first.y - pair.points.second.y));
        gmv.x += mv.back().x;
        gmv.y += mv.back().y;
    }
    gmv.x /= plist.size();
    gmv.y /= plist.size();

    // ii) remove the pair having more large mag than gmv.
    //std::cout << "gmv = " << gmv.x << "," << gmv.y << std::endl;
    int size = plist.size() - 1;
    for (int i = size; i >= 0; i--) {
        //std::cout << "dist = " << calcNormDist(mv[i], gmv) << std::endl;
        if (calcNormDist(mv[i], gmv) > gmv.norm()*100) {
            plist.erase(plist.begin()+i);
        }
    }
    

    LOG(INFO) << "pPoint vs. cPoints # : " 
        << pPoints.size() << "," << cPoints.size();
    LOG(INFO) << "findClosestPoints finds corr: " << plist.size();

    return plist;
}

void DEBUG_display_LidarPoints(std::vector<LidarPoint> &a) {
    LOG(INFO) << "# display Lidar Point Frame : " << a.size();
    for (int i = 0; i < a.size(); i++) {
        LOG(INFO) << "[" << i << "] " << a[i].qual << "\t"
            << a[i].angle << "\t" << a[i].dist;
    }
}

void MapManager::getFrameDistance( std::vector<LidarPoint> &a, 
        std::vector<LidarPoint> &b, int offset, float *dist, float *angle)
{
    *dist = 0;
    *angle = 0;
    int count = 0;
    for (int i = 0; i < b.size(); i++) {
        if (a[i+offset].qual > qualThreshold_ && b[i].qual > qualThreshold_) {
            *dist += fabs(a[i+offset].dist - b[i].dist);
            *angle += a[i+offset].angle - b[i].angle;
            count++;
        }
    }

    *dist /= count;
    *angle /= count;
}

Point2f MapManager::getCentroidOfPoints(std::vector<LidarPoint> &pts) {
    Point2f cent;
    int cnt = 0;

    for (auto &p : pts) {
        if (p.qual >= qualThreshold_) {
            cent.x += p.point.x;
            cent.y += p.point.y;
            cnt++;
            //LOG(INFO) << p.point.x << "\t" << p.point.y;
        }
    }

    cent.x /= cnt;
    cent.y /= cnt;

    //LOG(INFO) << "count = " << cnt;

    return cent;
}

void MapManager::removeOutlierFromPairList(
        std::vector<Point2fPair> &pairs, float thr_scale) {
    float global_dist = 0;

    // calculate pair distance and global distance
    for (auto &pair: pairs) {
        pair.distance = calcNormDist(pair.points.first, pair.points.second);
        global_dist += pair.distance;
    }
    global_dist /= pairs.size();


    // sets flag for outlier
    float dist_threshold = global_dist * thr_scale;
    int outlier_count = 0;
    for (auto &pair : pairs) {
        if (pair.distance > dist_threshold) {
            pair.outlier = true;
            outlier_count++;
        }
    }

#if 0
    // DEBUG: check the result
    LOG(INFO) << "RemoveOutlierFromPairList::";
    LOG(INFO) << "global distance and threshold: " << global_dist 
        << ", " << dist_threshold;
    for (auto &pair : pairs) {
        LOG(INFO) << pair.distance << "\t" << pair.outlier;
    }
    LOG(INFO) << "outlier = " << outlier_count << " / " << pairs.size() 
        << " (" << (float)outlier_count / pairs.size() << ")";
#endif
}

void MapManager::getTranslationFromPair(std::vector<Point2fPair> &pairs,
        Point2f &motionVector, float &estError) {
    Point2f tls;
    int tls_count = 0;

    // get motion vector from centroid difference of each frame.
    for (auto &pair : pairs) {
        if (!pair.outlier) {
            tls.x += pair.points.first.x - pair.points.second.x;
            tls.y += pair.points.first.y - pair.points.second.y;
            tls_count++;
        }
    }
    tls.x /= tls_count;
    tls.y /= tls_count;

    // get estimate error from difference variance of paired points
    float var = 0;
    for (auto &pair : pairs) {
        if (!pair.outlier) {
            float dfx = (pair.points.first.x - pair.points.second.x - tls.x);
            float dfy = (pair.points.first.y - pair.points.second.y - tls.y);
            var += (dfx * dfx) + (dfy * dfy);
        }
    }
    var /= tls_count;

    motionVector = tls;
    estError = var;
}

void MapManager::findOptimalTranslation(int frame_index) {
    CHECK_FRAME_RETURN(frame_index, 1, getMapMaxIndex());
    auto &pf = frames_[frame_index-1];  // previous frame
    auto &cf = frames_[frame_index];    // current frame
    Point2f trMv;   // Translation motion vector estimate
    float trErr;    // Translation estimate error

    // Get pairlist 
    pairList_ = findAngleMatchedPoints(pf.points_,cf.points_);
    //pairList_ = findClosestPoints(pf.points_,cf.points_);
    removeOutlierFromPairList(pairList_, 2.0);
    getTranslationFromPair(pairList_, trMv, trErr);

    //LOG(INFO) << "Translation = " 
    //    << setprecision(1) << setw(6) << setfill(' ') << mv.x << " , " 
    //    << setprecision(1) << setw(6) << setfill(' ') << mv.y;
    LOGI("Translation = %5.1f , %5.1f\t%5.1f", trMv.x, trMv.y, trErr);

#if 0   // centroid difference (inaccurate)
    Point2f centP, centC;
    centP = getCentroidOfPoints(pf.points_);
    centC = getCentroidOfPoints(cf.points_);
    LOG(INFO) << "centroid diff(" << centC.x - centP.x << " , " 
        << centC.y - centP.y << ")";
#endif
}

void MapManager::findOptimalRotation(int frame_index)  {
    getDistBasedRotationAngle(frame_index);
    estRotateAngle_ = getIterativeBasedRotationAngle(frame_index);
}

float MapManager::calcErrorGivenAngle(
        LidarFrame &prev, LidarFrame &curr, float angle) {
    std::vector<LidarPoint> pa, pb;
    int qualThr = qualThreshold_;

    // select only high measurement quality points
    for (auto &a : prev.points_) {
        if (a.qual > qualThr) {
            pa.push_back(a);
        }
    }
    // select only high measurement quality points w/ angle rotation
    for (auto &a : curr.points_) {
        if (a.qual > qualThr) {
            // update angle and x, y coordinate value
            float new_angle = a.angle + angle;
            pb.emplace_back(a.qual, a.dist, new_angle,
               a.dist*cos((new_angle/180.0)*PI), a.dist*sin((new_angle/180.0)*PI));
        }
    }

    std::vector<Point2fPair> pair = findClosestPoints(pa, pb);
    return calcNormDist(pair) * (pa.size() / pair.size());
}

float MapManager::getIterativeBasedRotationAngle(int frame_index)  {
    CHECK_FRAME_RETURN_X(frame_index, 1, getMapMaxIndex(), 0);
    auto &pf = frames_[frame_index-1];  // previous frame
    auto &cf = frames_[frame_index];    // current frame

    float minEstError = 0, estErrorThr = 0.1;
    float angle = 10;   // initial angle degree
    float angle_acc = 0;
    int repeat_num = 5;
    float currError = calcErrorGivenAngle(pf, cf, 0);
    for (int i = 0; i < repeat_num; i++) {
        // try turn left
        float leftTryError = calcErrorGivenAngle(pf, cf, angle_acc - angle);
        float rightTryError = calcErrorGivenAngle(pf, cf, angle_acc + angle);

        if (currError < leftTryError && currError < rightTryError) {
            minEstError = currError;
        }
        else if (leftTryError < rightTryError) {
            angle_acc -= angle;
            minEstError = leftTryError;
        }
        else {
            angle_acc += angle;
            minEstError = rightTryError;
        }
        angle /= 2.0;

        LOGI("estErr[angle](left,zero,right) = [%.0f, %.0f](%.1f,%.1f,%.1f)", 
                angle_acc, angle, leftTryError, currError, rightTryError);

        if (minEstError < estErrorThr) {
            break;
        }
    }

    return angle_acc;
}

void MapManager::getDistBasedRotationAngle(int frame_index)  {
    CHECK_FRAME_RETURN(frame_index, 1, getMapMaxIndex());
    auto &pf = frames_[frame_index-1];  // previous frame
    auto &cf = frames_[frame_index];    // current frame

    //LOG(INFO) << "point# = " << pf.points_.size() << " " << cf.points_.size();

    std::vector<LidarPoint> prev, curr;
    int point_num_diff = pf.points_.size() - cf.points_.size();
    int rSearchRange= 15;

    //DEBUG_display_LidarPoints(pf.points_);
    //DEBUG_display_LidarPoints(cf.points_);

    if (point_num_diff >= 0) {
        prev.resize((int)pf.points_.size() + (rSearchRange * 2));
        // copy last points of the frame
        std::copy(pf.points_.end()-rSearchRange, pf.points_.end(), prev.begin());
        // copy frame points
        std::copy(pf.points_.begin(), pf.points_.end(), prev.begin() + rSearchRange);
        // copy first points of the frame
        std::copy(pf.points_.begin(), pf.points_.begin()+rSearchRange, 
                prev.begin() + (rSearchRange + pf.points_.size()));
    }
    else {
        prev.resize((int)pf.points_.size() + (rSearchRange * 2) - point_num_diff);
        // copy last points of the frame
        std::copy(pf.points_.end()-rSearchRange, pf.points_.end(), prev.begin());
        // copy frame points
        std::copy(pf.points_.begin(), pf.points_.end(), prev.begin()+rSearchRange);
        // copy first points of the frame
        std::copy(pf.points_.begin(), 
             pf.points_.begin() + (rSearchRange - point_num_diff) , 
             prev.begin() + (rSearchRange + pf.points_.size()));
    }

    //DEBUG_display_LidarPoints(prev);
    std::vector<float> angle, dist;
    angle.resize(rSearchRange*2 + 1);
    dist.resize(rSearchRange*2 + 1);
    for (int i = 0; i <= rSearchRange*2; i++) {
        getFrameDistance(prev, cf.points_, i, &dist[i], &angle[i]);
        //LOG(INFO) << "dist[" << i << "] = " << dist << "\t" //    << angle;
    }

    int minDistIdx = std::min_element(dist.begin(), dist.end()) - dist.begin();

    LOGI("Rotation    = %5.1f' estErr = %5.1f", angle[minDistIdx], dist[minDistIdx]);
}

void MapManager::checkFrameDistance(int frame_index)  {
    CHECK_FRAME_RETURN(frame_index, 1, getMapMaxIndex());
    auto &pf = frames_[frame_index-1];  // previous frame
    auto &cf = frames_[frame_index];    // current frame

    //LOG(INFO) << "prevPts, currPts = [" << pf.points_.size() << " , "
    //    << cf.points_.size() << "]" << "\t";
    //    << pf.points_[0].angle << "," << cf.points_[0].angle;
    LOG(INFO) << frame_index << "\t\t";

    // find closest angle point for given angle(start angle of current)
    int angleOffset;
    float normDist;
    
    //pairList_ = findAngleMatchedPoints(pf.points_,cf.points_);
    pairList_ = findClosestPoints(pf.points_,cf.points_);
    LOG(INFO) << "pairList_.size = " << pairList_.size() << "\t";
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

    LOG(INFO) << "Points(p,c) = " << pPoints.size() << " , " << cPoints.size();

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
        LOG(INFO) << a.x << " , " << a.y << "\t\t"
                  << b.x << " , " << b.y;
    }

    LOG(INFO) << "[2x2] = [" << ma << " , " << mb;
    LOG(INFO) << "         " << mc << " , " << md << "]";

    MatrixXf a(2,2);
    a(0,0) = ma;
    a(0,1) = mb;
    a(1,0) = mc;
    a(1,1) = md;

    JacobiSVD<MatrixXf> svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
    //LOG(INFO) << svd.singularValues();
    //LOG(INFO) << "matrix U : " << svd.matrixU();
    //LOG(INFO) << "matrix V : " << svd.matrixV();
    
    MatrixXf rotateMatrix = svd.matrixV() * svd.matrixU().transpose();
    LOG(INFO) << "Rotation Matrix = " << rotateMatrix;
    LOG(INFO) << "angle = " 
        << (atan2(rotateMatrix(0,0), rotateMatrix(1,0)) / PI) * 180;
}

//void MapManager::calcSVD2x2(k

float MapManager::calcNormDist(std::vector<Point2fPair> &ppair) {
    float dist = 0;
    //LOG(INFO) << "##### = " << fa[0].angle << "," << offset;
    //LOG(INFO) << "angle = " << fa[offset].angle << "," << fb[0].angle;
    //LOG(INFO) << "angle = " << fa[offset+1].angle << "," << fb[1].angle;
    //LOG(INFO) << "angle = " << fa[offset+2].angle << "," << fb[2].angle;
    //LOG(INFO) << "angle = " << fa[offset+3].angle << "," << fb[3].angle;
    for (auto &pair : ppair) {
        dist += calcNormDist(pair.points.first, pair.points.second);
    }

    LOG(INFO) << "NormDistFrame = " << dist/1e3 << "\t";
    return dist;
}

float MapManager::calcNormDist(Point2f &a, Point2f &b) {
    return sqrt(pow((a.x - b.x),2) + pow((a.y - b.y),2));
}

float MapManager::calcAngleDist(LidarPoint &a, LidarPoint &b) {
    return a.angle - b.angle;
}
#endif
