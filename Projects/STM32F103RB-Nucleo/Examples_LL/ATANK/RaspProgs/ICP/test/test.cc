
#include "test.h"
#include "map.h"

extern MapManager mapmng;

void TEST_STEP_ERROR(void) {
    const int map_index_step = 4;
    const int map_max_index = mapmng.getMapMaxIndex();

    std::ofstream ofs("frame_error_stat.txt", std::ios::out);
    if (!ofs.is_open()) {
        LOG(FATAL) << "Can't open file 'frame_error_stat.txt";
    }

    for (int i = map_index_step; i < map_max_index; ++i) {
        int pmap_index = i - map_index_step;
        int cmap_index = i;

        mapmng.update_delta(pmap_index, cmap_index);

        LidarFrame& pframe = mapmng.getLidarFrame(pmap_index);
        LidarFrame& cframe = mapmng.getLidarFrame(cmap_index);

        std::vector<Point2f> ppts = pframe.getPoint2f();
        std::vector<Point2f> cpts = cframe.getPoint2f();

        // compensate points
        for (auto& p : ppts) {
            p = p + pframe.get_delta_tx();
        }

        // calculates error
        ofs << "# FRAME " << i << std::endl;
        ofs << "# p2c " << ppts.size() << std::endl;
        for (auto& p : ppts) {
            // find min-distance point
            float min_dist = FLT_MAX;
            std::vector<float> err_;
            for (auto& c : cpts) {
                float dist = p.distance(c);
                if (min_dist > dist) {
                    min_dist = dist;
                }
            }
            ofs << min_dist << std::endl;
        }

        ofs << "# c2p " << cpts.size() << std::endl;
        for (auto& c : cpts) {
            // find min-distance point
            float min_dist = FLT_MAX;
            for (auto& p : ppts) {
                float dist = c.distance(p);
                if (min_dist > dist) {
                    min_dist = dist;
                }
            }
            ofs << min_dist << std::endl;
        }
        std::cout << "Frame [" << i << "]\r";
    }
    std::cout << std::endl;

    ofs.close();
}
