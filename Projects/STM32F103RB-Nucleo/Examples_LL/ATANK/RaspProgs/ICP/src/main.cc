#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <glog/logging.h>

#include "map.h"
#include "ui.h"

MapManager mapmng("LidarFrames.dat",  150);

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    initOpenGL(argc, argv);

    return 0;
}
