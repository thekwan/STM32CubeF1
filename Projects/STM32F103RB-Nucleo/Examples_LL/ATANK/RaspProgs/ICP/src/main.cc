#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <glog/logging.h>

#include "map.h"
#include "ui.h"
#include "test.h"

#define ENABLE_GUI_MODE

MapManager mapmng("LidarFrames.dat",  150);

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

#if defined(ENABLE_GUI_MODE)
    initOpenGL(argc, argv);
#else
    TEST_STEP_ERROR();
#endif

    return 0;
}
