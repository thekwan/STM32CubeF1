#include <iostream>
#include <fstream>
#include <stdlib.h>

#include "map.h"
#include "ui.h"

MapManager mapmng("LidarFrames.dat");

int main(int argc, char *argv[]) {
    initOpenGL(argc, argv);

    return 0;
}
