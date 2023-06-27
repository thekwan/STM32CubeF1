#include "ui.h"
#include "map.h"
#include <iostream>

extern MapManager mapmng;

static float point_scale;
static float point_pos_x;
static float point_pos_y;
static int   pmap_index;
static int   cmap_index;
static int   map_index_step;
static bool  isDisplayCompensatedFrame = false;

#define COLOR_CODE_RED       0
#define COLOR_CODE_GREEN     1
#define COLOR_CODE_BLUE      2
#define COLOR_CODE_YELLOW    3

void initGL() {
    // set "clearing" or background color
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);   // black and opaque
}

std::vector<Point2f> getPointList(
        const std::vector<LidarPoint>& lpoints, 
        const uint8_t qual)
{
    std::vector<Point2f> plist;

    for (auto& p : lpoints) {
        if (p.getQual() > qual) {
            plist.emplace_back(p.getPoint2f());
        }
    }

    return plist;
}

void adjustCoordinate(std::vector<Point2f>& pts, Point2f tx, float scale) {
    for (auto& p : pts) {
        p = tx + p * scale;
    }
}

void addColoredPoints(std::vector<Point2f>& pts, int colorCode) {
    float cval_r = 0;
    float cval_g = 0;
    float cval_b = 0;

    switch(colorCode) {
        case COLOR_CODE_RED:
            cval_r = 1.0;
            break;
        case COLOR_CODE_GREEN:
            cval_g = 1.0;
            break;
        case COLOR_CODE_BLUE:
            cval_b = 1.0;
            break;
        case COLOR_CODE_YELLOW:
            cval_r = 1.0;
            cval_g = 1.0;
            break;
        default:
            break;
    }

    for (auto& p : pts) {
        glColor3f(cval_r, cval_g, cval_b);
        glVertex2f(p.getX(), p.getY());
    }
}


void display() {
    glClear(GL_COLOR_BUFFER_BIT);

    if (!mapmng.isReady()) {
        return;
    }

    Point2f ref(point_pos_x, point_pos_y);
    int8_t qual_thr = mapmng.getQualThreshold();


    std::vector<LidarPoint>& pframe = mapmng.getLidarFramePoints(pmap_index);
    std::vector<LidarPoint>& cframe = mapmng.getLidarFramePoints(cmap_index);

    std::vector<Point2f> ppts = getPointList(pframe, qual_thr);
    std::vector<Point2f> cpts = getPointList(cframe, qual_thr);

    adjustCoordinate(ppts, ref, point_scale);
    adjustCoordinate(cpts, ref, point_scale);

    // paired points
    //std::vector<Point2fPair> *ppair = mapmng.getPointPairs();

    // Define shapes enclosed within a pair of glBegin and glEnd
    glPointSize(2.0);
    glBegin(GL_POINTS);
        addColoredPoints(ppts, COLOR_CODE_GREEN);
        addColoredPoints(cpts, COLOR_CODE_YELLOW);
    glEnd();


    glFlush();
}

void reshape(GLsizei width, GLsizei height) {
    // Compute aspect ratio of the new windows
    if (height == 0) height = 1;
    GLfloat aspect = (GLfloat) width / (GLfloat) height;

    // Set the viewport to cover the new window.
    glViewport(0, 0, width, height);

    // Set the aspect ratio of the clipping area to match the Viewport
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    if (width >= height) {
        // aspect >= 1, set the height from -1 to 1, with larger width
        gluOrtho2D(-1.0 * aspect, 1.0 * aspect, -1.0, 1.0);
    }
    else {
        // aspect < 1, set the width to -1 to 1, with larger height
        gluOrtho2D(-1.0, 1.0, -1.0 / aspect, 1.0 / aspect);
    }
}

void doSpecial(int key, int x, int y) {
    switch(key) {
        case GLUT_KEY_LEFT: 
            point_pos_x -= 0.01;
            break;
        case GLUT_KEY_RIGHT:
            point_pos_x += 0.01;
            break;
        case GLUT_KEY_UP:
            point_pos_y += 0.01;
            break;

        case GLUT_KEY_DOWN:
            point_pos_y -= 0.01;
            break;
    }
    glutPostRedisplay();
}

void doKeyboard(unsigned char key, int x, int y) {
    switch(key) {
        case 'x':
        case 'X':
            point_scale *= 1.1;
            break;
        case 'z':
        case 'Z':
            point_scale /= 1.1;
            break;
        case 'q':
        case 'Q':
            exit(0);
            break;
        case 'u':
        case 'U':
            cmap_index = pmap_index;
            pmap_index = pmap_index - map_index_step;
            if (pmap_index < 0) {
                pmap_index = 0;
            }
            break;

            // Dump point data into files
        case 'd':
        case 'D':
            //mapmng.dumpPointData(map_index-1);  // previous frame
            //mapmng.dumpPointData(map_index);    // current frame
            break;
        case 't':
        case 'T':
            // turn on/off compensated lidar point frames
            isDisplayCompensatedFrame ^= 1;
            break;
        case 'i':
        case 'I':
            int max_index = mapmng.getMapMaxIndex();
            pmap_index = cmap_index;
            cmap_index = cmap_index + map_index_step;
            if (cmap_index >= max_index) {
                cmap_index = max_index-1;
            }

            mapmng.update_delta(cmap_index, pmap_index);

            // check frame distance
            //mapmng.checkFrameDistance(map_index);
            //mapmng.findOptimalTranslation(map_index);
            //mapmng.findOptimalRotation(map_index);
            //mapmng.icpProc(map_index);
            break;
    }
    std::cout << "p,c map index = " << pmap_index << " , " << cmap_index << std::endl;

    glutPostRedisplay();
}

void initOpenGL(int argc, char *argv[]) {
    point_scale = 1/16384.0;
    point_pos_x = 0.0;
    point_pos_y = 0.0;
    map_index_step = 4;
    pmap_index = 0;
    cmap_index = pmap_index + map_index_step;

    glutInit(&argc, argv);
    glutInitWindowSize(640,480);
    glutInitWindowPosition(50,50);
    glutCreateWindow("Viewport Transform");
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(doKeyboard);
    glutSpecialFunc(doSpecial);
    initGL();
    glutMainLoop();
    return;
}


