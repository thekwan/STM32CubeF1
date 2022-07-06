#include "ui.h"
#include "map.h"

extern MapManager mapmng;

static float point_scale;
static float point_pos_x;
static float point_pos_y;
static int   map_index;

void initGL() {
    // set "clearing" or background color
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);   // black and opaque
}

void display() {
    glClear(GL_COLOR_BUFFER_BIT);

    if (!mapmng.isReady()) {
        return;
    }

    int pPointsNum = 0;
    int cPointsNum = 0;

    int8_t qual_thr = mapmng.getQualThreshold();
    LidarFrame *pframe = mapmng.getLidarFrame(map_index-1);
    LidarFrame *cframe = mapmng.getLidarFrame(map_index);

    Point2f centP = mapmng.getCentroidOfPoints(pframe->points_);
    Point2f centC = mapmng.getCentroidOfPoints(cframe->points_);

    std::cout << centP.x << " , " << centP.y << std::endl;
    std::cout << centC.x << " , " << centC.y << std::endl;

    // paired points
    std::vector<Point2fPair> *ppair = mapmng.getPointPairs();

    // Define shapes enclosed within a pair of glBegin and glEnd
    glPointSize(2.0);
    glBegin(GL_POINTS);
        // Display points of previous frame
        glColor3f(0.0f, 1.0f, 0.0f);    // Green
        for(auto &a : pframe->points_) {
            if (a.qual > qual_thr) {
                glVertex2f(point_pos_x + (a.point.x - centP.x) * point_scale , 
                        point_pos_y + (a.point.y - centP.y) * point_scale );
                pPointsNum++;
            }
        }

        // Display points of current frame
        glColor3f(1.0f, 1.0f, 0.0f);    // Yellow
        for(auto &a : cframe->points_) {
            if (a.qual > qual_thr) {
                glVertex2f(point_pos_x + (a.point.x - centC.x) * point_scale , 
                        point_pos_y + (a.point.y - centC.y) * point_scale );
                cPointsNum++;
            }
        }

        // Display all low quality points
        glColor3f(0.0f, 0.0f, 1.0f);    // Blue
        for(auto &a : pframe->points_) {
            if (a.qual <= qual_thr) {
                glVertex2f(point_pos_x + (a.point.x  - centP.x) * point_scale,
                        point_pos_y + (a.point.y  - centP.y) * point_scale);
            }
        }

        glColor3f(1.0f, 0.0f, 0.0f);    // Red
        for(auto &a : cframe->points_) {
            if (a.qual <= qual_thr) {
                glVertex2f(point_pos_x + (a.point.x  - centC.x) * point_scale,
                        point_pos_y + (a.point.y  - centC.y) * point_scale);
            }
        }
    glEnd();

    glBegin(GL_LINES);
        glColor3f(1.0f, 0.0f, 0.0f);    // Red
        for(auto &a : *ppair) {
            glVertex2f(point_pos_x + a.first.x * point_scale, point_pos_y + a.first.y * point_scale);
            glVertex2f(point_pos_x + a.second.x * point_scale, point_pos_y + a.second.y * point_scale);
        }
    glEnd();

    glEnd();


    glFlush();

    std::cout << "high quality points(prev,curr) = (" << pPointsNum << " , "
              << cPointsNum << ")" << std::endl;
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
            map_index--;
            if (map_index < 0) {
                map_index = 0;
            }
            break;

            // Dump point data into files
        case 'd':
        case 'D':
            mapmng.dumpPointData(map_index-1);  // previous frame
            mapmng.dumpPointData(map_index);    // current frame
            break;
        case 'i':
        case 'I':
            int max_index = mapmng.getMapMaxIndex();
            map_index++;
            if (map_index >= max_index) {
                map_index = max_index-1;
            }

            // check frame distance
            //mapmng.checkFrameDistance(map_index);
            mapmng.findOptimalTranslation(map_index);
            mapmng.findOptimalRotation(map_index);
            //mapmng.icpProc(map_index);
            break;
    }
    glutPostRedisplay();
}

void initOpenGL(int argc, char *argv[]) {
    point_scale = 1/16384.0;
    point_pos_x = 0.0;
    point_pos_y = 0.0;
    map_index = 0;

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


