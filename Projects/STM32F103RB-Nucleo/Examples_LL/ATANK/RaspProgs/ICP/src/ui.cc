#include "ui.h"
#include "map.h"

extern MapManager mapmng;

static float point_scale;
static float point_pos_x;
static float point_pos_y;
static int   map_index;
static bool  isDisplayCompensatedFrame = false;

#define GL_SET_COLOR_RED     glColor3f(1.0f, 0.0f, 0.0f)
#define GL_SET_COLOR_GREEN   glColor3f(0.0f, 1.0f, 0.0f)
#define GL_SET_COLOR_BLUE    glColor3f(0.0f, 0.0f, 1.0f)
#define GL_SET_COLOR_YELLOW  glColor3f(1.0f, 1.0f, 0.0f)

void initGL() {
    // set "clearing" or background color
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);   // black and opaque
}

//#define DEBUG_TRANSLATION_COMPENSATE
//#define DEBUG_SHOW_ANGLE_MATCHED_OUTLIER

//#define PI 3.1415926535

void display() {
    glClear(GL_COLOR_BUFFER_BIT);

    if (!mapmng.isReady()) {
        return;
    }

    int pPointsNum = 0;
    int cPointsNum = 0;

    int8_t qual_thr = mapmng.getQualThreshold();
    std::vector<LidarPoint>& pframe = mapmng.getLidarFramePoints(map_index);
    std::vector<LidarPoint>& cframe = mapmng.getLidarFramePoints(map_index+1);

    Point2f ref(point_pos_x, point_pos_y);

    // paired points
    //std::vector<Point2fPair> *ppair = mapmng.getPointPairs();

    // Define shapes enclosed within a pair of glBegin and glEnd
    glPointSize(2.0);
    glBegin(GL_POINTS);
        // Display points of previous frame
        GL_SET_COLOR_GREEN;    // Green
        for(auto &a : pframe) {
            if (a.getQual() > qual_thr) {
                Point2f c = (a.getPoint2f() * point_scale) + ref;
                glVertex2f(c.getX(), c.getY());
                pPointsNum++;
            }
        }

        // Display points of current frame
        GL_SET_COLOR_YELLOW;    // Yellow
        for(auto &a : cframe) {
            if (a.getQual() > qual_thr) {
                Point2f c = (a.getPoint2f() * point_scale) + ref;
                glVertex2f(c.getX(), c.getY());
                cPointsNum++;
            }
        }

        // Display all low quality points
        GL_SET_COLOR_BLUE;    // Blue
        for(auto &a : pframe) {
            if (a.getQual() <= qual_thr) {
                Point2f c = (a.getPoint2f() * point_scale) + ref;
                glVertex2f(c.getX(), c.getY());
            }
        }

        GL_SET_COLOR_RED;    // Red
        for(auto &a : cframe) {
            if (a.getQual() <= qual_thr) {
                Point2f c = (a.getPoint2f() * point_scale) + ref;
                glVertex2f(c.getX(), c.getY());
            }
        }
    glEnd();

#if defined(DEBUG_SHOW_ANGLE_MATCHED_OUTLIER)
    glBegin(GL_LINES);
        // all pair lines
        glColor3f(1.0f, 0.0f, 0.0f);    // Red
        for(auto &a : *ppair) {
            glVertex2f(point_pos_x + a.points.first.x * point_scale,
                    point_pos_y + a.points.first.y * point_scale);
            glVertex2f(point_pos_x + a.points.second.x * point_scale, 
                    point_pos_y + a.points.second.y * point_scale);
        }

        // removed pair lines
        glColor3f(1.0f, 1.0f, 0.0f);    // Yellow
        for(auto &a : *ppair) {
            if (a.outlier) {
                glVertex2f(point_pos_x + a.points.first.x * point_scale,
                        point_pos_y + a.points.first.y * point_scale);
                glVertex2f(point_pos_x + a.points.second.x * point_scale, 
                        point_pos_y + a.points.second.y * point_scale);
            }
        }
    glEnd();
#endif

    /* Display compensated lidar point frame on screen
     */
#if 0
    float  angle_comp = mapmng.getEstAngle();
    if (isDisplayCompensatedFrame && angle_comp != 0) {
        glPointSize(2.0);
        glBegin(GL_POINTS);
            // Display points of previous frame
            glColor3f(0.0f, 1.0f, 1.0f);    // Cian?
            for(auto &a : cframe->points_) {
                if (a.qual > qual_thr) {
                    Point2f point;
                    point.x = a.dist * cos(((a.angle + angle_comp)/180.0) * PI);
                    point.y = a.dist * sin(((a.angle + angle_comp)/180.0) * PI);
                    glVertex2f(point_pos_x + (point.x - adjustP.x) * point_scale , 
                            point_pos_y + (point.y - adjustP.y) * point_scale );
                    pPointsNum++;
                }
            }
        glEnd();
    }
#endif

    glEnd();


    glFlush();

    //std::cout << "high quality points(prev,curr) = (" << pPointsNum << " , "
    //          << cPointsNum << ")" << std::endl;
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
            map_index++;
            if (map_index >= max_index) {
                map_index = max_index-1;
            }

            // check frame distance
            //mapmng.checkFrameDistance(map_index);
            //mapmng.findOptimalTranslation(map_index);
            //mapmng.findOptimalRotation(map_index);
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


