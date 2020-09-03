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

    std::vector<point2_t> pts = mapmng.get_map_point(map_index);

    // Define shapes enclosed within a pair of glBegin and glEnd
    glBegin(GL_POINTS);
        glColor3f(1.0f, 1.0f, 0.0f);    // Red
        for(auto &a : pts) {
            glVertex2f(point_pos_x + a.x * point_scale, point_pos_y + a.y * point_scale);
        }
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
        case 'i':
        case 'I':
            int max_index = mapmng.get_map_num();
            map_index++;
            if (map_index >= max_index) {
                map_index = max_index-1;
            }
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


