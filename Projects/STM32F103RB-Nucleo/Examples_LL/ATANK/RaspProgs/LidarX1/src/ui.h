#ifndef __UI_H__
#define __UI_H__

/* OpenGL function includes
 */
#include <GL/glut.h>
#include <vector>

void initOpenGL(int argc, char *argv[]);
void doKeyboard(unsigned char key, int x, int y);
void doSpecial(int key, int x, int y);
void reshape(GLsizei width, GLsizei height);
void display();
void initGL();

#endif
