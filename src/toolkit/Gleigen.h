#ifndef TOOLKIT_GLEIGEN_H
#define TOOLKIT_GLEIGEN_H

#include <Eigen/Dense>
#include "utils/LoadOpengl.h"

namespace rtql8 {
    namespace toolkit {
        static void glColor3d(const Eigen::Vector3d& color) {
            ::glColor3d(color(0), color(1), color(2));
        }
        static void glColorRGB(int r, int g, int b) {
            ::glColor3d(
                (double)r / 255.0, (double)g / 255.0, (double)b / 255.0);
        }

        static void glTranslated(const Eigen::Vector3d& pos) {
            ::glTranslated(pos(0), pos(1), pos(2));
        }

        static void glScaled(const Eigen::Vector3d& s) {
            ::glScaled(s(0), s(1), s(2));
        }

        static void glScaled(double s) {
            ::glScaled(s, s, s);
        }

        static void glVertex3d(const Eigen::Vector3d& v) {
            ::glVertex3d(v(0), v(1), v(2));
        }

        static void glLoadMatrixd( const Eigen::Matrix4d& M) {
            double m[17];
            int ptr = 0;
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    m[ptr++] = M(j, i);
                }
            }
            ::glLoadMatrixd(m);
        }

        static void glMultMatrixd( const Eigen::Matrix4d& M) {
            double m[17];
            int ptr = 0;
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    m[ptr++] = M(j, i);
                }
            }
            ::glMultMatrixd(m);
        }


        static void glRenderString(void* font, const char* const text) {
            int len, i;
            glRasterPos2f(0.0f, 0.0f);
            len = (int) strlen(text);
            for (i = 0; i < len; i++)
            {
                glutBitmapCharacter(font, text[i]);
            }
        }

        static void glRenderString(const char* const text) {
            glRenderString((void*)GLUT_BITMAP_HELVETICA_18, text);
        }


        class GLMatrixManip {
        public:
            GLMatrixManip() : count(0) { ::glPushMatrix(); }
            ~GLMatrixManip() { ::glPopMatrix(); }
            bool exit() { ++count; if (count <= 1) return true; else return false; }
        private:
            int count;
        };

    } // namespace toolkit
} // namespace rtql8

#define glBlock() for(rtql8::toolkit::GLMatrixManip m; m.exit(); )


#endif // #ifndef TOOLKIT_GLEIGEN_H

