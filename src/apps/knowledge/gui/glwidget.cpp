#include "glwidget.h"

// Other headers
#include "utils/LoadOpengl.h"
#include "application.h"
#include "toolkit/CppCommon.h"
#include "toolkit/Gleigen.h"
// Local headers
#include "window.h"
#include "camera.h"

namespace gui {
    GLWidget::GLWidget(Window* _parent)
        : QGLWidget(QGLFormat(QGL::SampleBuffers))
        , MEMBER_INIT(window, _parent)
        , MEMBER_INIT_NULL(camera)
    {
        setFixedSize(GL_WINDOW_WIDTH, GL_WINDOW_HEIGHT);
        setAutoFillBackground(false);

        set_camera( new Camera() );

        camera()->begin = Eigen::Vector2d(0, 0);
        // camera()->pos = Eigen::Vector3d(0, 0, 4);
        camera()->trackball(0.0f, 0.0f, 0.0f, 0.0f);

        // // Default setting
        // camera()->pos = Eigen::Vector3d(-0.2, -0.05, 5.0);
        // camera()->q(0) = -0.1848f;
        // camera()->q(1) = -0.4537f;
        // camera()->q(2) = -0.1036f;
        // camera()->q(3) =  0.8656f;

        // [-0.2, -0.55, 5] [-0.0798776, -0.018322, -0.0010117, 0.99664]
        // camera()->pos  = Eigen::Vector3d(-0.2, -0.55, 5.0);
        // camera()->pos  = Eigen::Vector3d(-1.2, -0.55, 5.0);
        camera()->pos  = Eigen::Vector3d(-0.8, -0.95, 5.3);
        // camera()->q(0) = -0.079;
        // camera()->q(1) = -0.018;
        // camera()->q(2) = -0.001;
        // camera()->q(3) = 0.996;
        camera()->q(0) = -0.07527;
        camera()->q(1) = -0.001988;
        camera()->q(2) = -0.000;
        camera()->q(3) = 0.99645;
    }

/*
  CAM(p = [-0.1000 -0.4000 5.0000], q = [-0.2836 0.2748 0.0557 0.9170])
CAM(p = [-0.1000 0.5000 2.1000], q = [-0.1019 0.6819 0.1001 0.7174])
CAM(p = [-0.8500 0.5000 2.9000], q = [0.0855 -0.1091 0.0247 0.9900])
*/

                                 
    void GLWidget::initializeGL() {
        static const GLfloat light0_pos[4]   = { 0.0f, 50.0f, 0.0f, 0.0f };
        // white light
        static const GLfloat light0_color[4] = { 0.01f, 0.01f, 0.01f, 1.0f };
        static const GLfloat light1_pos[4]   = {  50.0f, 50.0f, 0.0f, 0.0f };
        // cold blue light
        static const GLfloat light1_color[4] = { 0.4f, 0.4f, 0.4f, 1.0f };

        const char* vendor = (const char*)glGetString( GL_VENDOR );
        const char* renderer = (const char*)glGetString( GL_RENDERER );
        const char* version = (const char*)glGetString( GL_VERSION );
        const char* extensions = (const char*)glGetString( GL_EXTENSIONS );
        // cout << "Vendor = " << vendor << endl;
        // cout << "Renderer = " << renderer << endl;
        // cout << "Version = " << version << endl;
        // cout << "Extensions = " << extensions << endl;

        /* remove back faces */
        glDisable(GL_CULL_FACE);
        glEnable(GL_DEPTH_TEST);
        // glEnable(GL_CULL_FACE);
        // glCullFace(GL_BACK);
        glDepthFunc(GL_LEQUAL);
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

        glEnable(GL_LINE_SMOOTH);
        glHint(GL_LINE_SMOOTH_HINT,  GL_NICEST);

        /* speedups */
        glEnable(GL_DITHER);
        glShadeModel(GL_SMOOTH);
        glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
        glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

        /* light */
        GLfloat ambient[] = {0.8f, 0.8f, 0.8f, 1.0f};
        glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);

        glLightfv(GL_LIGHT0, GL_POSITION, light0_pos);
        glLightfv(GL_LIGHT0, GL_DIFFUSE,  light0_color);
        // glLightfv(GL_LIGHT0, GL_AMBIENT,  light0_color);
        glLightfv(GL_LIGHT1, GL_POSITION, light1_pos);
        glLightfv(GL_LIGHT1, GL_DIFFUSE,  light1_color);
        glEnable(GL_LIGHT0);
        // glEnable(GL_LIGHT1);
        glEnable(GL_LIGHTING);


        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
        glEnable(GL_COLOR_MATERIAL);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        // LOG_INFO << FUNCTION_NAME() << " OK";
    }

    // GLUquadricObj* mQuadric = NULL;


    void GLWidget::paintGL() {
        using namespace rtql8::toolkit;
        // cout << IO(camera()->pos) << " " << IO(camera()->q) << endl;
        glEnable(GL_DEPTH_TEST);
        // Clear
        // glClearColor( 1.0f, 1.0f, 1.0f, 1.0f );

        // glClearColor( 0.15f, 0.15f, 0.15f, 1.0f );
        glClearColor( 0.15f, 0.15f, 0.15f, 0.0f );
        glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

        // Render
        glLoadIdentity();

        GLfloat m[4][4];
        glTranslatef( camera()->pos(0), camera()->pos(1), -camera()->pos(2) );
        // glTranslated( camera()->pos );


        camera()->build_rotmatrix( m );
        glMultMatrixf( &m[0][0] );

        // if (follow() >= 0) {
        //     Eigen::Vector3d p = window()->sim()->skel(1)->getNode(follow())->getWorldCOM();
        //     sehoon::glTranslated(-p);
        // }


        glEnable(GL_LIGHTING);
        bool renderUI = window()->isRenderingUI();
        window()->app()->render(renderUI);
        // window()->sim()->render();

        // Testing object for rendering
        // glColor3d(1.0, 0.0, 0.0);
        // glBlock() {
        //     glutSolidSphere(1.0, 10, 10);
        // }
        // renderController();


        // renderAxis();
        // renderCOM();
        // renderCOMTrajectory();
        // renderCOP();
        
        // if (!mQuadric) {
        //     mQuadric = gluNewQuadric();
        // }
        // gluCylinder(mQuadric, 1.0, 1.0, 4.0, 20, 20);
    }

    void GLWidget::resizeGL( int w, int h ) {
        glViewport(0, 0, (GLint) w, (GLint) h);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();

        gluPerspective(45.0f, (GLfloat)w/h, 1.0, 100.0);
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        // LOG_INFO << FUNCTION_NAME();
    }

    int GLWidget::buttonToInteger(QMouseEvent* event) {
        if ( (event->buttons() & Qt::LeftButton) != 0) {
            return 0;
        } else if ( (event->buttons() & Qt::MidButton) != 0) {
            return 1;
        } else if ( (event->buttons() & Qt::RightButton) != 0) {
            return 2;
        }
        return 0;
    }

    void GLWidget::mousePressEvent(QMouseEvent* event) {
        QPoint m1 = event->pos();
        int btn = buttonToInteger(event);
        if ( window()->app()->click(btn, 0, m1.x(), m1.y()) ) {
            return;
        }
        camera()->begin = Eigen::Vector2d( m1.x(), m1.y() );
    }

    void GLWidget::mouseReleaseEvent(QMouseEvent* event) {
        QPoint m1 = event->pos();
        int btn = buttonToInteger(event);
        if ( window()->app()->click(btn, 1, m1.x(), m1.y()) ) {
            return;
        }
    }


    void GLWidget::mouseMoveEvent(QMouseEvent* event) {
        QPoint m1 = event->pos();
        if ( window()->app()->drag(m1.x(), m1.y()) ) {
            return;
        }
        if ( (event->buttons() & Qt::LeftButton) != 0) {
            if ( (qApp->keyboardModifiers() & Qt::ShiftModifier) != 0 ) {
                float dx = m1.x() - camera()->begin.x();
                float dy = m1.y() - camera()->begin.y();
                camera()->pos(0) += 0.05 * dx;
                camera()->pos(1) -= 0.05 * dy;
            } else if ( (qApp->keyboardModifiers() & Qt::ControlModifier) != 0 ) {
                float amount = (m1.y() - camera()->begin.y())
                    + (m1.x() - camera()->begin.x());
                camera()->pos(2) += 0.1 * amount;
            } else {
                float sx = size().width();
                float sy = size().height();
                camera()->add_quat(
                    (2.0 * camera()->begin.x() - sx) / sx,
                    (sy - 2.0 * camera()->begin.y()) / sy,
                    (2.0 * m1.x() - sx) / sx,
                    (sy - 2.0 * m1.y()) / sy
                    );
            }
        }
        camera()->begin = Eigen::Vector2d( m1.x(), m1.y() );

        // camera()->beginx = m1.x();
        // camera()->beginy = m1.y();
        
    }


    void GLWidget::renderAxis() {
        // if (!window()->actRenderAxis()->isChecked()) {
        //     return;
        // }
        // glBlock() {
        //     sehoon::globjects::renderAxis(10.0);
        // }
    }

    void GLWidget::renderCOM() {
        // if (!window()->actRenderCOM()->isChecked()) {
        //     return;
        // }
        // Eigen::Vector3d COM = window()->sim()->skelstate()->COM;
        // glBlock() {
        //     sehoon::glTranslated(COM);
        //     sehoon::glColor3d( sehoon::glcolors::pink() );
        //     glutSolidSphere(0.1, 10, 10);
        // }

        // Eigen::Vector3d Cdot = window()->sim()->skelstate()->COMdot;
        // if (Cdot.norm() > 0.05) {
        //     glBlock() {
        //         Eigen::Vector3d p = COM;
        //         Eigen::Vector3d dir = Cdot;
        //         Eigen::Vector3d q = p + 0.2 * dir;
        //         sehoon::globjects::renderArrow(p, q, 0.05, 0.2);
        //     }
        // } else {
        //     glBlock() {
        //         Eigen::Vector3d p = COM;
        //         Eigen::Vector3d dir(1, 0, 0);
        //         Eigen::Vector3d q0 = p + 0.1 * dir;
        //         Eigen::Vector3d q1 = p - 0.1 * dir;
        //         sehoon::globjects::renderArrow(p, q0, 0.05, 0.2);
        //         sehoon::globjects::renderArrow(p, q1, 0.05, 0.2);
        //     }
        // }
    }

    void GLWidget::renderCOMTrajectory() {
        // if (!window()->actRenderCOM()->isChecked()) {
        //     return;
        // }
        // // glPushAttrib(GL_LINE_BIT);
        // glDisable(GL_LIGHTING);
        // glLineWidth(3.0);
        // sehoon::glColor3d( sehoon::glcolors::pink() );
        // EIGEN_V_VEC3D traj = window()->sim()->comTrajectory();
        // glBegin(GL_LINE_STRIP);
        // FOREACH(const Eigen::Vector3d& C, traj) {
        //     sehoon::glVertex3d(C);
        // }
        // glEnd();
        // glLineWidth(1.0);
        // glEnable(GL_LIGHTING);

        // glPopAttrib();
    }


    void GLWidget::renderCOP() {
        // if (!window()->actRenderCOM()->isChecked()) {
        //     return;
        // }
        // Eigen::Vector3d COP = window()->sim()->skelstate()->COP;
        // glBlock() {
        //     sehoon::glTranslated(COP);
        //     sehoon::glColor3d( sehoon::glcolors::lime() );
        //     glutSolidSphere(0.1, 10, 10);

        // }
    }

    void GLWidget::renderController() {
        // if (!window()->actRenderCon()->isChecked()) {
        //     return;
        // }
        // window()->con()->render();
    }



} // namespace gui
