#ifndef GUI_WINDOW_H
#define GUI_WINDOW_H

#include <QWidget>
#include <QtGui>
#include <vector>
#include <map>
#include <string>

#include "common/app_hppcommon.h"

QT_BEGIN_NAMESPACE
class QLabel;
class QWidget;
QT_END_NAMESPACE

namespace gui {
    class GLWidget;
    class WidgetOptExplorer;
    class Application;
    // class Commander;
} // namespace gui

namespace rtql8 {
    namespace toolkit {
        class Simulator;
    } // namespace toolkit
} // namespace rtql8


namespace gui {
    
    class Window : public QMainWindow {
        Q_OBJECT;

    public:
        Window();
        virtual ~Window();

        rtql8::toolkit::Simulator* sim();
        
        void initApp();
        void initUI();
        void initTimer();
        void setCenter();

        void createActions();
        QAction* createAction(const char* _name);
        void createToolbars();
        void createCommanderToolbar();
        void createMenus();

    public slots:
        void onTimerRender();
        void onTimerKinect();
        void onTimerIdle();
        void onSliderFrameChanged(int index);
        void updateInfo();

        void onActionPlay();
        void onActionUI();
        void onActionLoad();
        void onActionSave();
        void onActionSaveAs();

        void onActionExportSkel();
        void onActionExportMotion();

        void onActionPrint();
        void onActionReset();

        void onActionRead();
        void onActionWrite();
        void onActionWriteAs();

        void onActionSolver();

        void onActionEnter();
        void onActionSolverRun();
        void onActionSolverStop();

        void onCMDReturnPressed();

        void onRadioOptModeToggled(bool v);
        void onBtnRefreshClicked();
        void onBtnLoadGeneClicked();

        bool isRenderingUI();

    protected:
        void keyPressEvent(QKeyEvent* event);


        void takeCapture();
        void takeScreenshot(const char* const filename);
    protected:
        MEMBER_PTR(QTimer*, timerRender);
        MEMBER_PTR(QTimer*, timerIdle);
        MEMBER_PTR(QTimer*, timerKinect);

        MEMBER_PTR(GLWidget*, gl);
        MEMBER_PTR(WidgetOptExplorer*, exp);
        MEMBER_PTR(QLabel*, labelTime);
        MEMBER_PTR(QSlider*, sliderFrame);
        MEMBER_PTR(QSlider*, sliderTarget);
        // Commander Toolbar
        MEMBER_PTR(QLineEdit*, cmdTextInput);
        MEMBER_PTR(QStatusBar*, statusbar);
        MEMBER_VAR(std::string, prevstatus);

        MEMBER_PTR(QRadioButton*, radioOptSingle);
        MEMBER_PTR(QRadioButton*, radioOptRemote);

    protected:
        std::map<std::string, QAction*> actions;
        
    protected:
        MEMBER_PTR(Application*, app);
        // MEMBER_PTR(Commander*, commander);
        MEMBER_VAR(bool, wasRunning);
    };

} // namespace gui

#endif // #ifndef GUI_WINDOW_H
