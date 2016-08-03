#include "window.h"

#include <boost/algorithm/string.hpp>

#include "glwidget.h"
#include "widget_opt_explorer.h"
#include "application.h"
// #include "commander.h"
#include "toolkit/Toolkit.h"
#include "GuiKinController.h"
#ifdef _WIN32
#undef ERROR
#endif // #ifdef _WIN32
#include "common/app_cppcommon.h"
#include "utils/Paths.h"

namespace gui {
    Window::Window()
        : QMainWindow()
        , MEMBER_INIT_NULL(timerRender)
        , MEMBER_INIT_NULL(timerKinect)
        , MEMBER_INIT_NULL(timerIdle)
        , MEMBER_INIT_NULL(gl)
        , MEMBER_INIT_NULL(exp)
        , MEMBER_INIT_NULL(app)
        // , MEMBER_INIT_NULL(commander)
    	, MEMBER_INIT(prevstatus, "")
    {
        srand( (unsigned int) time (NULL) );
        initApp();
        initUI();
        initTimer();
        setCenter();

        set_wasRunning(true);
        // CHECK_EQ(2 + 3, 4);
    }

    Window::~Window() {
        // if (commander()) {
        //     commander()->process("stop");
        // }
        // MEMBER_RELEASE_PTR(commander);

        MEMBER_RELEASE_PTR(timerRender);
        MEMBER_RELEASE_PTR(timerIdle);
        MEMBER_RELEASE_PTR(gl);
    }

    void Window::initApp() {
        set_app( new Application() );
        app()->init(GL_WINDOW_WIDTH, GL_WINDOW_HEIGHT);
        // set_commander( new Commander(app()) );
        LOG_INFO << FUNCTION_NAME() << " OK";
    }

    rtql8::toolkit::Simulator* Window::sim() {
        return app()->sim();
    }


    void Window::initUI() {
        QWidget* widget = new QWidget;
        setCentralWidget(widget);
        QHBoxLayout* layout = new QHBoxLayout();

        set_gl( new GLWidget(this) );
        layout->addWidget( gl() );

        // ////////////////////////////////////////////////////////////////////////////////
        // // The left hand side view - for exploring the gene
        // // Outdate- commenting out 
        // {
        //     QVBoxLayout* layoutViews = new QVBoxLayout();
        //     set_exp( new WidgetOptExplorer(this) );
        //     layoutViews->addWidget(exp());

        //     {
        //         QPushButton* btn = new QPushButton(tr("Refresh")) ;
        //         layoutViews->addWidget(btn);
        //         connect( btn, SIGNAL(clicked()), this, SLOT(onBtnRefreshClicked()) );
                
        //     }
        //     {
        //         QPushButton* btn = new QPushButton(tr("LoadGene")) ;
        //         layoutViews->addWidget(btn);
        //         connect( btn, SIGNAL(clicked()), this, SLOT(onBtnLoadGeneClicked()) );
                
        //     }
        //     //     set_gl_front( new GLWidget(this) );
        //     //     gl_front()->setFrontViewWidget();
        //     //     layoutViews->addWidget(gl_front());

        //     //     set_gl_side( new GLWidget(this) );
        //     //     gl_side()->setSideViewWidget();
        //     //     layoutViews->addWidget(gl_side());

        //     //     set_txt( new TextWidget(this) );
        //     //     layoutViews->addWidget(txt());

        //     layoutViews->addStretch(1);
        //     layout->addLayout(layoutViews);
        // }
        // // The left hand side view - for exploring the gene
        // ////////////////////////////////////////////////////////////////////////////////


        // // QTextEdit* txt = new QTextEdit();

        widget->setLayout(layout);

        createActions();
        createToolbars();
        createCommanderToolbar();
        set_statusbar( statusBar() );
        statusbar()->showMessage("Hi there!");
        createMenus();

        LOG_INFO << FUNCTION_NAME() << " OK";
    }

    void Window::initTimer() {
        set_timerRender( new QTimer(this) );
        connect(timerRender(), SIGNAL(timeout()), this, SLOT(onTimerRender()));
        timerRender()->start(30);

        set_timerKinect( new QTimer(this) );
        connect(timerKinect(), SIGNAL(timeout()), this, SLOT(onTimerKinect()));
        timerKinect()->start(50);


        set_timerIdle( new QTimer(this) );
        connect(timerIdle(), SIGNAL(timeout()), this, SLOT(onTimerIdle()));
        timerIdle()->start(0);

        LOG_INFO << FUNCTION_NAME() << " OK";
    }

    void Window::setCenter() {
        QWidget* widget = this;
        QSize size = widget->sizeHint();
        QDesktopWidget* desktop = QApplication::desktop();
        int width = desktop->width();
        int height = desktop->height();
        int mw = size.width();
        int mh = size.height();
        // int centerW = (width/2) - (mw/2);
        // int centerH = (height/2) - (mh/2);
        // int centerW = width / 2;
        int centerW = 0;
        int centerH = 0;
        widget->move(centerW, centerH);

        // int w = width();
        // int h = height();
        // cout << w << " " << h << endl;
        LOG_INFO << FUNCTION_NAME() << " OK";
    }

    void Window::createActions() {
        createAction("Play")->setCheckable(true);
        createAction("Anim")->setCheckable(true);
        createAction("Record")->setCheckable(true);
        createAction("Reset");
        createAction("UI")->setCheckable(true);
        actions["UI"]->setChecked( app()->getFlagUIButtonChecked() );
        createAction("Load")->setShortcut( QKeySequence("Ctrl+L") );
        createAction("Save")->setShortcut( QKeySequence("Ctrl+S") );
        createAction("Save As")->setShortcut( QKeySequence("Ctrl+Shift+S") );
        createAction("Export Skel");
        createAction("Export Motion");
        createAction("Print")->setShortcut( QKeySequence("Ctrl+P") );

        createAction("Read")->setShortcut( QKeySequence("Ctrl+R") );
        createAction("Write")->setShortcut( QKeySequence("Ctrl+W") );
        createAction("Write As")->setShortcut( QKeySequence("Ctrl+Shift+W") );

        createAction("Solver");
        createAction("Enter");
        createAction("Auto")->setCheckable(true);
        createAction("SolverRun");
        createAction("SolverStop");
    }

    QAction* Window::createAction(const char* _name) {
        std::string name(_name);
        std::string method = "1onAction" + name + "()"; // SLOT(macro)
        boost::erase_all(method, " ");
        // LOG_INFO << "name = " << name << " method = " << method;
        QAction* action = new QAction(tr(name.c_str()), this);
        connect(action, SIGNAL(triggered()), this, method.c_str());
        actions[name] = action;
        return action;
    }
    
    void Window::createToolbars() {
        QToolBar* toolbar = addToolBar(tr("Playback"));
        set_labelTime( new QLabel(tr("----")) );

        toolbar->addWidget( labelTime() );
        toolbar->addAction( actions["Play"] );
        toolbar->addAction( actions["Anim"] );
        toolbar->addAction( actions["Record"] );
        toolbar->addAction( actions["Reset"] );
        toolbar->addAction( actions["UI"] );


        set_sliderFrame( new QSlider(Qt::Horizontal) );
        sliderFrame()->setMaximumSize(QSize(200, 30));
        toolbar->addWidget( sliderFrame() );
        connect( sliderFrame(), SIGNAL(valueChanged(int)),
                 this, SLOT(onSliderFrameChanged(int)) );


        toolbar->addWidget( new QLabel(tr("Target")) );
        set_sliderTarget( new QSlider(Qt::Horizontal) );
        sliderTarget()->setRange(0, 100);
        sliderTarget()->setValue(100 * app()->getTaskParam());
        toolbar->addWidget( sliderTarget() );

        toolbar->addAction( actions["Solver"] );

        set_radioOptSingle(new QRadioButton(tr("Single")) );
        set_radioOptRemote(new QRadioButton(tr("Remote")) );
        radioOptSingle()->setChecked(true);
        connect( radioOptSingle(), SIGNAL(toggled(bool)),
                 this, SLOT(onRadioOptModeToggled(bool)) );
        connect( radioOptRemote(), SIGNAL(toggled(bool)),
                 this, SLOT(onRadioOptModeToggled(bool)) );
        toolbar->addWidget( radioOptSingle() );
        toolbar->addWidget( radioOptRemote() );

        LOG_INFO << FUNCTION_NAME() << " OK";

    }

    void Window::createCommanderToolbar() {
        QToolBar* toolbar = addToolBar(tr("Command"));
        toolbar->addWidget( new QLabel(tr("Instruction>")) );

        set_cmdTextInput( new QLineEdit() );
        toolbar->addWidget(cmdTextInput());
        connect(cmdTextInput(), SIGNAL(returnPressed()),
                this, SLOT(onCMDReturnPressed()) );

        toolbar->addAction( actions["Enter"] );
        toolbar->addAction( actions["Auto"] );
    }

    void Window::createMenus() {
        QMenu* menuFile = menuBar()->addMenu(tr("File"));
        menuFile->addAction( actions["Load"] );
        menuFile->addAction( actions["Save"] );
        menuFile->addAction( actions["Save As"] );
        menuFile->addSeparator();
        menuFile->addAction( actions["Export Skel"] );
        menuFile->addAction( actions["Export Motion"] );
        menuFile->addSeparator();
        menuFile->addAction( actions["Print"] );

        QMenu* menuController = menuBar()->addMenu(tr("Controller"));
        menuController->addAction( actions["Enter"] );
        menuController->addSeparator();
        
        menuController->addAction( actions["Read"] );
        menuController->addAction( actions["Write"] );
        menuController->addAction( actions["Write As"] );

        QMenu* menuOptimizer = menuBar()->addMenu(tr("Solver"));
        menuOptimizer->addAction( actions["SolverRun"] );
        menuOptimizer->addAction( actions["SolverStop"] );
        menuOptimizer->addSeparator();
        menuOptimizer->addAction( actions["Solver"] );
     
        LOG_INFO << FUNCTION_NAME() << " OK";
    }
    

    void Window::onTimerRender() {
        gl()->updateGL();
    }

    void Window::onTimerKinect() {
#ifdef KN_USE_KINECT
        if (app()->kinect()) {
            app()->kinect()->onTimerUpdate();
        }
#endif
    }

    void Window::onTimerIdle() {
        //LOG_INFO << FUNCTION_NAME();
        updateInfo();
        // LOG_EVERY_N(INFO, 100) << "selection = " << IO(exp()->selectedGene());

        if (app()->autostart()) {
            onActionReset();
            app()->onSimulationBegin();
            actions["Play"]->setChecked(true);
        }

        bool isPlaying = actions["Play"]->isChecked()
            || actions["Anim"]->isChecked()
            || sliderFrame()->isSliderDown();
        if (app()->useMotionArrays == false) {
            isPlaying = true;
        }
            
        app()->set_isPlaying( isPlaying );
        if (actions["Play"]->isChecked()) {
            bool isRunning = app()->step();
            if (wasRunning() && !isRunning ) {

                bool anotherSimulationNeeded = app()->onSimulationTerminated();
                if (anotherSimulationNeeded) {
                    LOG_INFO << "Simulation terminated: reset to the beginning";
                    onActionReset();
                    app()->onSimulationBegin();
                } else {
                    LOG_INFO << "Simulation terminated: pop the play button";
                    actions["Play"]->setChecked(false);
                    // onActionReset();
                }
            }
            set_wasRunning(isRunning);
        } else if (actions["Anim"]->isChecked()) {
            int index = sliderFrame()->value();
            index += 2;
            if (index < app()->replayNumFrames()) {
            	sliderFrame()->setValue(index);
            } else {
                LOG_INFO << "Animation terminated: pop the play button";
                actions["Anim"]->setChecked(false);
            }
        }

        if (actions["Record"]->isChecked() && actions["Anim"]->isChecked()) {
            takeCapture();
        }
        //LOG_INFO << FUNCTION_NAME() << " OK";

    }

    void Window::onSliderFrameChanged(int index) {
        app()->replayUpdateToFrame(index);
    }

    void Window::updateInfo() {
        labelTime()->setText(
            tr( (boost::format("T : %07.4f") % sim()->t()).str().c_str() )
            );

        sliderFrame()->setRange(0, app()->replayNumFrames() - 1);
    }

    void Window::onActionPlay() {
        LOG_INFO << FUNCTION_NAME() << " OK";
    }

    void Window::onActionUI() {
        LOG_INFO << FUNCTION_NAME() << " OK";
    }

    void Window::onActionLoad() {
        QString qfilename = QFileDialog::getOpenFileName(
            this, tr("Load replay from json"), tr("../data/replay/"), tr("Json Files (*.json)"));
        std::string filename = qfilename.toStdString();
        if (filename.length() == 0) {
            LOG_WARNING << "User cancelled loading";
            return;
        }
        LOG_INFO << "Filename = [" << filename << "]";
        app()->replay()->loadFile(filename.c_str());
        app()->fillTheReplays();

        LOG_INFO << FUNCTION_NAME() << " "
                 << ": # frames = " << app()->replayNumFrames() << " OK";
    }

    void Window::onActionSave() {
        std::string filename = app()->getFilename("json");
        app()->replay()->saveFile(filename.c_str());
        LOG_INFO << FUNCTION_NAME() << " : [" << filename << "] OK";
    }

    void Window::onActionSaveAs() {
        QString qfilename = QFileDialog::getSaveFileName(
            this, tr("Save replay as json"), tr("./"), tr("Json Files (*.json)"));
        std::string filename = qfilename.toStdString();
        if (filename.length() == 0) {
            LOG_WARNING << "User cancelled saving";
            return;
        }
        app()->replay()->saveFile(filename.c_str());

        LOG_INFO << FUNCTION_NAME() << " : [" << filename << "] OK";
    }

    void Window::onActionExportSkel() {
        app()->exportSkel();
        LOG_INFO << FUNCTION_NAME() << " OK";
    }

    void Window::onActionExportMotion() {
        app()->exportMotion();
        LOG_INFO << FUNCTION_NAME() << " OK";
    }

    void Window::onActionPrint() {
        LOG_INFO << endl << app()->printString();
        LOG_INFO << FUNCTION_NAME() << " OK";
    }

    void Window::onActionRead() {
        // std::string filename = RTQL8_DATA_PATH"xml/test.xml";
        QString qfilename = QFileDialog::getOpenFileName(
            this, tr("Load knowledge from xml"), tr(RTQL8_DATA_PATH"knowledge"), tr("XML Files (*.xml)"));
        std::string filename = qfilename.toStdString();
        if (filename.length() == 0) {
            LOG_WARNING << "User cancelled loading";
            return;
        }
        LOG_INFO << "Filename = [" << filename << "]";
        app()->loadController(filename.c_str());
        LOG_INFO << FUNCTION_NAME() << " : [" << filename << "] OK";
    }
    
    void Window::onActionWrite() {
        std::string filename = RTQL8_DATA_PATH"knowledge/output.xml";
        app()->saveController(filename.c_str());
        LOG_INFO << FUNCTION_NAME() << " : [" << filename << "] OK";
    }
    
    void Window::onActionWriteAs() {
        QString qfilename = QFileDialog::getSaveFileName(
            this, tr("Save knowledge as xml"), tr(RTQL8_DATA_PATH"knowledge")
            , tr("XML Files (*.xml)"));
        std::string filename = qfilename.toStdString();
        if (filename.length() == 0) {
            LOG_WARNING << "User cancelled saving";
            return;
        }
        app()->saveController(filename.c_str());
        LOG_INFO << FUNCTION_NAME() << " : [" << filename << "] OK";
    }

    void Window::onActionReset() {
        LOG_INFO << FUNCTION_NAME();
        int v = sliderTarget()->value();
        int x = sliderTarget()->maximum();
        double t = (double)v / (double)x;
        cout << "t = " << t << " = " << v << " / " << x << endl;
        app()->reset(t);
    }

    // void Window::onActionOptimize() {
    void Window::onActionSolver() {
        //app()->optimize();
        //exp()->reload();
        // app()->test();
        onActionSolverRun();
    }

    void Window::onActionEnter() {
        std::string txt = cmdTextInput()->text().toStdString();
        LOG_INFO << FUNCTION_NAME() << " OK : " << txt;
        bool result = app()->interpret(txt.c_str());

        if (result) {
            cmdTextInput()->setText(tr(""));
        }
    }

    void Window::onActionSolverRun() {
        int evaluatorType = 0;
        if (radioOptSingle()->isChecked()) {
            evaluatorType = 0;
        } else if (radioOptRemote()->isChecked()) {
            evaluatorType = 2;
        }
        LOG_INFO << FUNCTION_NAME() << " evaluatorType = " << evaluatorType;
        app()->solver_run(evaluatorType);
        LOG_INFO << FUNCTION_NAME() << " OK";
    }

    void Window::onActionSolverStop() {
        app()->solver_stop();
        LOG_INFO << FUNCTION_NAME() << " OK";
    }
    
    void Window::onCMDReturnPressed() {
        onActionEnter();
    }

    void Window::onRadioOptModeToggled(bool v) {
        if (v != true) {
            return;
        }
        int evaluatorType = 0;
        if (radioOptSingle()->isChecked()) {
            evaluatorType = 0;
        } else if (radioOptRemote()->isChecked()) {
            evaluatorType = 2;
        }
        LOG_INFO << FUNCTION_NAME() << " OK: Type = " << evaluatorType;
        app()->set_solver_type(evaluatorType);

    }

    void Window::onBtnRefreshClicked() {
        exp()->reload();
        // Eigen::VectorXd params = exp()->selectedGene();
        // app()->setControllerParams(params);
        // app()->reset();
        // updateInfo();
        LOG_INFO << FUNCTION_NAME() << " OK";
    }

    void Window::onBtnLoadGeneClicked() {
        Eigen::VectorXd params = exp()->selectedGene();
        app()->setControllerParams(params);
        app()->reset();
        set_wasRunning(true);
        updateInfo();
        LOG_INFO << FUNCTION_NAME() << " OK";
    }

    void Window::keyPressEvent(QKeyEvent* event) {
        if (event->key() == Qt::Key_U) {
            LOG_INFO << FUNCTION_NAME() << " : U";
            app()->up();
            return;
        }
        if (event->key() == Qt::Key_D) {
            LOG_INFO << FUNCTION_NAME() << " : D";
            app()->down();
            return;
        }
        if (event->key() == Qt::Key_P) {
            LOG_INFO << FUNCTION_NAME() << " : P";
            app()->perturb();
            return;
        }
    }

    int captureRate = 0;
    int captureID = 0;
    void Window::takeCapture() {
        // if (!actPlay()->isChecked() && !actAnim()->isChecked()) {
        //     return;
        // }
        // if (!actCapture()->isChecked()) {
        //     return;
        // }
        captureRate++;
        if (captureRate % 10 != 1) {
            return;
        }
        
        // std::string filename = (boost::format("./captures/capture%04d.png") % captureID).str();
        std::string filename = (boost::format("./captures/%s.%04d.png")
                                % app()->getFilestem()
                                % captureID).str();
        takeScreenshot(filename.c_str());
        captureID++;

    }

    void Window::takeScreenshot(const char* const filename) {
        // QPixmap pixmap;
        // pixmap = QPixmap::grabWindow(this->winId());
        // // pixmap = pixmap.scaled(1200, 800);
        // // pixmap.grabWidget(this->gl());
        // LOG_INFO << "size = " << pixmap.width() << " x " << pixmap.height();

        // pixmap.save(tr(filename), "png");
        // QImage img(gl()->size(), QImage::Format_RGB32);
        // QPainter painter(&img);
        // painter.setPen(Qt::blue);
        // painter.setFont(QFont("Arial", 30));
        // painter.drawText(rect(), Qt::AlignCenter, "Qt");

        QImage img = gl()->grabFrameBuffer();
        img = img.convertToFormat(QImage::Format_ARGB32);
        QRect rect(0, 10, 1280, 720);
        QImage sub = img.copy(rect);
        img = sub;
        if (app()->getFlagRemoveGround()) {
            QRgb key = img.pixel(1, 1);

            for (int i = 0; i < img.width(); i++) {
                for (int j = 0; j < img.height(); j++) {
                    QRgb c = img.pixel(i, j);
                    if (c == key) {
                        QRgb cc = qRgba( qRed(c), qGreen(c), qBlue(c), 0);
                        img.setPixel(i, j, cc);
                    }
                }
            }
        }
        // {
        //     QImage tr = img.createMaskFromColor( img.pixel(10, 10),
        //                                          Qt::MaskInColor );
        //     img = tr;
        // }
        // img.fill(qRgba(0,0,0,0));
     // gl()->render(&painter);
        img.save(filename);
        LOG_INFO << FUNCTION_NAME() << " : [" << filename << "]";
    }

    bool Window::isRenderingUI() {
        return actions["UI"]->isChecked();
    }

} // namespace gui
