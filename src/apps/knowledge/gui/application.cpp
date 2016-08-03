#include "application.h"

#include <map>

#include "kinematics/FileInfoDof.h"
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/SkeletonDynamics.h"
#include "renderer/OpenGLRenderInterface.h"
#include "utils/Paths.h"
#include "utils/UtilsRotation.h"
#include "toolkit/Toolkit.h"
#include "toolkit/Moreeigen.h"
#include "common/app_cppcommon.h"
#include "common/render_cppcommon.h"
#include "common/fullbody1.h"

#include "mpiclient/mpiclient"

// #include "controller/controller"
// #include "controller/SimPack.h"

#include "controller/knowledge.h"
#include "controller/simpack.h"
#include "controller/app_composite_controller.h"
#include "controller/rapidxml_helper.h"
#include "controller/cost.h"

#include "operation/interpreter.h"
// #include "operation/op_controller.h"
#include "solver/solver.h"
#include "solver/problem.h"
// #include "interpreter.h"
#include "GuiKinController.h"
#include "motion.h"
#include "utils/Timer.h"
#include "utils/MayaExportSkeleton.h"
#include "utils/MayaExportMotion.h"

#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/support/date_time.hpp>

namespace gui {

    Application::Application()
        : MEMBER_INIT_NULL(ri)
          // , MEMBER_INIT_NULL(simpack)
          // , MEMBER_INIT_NULL(interpreter)
        , MEMBER_INIT_NULL(kn)
          // , MEMBER_INIT_NULL(sim)
          // , MEMBER_INIT_NULL(replay)
          // , MEMBER_INIT_NULL(con)
        , MEMBER_INIT_NULL(kinect)
        , currentMotionArraySolverVersion(-2)
        , MEMBER_INIT(isPlaying, false)
    {

        namespace logging = boost::log;
        namespace src = boost::log::sources;
        namespace expr = boost::log::expressions;
        namespace keywords = boost::log::keywords;
        logging::add_file_log
            (
                keywords::file_name = "log_gui.txt",
                keywords::format = "[%TimeStamp%]: %Message%"
                
                );
        logging::add_common_attributes();

        logging::add_console_log(cout,
                                 keywords::format =
                                 (
                                     expr::stream
                                     << "[" << expr::format_date_time< boost::posix_time::ptime >(
                                         "TimeStamp", "%Y-%m-%d %H:%M:%S.%f") << "]"
                                     << "<" << logging::trivial::severity
                                     << "> : " << expr::smessage
                                     )
            );


        motionTargetPose = new Motion();
        isSimTerminated = false;
        useMotionArrays = false;
        flagDebugRendering = false;
        flagGroundToChess = true;
        flagGroundToMatt = false;
        flagRemoveGround = false;
        flagUIButtonChecked = true;
    }
    
    Application::~Application() {
        MEMBER_RELEASE_PTR(kinect);
        // MEMBER_RELEASE_PTR(interpreter);
        // MEMBER_RELEASE_PTR(con);
        // MEMBER_RELEASE_PTR(replay);
        // MEMBER_RELEASE_PTR(simpack);
        MEMBER_RELEASE_PTR(ri);
    }

    void Application::init(int window_width, int window_height) {
        namespace rx = rapidxml;
        std::string knowledge_filename("");
        std::map<std::string, std::string> opti_options;
        try {
            rx::file<char> file(RTQL8_DATA_PATH"/gui.xml");
            rx::xml_document<char> doc;
            doc.parse<0>(file.data());

            rx::xml_node<char>* root = doc.first_node("gui");

            // Parse knowledge file
            knowledge_filename
                = std::string(RTQL8_DATA_PATH)
                + root->first_node("knowledge")->value();
            knfilename = knowledge_filename;
            LOG_INFO << "knowledge_filename = " << knowledge_filename;
            // Parse motion array
            useMotionArrays = rx::attr_bool(root->first_node("motionarrays"), "use");
            LOG_INFO << "useMotionArrays = " << useMotionArrays;

            flagDebugRendering = rx::attr_bool(root->first_node("debugrendering"), "value");
            LOG_INFO << "flagDebugRendering = " << flagDebugRendering;

            flagGroundToChess = rx::attr_bool(root->first_node("groundToChess"), "value");
            LOG_INFO << "flagGroundToChess = " << flagGroundToChess;

            flagGroundToMatt = rx::attr_bool(root->first_node("groundToMatt"), "value");
            LOG_INFO << "flagGroundToMatt = " << flagGroundToMatt;

            flagRemoveGround = rx::attr_bool(root->first_node("removeGround"), "value");
            LOG_INFO << "flagRemoveGround = " << flagRemoveGround;


            flagUIButtonChecked = rx::attr_bool(root->first_node("uiButtonChecked"), "value");
            LOG_INFO << "flagUIButtonChecked = " << flagUIButtonChecked;


            // Parse optimization options
            rx::xml_node<char>* node_opti = root->first_node("optimization");
            if (node_opti) {
                for (rx::xml_attribute<char>* attr = node_opti->first_attribute();
                     attr; attr = attr->next_attribute()) {
                    std::string name  = attr->name();
                    std::string value = attr->value();
                    opti_options[name] = value;
                }
            }
            
        } catch (const std::exception& e) {
            LOG_ERROR << FUNCTION_NAME() << " : error = " << e.what() << endl;
            exit(0);
        }


        set_kn( new controller::Knowledge() );
        bool result = kn()->loadXML(knowledge_filename.c_str());
        // bool result = kn()->loadXML(RTQL8_DATA_PATH"/knowledge/output.xml");

        LOG_INFO << endl << kn()->toString();
        LOG_INFO << "result = " << result;
        LOG_INFO << "activated = " << con()->activated_time();
        kn()->simpack()->reset();
        LOG_INFO << "reset OK";
        LOG_INFO << "activated = " << con()->activated_time();

        // kn()->saveXML(RTQL8_DATA_PATH"/knowledge/output.xml");
        // exit(0);

        set_ri( new rtql8::renderer::OpenGLRenderInterface() );
        LOG_INFO << "create rendering interface OK";

        set_replay( new rtql8::toolkit::SimReplay(sim()->getSimConfig()) );
        pushReplayFrame();
        // replay()->frames.push_back( sim()->getSimState() );
        LOG_INFO << "create replay structure OK";

        const int SKEL_INDEX = 1;
        std::string skel_filename = sim()->getSimConfig().skels[SKEL_INDEX].filename;
        Motion::initSkeleton( skel_filename.c_str() );

        {
            LOG_INFO << "loading bar...";
            barModel = new rtql8::kinematics::FileInfoSkel<
                rtql8::dynamics::SkeletonDynamics>();
            bool result = barModel->loadFile(RTQL8_DATA_PATH"skel/bar.skel");
            LOG_INFO << "loading bar..." << result;
        }
        {
            LOG_INFO << "loading matt...";
            mattModel = new rtql8::kinematics::FileInfoSkel<
                rtql8::dynamics::SkeletonDynamics>();
            bool result = mattModel->loadFile(RTQL8_DATA_PATH"skel/ground4_matt.skel");
            LOG_INFO << "loading matt..." << result;
        }
        
        LOG_INFO << FUNCTION_NAME() << " OK";

#ifdef KN_COMPILE_SOLVER
        set_solver( new solver::Solver(kn()) );
        LOG_INFO << "Solver configuration.......";
        for (std::map<std::string, std::string>::iterator i = opti_options.begin();
             i != opti_options.end(); i++) {
            using boost::lexical_cast;
            using rtql8::toolkit::moreeigen::convertStringToVectorXd;
            
            std::string name  = i->first;
            std::string value = i->second;
            if (name == "mu") {
                solver()->set_mu( lexical_cast<int>(value) );
                LOG_INFO << "mu = " << solver()->mu();
            } else if (name == "lambda") {
                solver()->set_lambda( lexical_cast<int>(value) );
                LOG_INFO << "lambda = " << solver()->lambda();
            } else if (name == "sigma") {
                solver()->set_sigma( lexical_cast<double>(value) );
                LOG_INFO << "sigma = " << solver()->sigma();
            } else if (name == "T") {
                solver()->set_sampled_tasks( convertStringToVectorXd(value.c_str()) );
                LOG_INFO << "T = " << IO(solver()->sampled_tasks());
            } else if (name == "clusterMode") {
                solver()->set_clusterMode( lexical_cast<int>(value) );
                LOG_INFO << "clusterMode = " << solver()->clusterMode();
            } else if (name == "clusterMinDist") {
                solver()->set_clusterMinDist( lexical_cast<double>(value) );
                LOG_INFO << "clusterMinDist = " << solver()->clusterMinDist();
            } else {
                LOG_INFO << "unrecognized option: " << name << " = " << value;
            }
        }
        LOG_INFO << "Solver initialization OK";
#else
        LOG_INFO << "Solver is not initialzed due to the compile option";
#endif



#ifdef KN_USE_KINECT
        LOG_INFO << "start to initialize Kinect";
        GuiKinController* kin = new GuiKinController(this);
        LOG_INFO << "Kinect allocated";
        int kinInitRetVal = kin->onInit();
        LOG_INFO << "Kinect initialized = " << kinInitRetVal;
        kin->initWinVals(ri(), window_width, window_height);
        LOG_INFO << "Kinect variable set";
        // vector<int> tmpVecButs = vector<int>(3);				//idx 0 is btns, 1(if present) is sldrs, 2(if present) is crank controls
        // tmpVecButs[0] = 5; tmpVecButs[1] = 3; tmpVecButs[2] = 1;
        // kin->buildUI(tmpVecButs);
        LOG_INFO << "Kinect buildUI";
        set_kinect( kin );
        LOG_INFO << "Kinect initialization OK";
#else
        LOG_INFO << "Kinect is not initialized due to the compile option";
#endif

        
    }

    void Application::reset() {
        simpack()->reset();
        clearReplayFrames();
        // replay()->frames.clear();
        LOG_INFO << FUNCTION_NAME() << " OK";
    }

    void Application::reset(double target) {
        this->myTarget = target;
        con()->setParamsWithTask(target);

        // Eigen::VectorXd params = Eigen::VectorXd::Random(4);
        // cout << "params = " << IO(params) << endl;
        // simpack()->con()->setParams(params);

        simpack()->reset();
        clearReplayFrames();
        // replay()->frames.clear();
        isSimTerminated = false;
        LOG_INFO << FUNCTION_NAME() << " : " << target << " OK";
    }


    bool Application::step() {
        if (isSimTerminated) {
            // con()->control();
            con()->controlWithSPDOnly();
            sim()->step();
            pushReplayFrame();
            // replay()->frames.push_back( sim()->getSimState() );
            return true;
        }

        bool result = simpack()->step();
        pushReplayFrame();
        // replay()->frames.push_back( sim()->getSimState() );

        // cout << sim()->getSkelState(1).minBarDist << endl;


        if (!result) {
            LOG_INFO << "controller = " << con()->toString();
            // LOG_INFO << "evaluate = " << con()->evaluate(0.5);
            // LOG_INFO << "final = " << sim()->getSimState();
            double task  = 0.0;
            double error = 0.0;
            con()->evaluateTaskAndError(&task, &error);
            LOG_INFO << "task & error = " << task << " " << error;
            isSimTerminated = true;
        }

        return result;
    }

    void drawCov(const Eigen::MatrixXd& C) {
        using namespace Eigen;
        glBlock() {
            JacobiSVD<MatrixXd> svd(C, ComputeFullU | ComputeFullV);
            MatrixXd U = svd.matrixU();
            VectorXd d = svd.singularValues();
            glTranslated(0.0, 1.0, 0.0);
            // cout << "C = " << C << " U = " << U << " d = " << d << endl;

            Matrix4d M = Matrix4d::Identity();
            M.topLeftCorner(3, 3) = U;
            rtql8::toolkit::glMultMatrixd(M);
            glScaled(d(0), d(1), d(2));
            glutSolidSphere(1.0, 16, 16);
        }
    }

    Eigen::Matrix3d interpolate(const Eigen::Matrix3d& A,
                                const Eigen::Matrix3d& B,
                                double w) {
        using namespace Eigen;
        LLT<Matrix3d> lltA(A);
        LLT<Matrix3d> lltB(B);
        Matrix3d LA = lltA.matrixL();
        Matrix3d LB = lltB.matrixL();
        Matrix3d Lw = (1.0 - w) * LA + w * LB;
        Matrix3d W = Lw * Lw.transpose();
        return W;
    }
                                
    

    double weight = 0.0;
    void Application::render(bool renderUI) {
        // using namespace rtql8::utils::rotation;
        // Eigen::Matrix3d D1 = Eigen::Vector3d(0.5, 0.5, 0.1).asDiagonal();
        // Eigen::Matrix3d R1 = eulerToMatrix(Eigen::Vector3d(1.3, 0.7, 0.5), XYZ);
        // Eigen::Matrix3d C1 = R1 * D1 * R1.transpose();
                                                  
        // Eigen::Matrix3d D2 = Eigen::Vector3d(0.6, 0.1, 0.3).asDiagonal();
        // Eigen::Matrix3d R2 = eulerToMatrix(Eigen::Vector3d(1.5, -2.5, 3.8), XYZ);
        // Eigen::Matrix3d C2 = R2 * D2 * R2.transpose();

        // double t = weight;
        // cout << " t = " << t << endl;
        // weight += 0.01;
        // if (weight > 1.0) weight = 0.0;

        // glBlock() {
        //     glTranslated(2.0 * 0.0 - 1.0, 0.0, 0.0);
        //     glColor4d(1, 0, 0, 0.9);
        //     drawCov(C1);
        // }
        // glBlock() {
        //     glTranslated(2.0 * 1.0 - 1.0, 0.0, 0.0);
        //     glColor4d(0, 0, 1, 0.9);
        //     drawCov(C2);
        // }
        // Eigen::Matrix3d W = interpolate(C1, C2, t);
        // glBlock() {
        //     glTranslated(2.0 * t - 1.0, 0.0, 0.0);
        //     glColor4d(1 - t, 0, t, 0.9);
        //     drawCov(W);
        // }

        // glBlock() {

        //     glTranslated(0.0, -0.025, 0.0);
        //     renderChessBoard(20, 20, 40.0, 40.0);
        // }
        // return;
        // // Render the global axis
        // renderAxis();

        

        // Render skeleton
        bool isIKMode = kinect() && kinect()->isIKMode();
        for (int i = 0; i < sim()->nSkels(); i++) {
            if (i == 0) {
                if (flagGroundToChess) continue;
                if (flagRemoveGround) continue;
            } else if (i == 1) {
                if (isIKMode || !isPlaying()) continue;
            }
            sim()->skel(i)->draw(ri());
        }

        if (barModel && simpack()->hasBar()) {
            Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
            q.head<3>() = simpack()->getBarPos();
            barModel->getSkel()->setPose(q);
            barModel->getSkel()->draw(ri());
        }

        if (flagGroundToChess && !flagRemoveGround) {
            glBlock() {
                glTranslated(0.0, -0.025, 0.0);
                renderChessBoard(20, 20, 40.0, 40.0);
            }
        }

        if (flagGroundToMatt && !flagRemoveGround) {
            glBlock() {
                mattModel->getSkel()->draw(ri());

                // glTranslated(1.0, -0.025, 0.0);
                // glColor4d(0.0, 0.0, 0.7, 0.95);
                // glScaled(2.0, 0.05, 1.2);
                // glutSolidCube(1.0);
            }
        }


        if (!isIKMode) {
            // Render target pose
            if (isPlaying()) {
                glBlock() {
                    glPushAttrib( GL_ALL_ATTRIB_BITS );
                    glPolygonMode(GL_FRONT_AND_BACK,  GL_LINE);
                    glTranslated(2.0, 0.5, 0.0);
                    Eigen::VectorXd& targetPose = simpack()->targetPose();
                    if (flagDebugRendering) {
                        motionTargetPose->render(ri(), targetPose);
                    }
                    glPopAttrib();
                }
            }
            if (!isPlaying()) {
                // Render motions
                Motion::globalStep(motions);
                for (int i = 0; i < motions.size(); i++) {
                    Motion& motion = motions[i];
                    if (motion.packed() == false) {
                        continue;
                    }
                    motion.step();
                    motion.render(ri());
                    if (flagDebugRendering) {
                        motion.renderHeadTrajectory();
                    }
                }
            }
        }


        if (isPlaying() && flagDebugRendering) {
            using namespace fullbody1;
            using namespace rtql8::dynamics;
            using namespace rtql8::utils::rotation;
            BodyNodeDynamics* node = dynamic_cast<
                BodyNodeDynamics*>(sim()->skel(1)->getNode(0));
            Eigen::Vector3d p = node->getWorldCOM();

            const Eigen::VectorXd& q = sim()->getSimState().skels[1].q;
            Eigen::Vector3d em = q.segment<3>(3);
            Eigen::Quaterniond quat = expToQuat(em);
            Eigen::Matrix3d R = quatToMatrix(quat);
            Eigen::Vector3d euler = matrixToEuler(R, XZY);
            double yRot = euler(2);
            Eigen::Matrix3d M = eulerToMatrixY(yRot);
            Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
            T.topLeftCorner<3, 3>() = M;

            glBlock() {
                rtql8::toolkit::glTranslated(p);
                rtql8::toolkit::glMultMatrixd(T);
                rtql8::toolkit::glScaled(0.05);
                renderAxis();
            }

            


        }

        // Render contact forces
        if (flagDebugRendering) {
            glPushAttrib(GL_LINE_BIT);
            rtql8::toolkit::glColorRGB(255, 105, 180);
            glLineWidth(1.0);
            const Eigen::VectorXd &contacts = sim()->getSimState().contacts;

            for (int i = 0; i < sim()->getSimState().nT(); i++) {
                using rtql8::toolkit::glVertex3d;

                Eigen::Vector3d p = contacts.segment<3>(i * 6 + 0);
                Eigen::Vector3d f = contacts.segment<3>(i * 6 + 3);
                ::glBegin(GL_LINES);
                glVertex3d(p);
                glVertex3d(p - 0.01 * f);
                ::glEnd();

                glBlock() {
                    rtql8::toolkit::glTranslated(p);
                    glutSolidSphere(0.05, 10, 10);
                
                }
            }
            glPopAttrib();
        }

        if (isPlaying() && flagDebugRendering) {
            const Eigen::Vector3d& COM = sim()->getSimState().skels[1].COM;
            glBlock() {
                rtql8::toolkit::glTranslated(COM);
                rtql8::toolkit::glColorRGB(160, 30, 240);
                glutSolidSphere(0.1, 4, 2);
            }
            // Render the COM velocity
            glBlock() {
                const Eigen::Vector3d& P = sim()->getSimState().skels[1].P / 50.0;
                rtql8::toolkit::glColorRGB(190, 30, 200);
                // rtql8::toolkit::glTranslated(COM);
                // glutSolidSphere(0.1, 4, 2);
                rtql8::toolkit::globjects::renderArrow2(
                    COM, COM + 0.2 * P, 0.02, 0.09, 0.08);
            }
            
        // Render the center of pressure
            using namespace fullbody1;
            const Eigen::Vector3d& COP = con()->APOS(2, l_toe, r_toe);
            glBlock() {
                rtql8::toolkit::glTranslated(COP);
                rtql8::toolkit::glColorRGB(160, 30, 240);
                glutSolidSphere(0.1, 4, 2);
            }
            // Head trajectories
            glColor4d(0.0, 0.9, 0.9, 0.5);
            for (int i = 0; i < replayHeadPositions.size(); i += 100) {
                glBlock() {
                    rtql8::toolkit::glTranslated( replayHeadPositions[i] );
                    glutSolidSphere(0.03, 4, 4);
                }
            }
            

        }

#ifdef KN_USE_KINECT
        if (kinect() && renderUI) {
            glBlock() {
                glDisable(GL_LIGHTING);
                kinect()->onDrawUpdate(ri());
                glEnable(GL_LIGHTING);
            }
        }        
#endif
    }

    bool Application::interpret(const char* const instruction) {
        bool runopt = false;
        bool resetMotions = false;
        operation::Interpreter interpreter(kn());
        bool ret = interpreter.parse(instruction, &runopt, &resetMotions);
        if (std::string(instruction) == "optimize") {
            ret = true;
            runopt = true;
        }
        // bool ret = true;

#ifdef KN_COMPILE_SOLVER
        if (runopt && solver()) {
            solver::Problem* p = new solver::Problem();
            solver()->setProblem(p);
            currentMotionArraySolverVersion = -1;
            solver()->set_motionUpdatedIteration(currentMotionArraySolverVersion);
            LOG_INFO << "Optimization Problem Placed!!!!!";
        } else if (resetMotions) {
            currentMotionArraySolverVersion = -2;
            LOG_INFO << "Just reset motions!!!";
        }
#endif
        clearMotionArrays();
        return ret;
    }

    bool Application::interpret(boost::format& instruction) {
        return interpret(instruction.str().c_str());
    }

    std::string Application::printString() {
        const Eigen::VectorXd& q = sim()->getSimState().skels[1].q;
        std::stringstream sout;
        sout << IO(q);
        return sout.str();
    }
    
    int Application::replayNumFrames() {
        return replay()->frames.size();
    }

    void Application::replayUpdateToFrame(int idx) {
        if (idx < 0 || replayNumFrames() <= idx) {
            LOG_WARNING << FUNCTION_NAME() << " : invalid index = " << idx;
            return;
        }
        sim()->setSimState( replay()->frames[idx] );
        simpack()->setTargetPose( replayTargetPoses[idx] );
    }

    void Application::clearReplayFrames() {
        replay()->frames.clear();
        replayTargetPoses.clear();        
        replayHeadPositions.clear();
    }

    void Application::pushReplayFrame() {
        replay()->frames.push_back( sim()->getSimState() );
        replayTargetPoses.push_back( simpack()->targetPose() );
        replayHeadPositions.push_back( con()->POS(fullbody1::head) );
    }

    void Application::fillTheReplays() {
        while(replayTargetPoses.size() < replayNumFrames()) {
            replayTargetPoses.push_back( simpack()->targetPose() );
            replayHeadPositions.push_back( con()->POS(fullbody1::head) );
        }

    }

    void Application::perturb() {
        using namespace fullbody1;
        using namespace rtql8::dynamics;
        BodyNodeDynamics* node = dynamic_cast<
            BodyNodeDynamics*>(sim()->skel(1)->getNode(head));
        Eigen::Vector3d p(0, 0, 0);
        Eigen::Vector3d f(-2000, 0, 0);
        node->addExtForce(p, f);

        LOG_INFO << FUNCTION_NAME() << " OK";
    }

    void Application::set_solver_type(int evaluatorType) {
#ifdef KN_COMPILE_SOLVER
        solver()->set_evaluatorType(evaluatorType);
        solver()->setEvaluator();
        LOG_INFO << FUNCTION_NAME() << " OK";
#endif
    }

    void Application::solver_run(int evaluatorType) {
#ifdef KN_COMPILE_SOLVER
        solver()->set_evaluatorType(evaluatorType);
        solver()->setEvaluator();
        solver()->run();
#endif
        LOG_INFO << FUNCTION_NAME() << " OK";
    }

    void Application::solver_stop() {
#ifdef KN_COMPILE_SOLVER
        solver()->stop();
#endif
        LOG_INFO << FUNCTION_NAME() << " OK";
    }

    void Application::solver_request_next_style() {
#ifdef KN_COMPILE_SOLVER
        if (solver()) {
            clearMotionArrays();
            currentMotionArraySolverVersion = -1;
            solver()->set_motionUpdatedIteration(-1);
            solver()->nextStyle();
        }
#endif
        LOG_INFO << FUNCTION_NAME() << " OK";
    }

    void Application::solver_request_terminate() {
#ifdef KN_COMPILE_SOLVER
        if (solver()) {
            solver()->terminateOpt();
        }
#endif
        LOG_INFO << FUNCTION_NAME() << " OK";
    }

    int Application::solver_current_style() {
#ifdef KN_COMPILE_SOLVER
        if (solver()) {
            return solver()->currentStyle();
        }
#endif
        return -1;
    }

    int Application::solver_num_styles() {
#ifdef KN_COMPILE_SOLVER
        if (solver()) {
            return solver()->numStyles();
        }
#endif
        return -1;
    }

    void Application::down() {
        // leg()->down();
        LOG_INFO << FUNCTION_NAME() << " OK";
    }

    void Application::up() {
        // leg()->up();
        LOG_INFO << FUNCTION_NAME() << " OK";
    }

    void Application::saveController(const char* const filename) {
        kn()->saveXML(filename);
        LOG_INFO << FUNCTION_NAME() << " OK";
    }
    
    void Application::loadController(const char* const filename) {
        LOG_INFO << "deleting the current knowledge...";
        delete kn();
        set_kn( new controller::Knowledge() );
        LOG_INFO << "loading the new knowledge...";
        bool result = kn()->loadXML(filename);
        LOG_INFO << "result = " << result;
        LOG_INFO << "xmldoc = " << kn()->xmldoc();
        kn()->simpack()->reset();
        LOG_INFO << "reset OK";
#ifdef KN_COMPILE_SOLVER
        solver()->set_kn( kn() );
        LOG_INFO << "set a new knowledge to the solver";
#endif
        
        clearMotionArrays();
        kn()->pushHint("clear");
    }

    double Application::getTaskParam() {
        return con()->getTaskParam();
    }

    void Application::setControllerParams(const Eigen::VectorXd& params) {
        // if (con()->dim() != params.size()) {
        //     int lhs = con()->dim();
        //     int rhs = params.size();
        //     LOG_WARNING << FUNCTION_NAME() << " : invalid dim. " << lhs << ", " << rhs;
        //     return;
        // }
        // con()->set_params(params);
        LOG_INFO << FUNCTION_NAME() << " OK : params = " << IO(params);
    }

    void Application::optimize() {
        LOG_INFO << FUNCTION_NAME() << " OK";
    }


    void Application::test() {

        LOG_INFO << FUNCTION_NAME() << " OK";
    }

// Mouse event functions
    bool Application::click(int button, int state, int x, int y) {
	//button : left : 0, middle : 1, right : 2
	//state : down : 0, up : 1
#ifdef KN_USE_KINECT
        if (kinect()) {
            int d = 0;
            return kinect()->checkUIElements(x, y, d, true, button, 1 - state);
        }
#endif
        return false;
    }

    bool Application::drag(int x, int y) {
#ifdef KN_USE_KINECT
        if (kinect()) {
            int d = 0;
            return kinect()->checkUIElements(x, y, d, true, 0, 2);
        }
#endif
        return false;
    }
// Motion related functions
    void Application::clearMotionArrays() {
        motions.clear();
    }

    bool Application::onSimulationTerminated() {
        if (!useMotionArrays) {
            return false;
        }
        int n = motions.size();
        if (n > 0) {
            Motion& motion = motions[n - 1];
            if (motion.packed() == false) {
                motion.pack();
                motion.setHeadPositions(replayHeadPositions);
            }
        }

        int NUM_MOTIONS = (isConRanged()) ? 3 : 1;
        LOG_INFO << "NUM_MOTIONS = " << NUM_MOTIONS;
        if (n >= NUM_MOTIONS) {
            return false;
        } else {
            return true;
        }
    }

    void Application::onSimulationBegin() {
        if (!useMotionArrays) {
            return;
        }
        int n = motions.size();

        double task = 0.0;
        // Eigen::Vector3d offset(0.0, 0.0, -2.0);
        Eigen::Vector3d offset(0.0, 0.0, 0.0);
        if (isConRanged()) {
            switch(n) {
            case 0: task = 0.0; offset(0) = -2.0; break;
            case 1: task = 0.5; offset(0) =  0.0; break;
            case 2: task = 1.0; offset(0) =  2.0; break;
            }
        }
        
        Motion m(replay());
        m.set_offset( offset );
        motions.push_back(m);
        con()->setParamsWithTask( task );
        LOG_INFO << FUNCTION_NAME() << " task = " << task;

    }

    bool Application::autostart() {
        if (useMotionArrays == false) {
            return false;
        }
        
#ifdef KN_COMPILE_SOLVER
        // if (solver() && solver()->prob() != NULL) {
        if (solver()) {
            int optimized = solver()->motionUpdatedIteration();
            int current   = currentMotionArraySolverVersion;
            // cout << "version = " << optimized << " " << current << endl;
            if (current < optimized) {
                LOG_INFO << "##################################################";
                LOG_INFO << "Current : " << current << " "
                         << "Optimized : " << optimized;
                LOG_INFO << "##################################################";
                clearMotionArrays();
                currentMotionArraySolverVersion = optimized;
                if (this->kinect()) {
                    this->kinect()->setOptProgressBar(currentMotionArraySolverVersion);
                }

                return true;
            }
            return false;
        }
#endif
        return (useMotionArrays) && (motions.size() == 0);
    }

    controller::SimPack* Application::simpack() {
        return kn()->simpack();
    }

    rtql8::toolkit::Simulator* Application::sim() {
        return simpack()->sim();
    }

    controller::AppCompositeController* Application::con() {
        return simpack()->con();
    }

    bool Application::isConRanged() {
        return (con()->cost()->isRanged());
    }

    void Application::exportSkel() {
        using namespace rtql8::utils::mayaexports;
        MayaExportSkeleton ex;
        std::ofstream fout("skeleton.ma");
        ex.exportMayaAscii(sim()->skel(1), fout);
        LOG_INFO << FUNCTION_NAME() << " OK";
    }
    
    void Application::exportMotion() {
        using namespace rtql8::kinematics;
        using namespace rtql8::utils::mayaexports;
        FileInfoDof file(sim()->skel(1));
        for (int i = 0; i < replayNumFrames(); i += 10) {
            Eigen::VectorXd q = replay()->frames[i].skels[1].q;
            file.addDof(q);
        }
        MayaExportMotion ex(sim()->skel(1), &file);
        ex.exportMayaAnim("motion.anim", 0, file.getNumFrames() - 1, "", sim()->skel(1)->getNumNodes());
        LOG_INFO << FUNCTION_NAME() << " OK : " << file.getNumFrames();
    }

    std::string Application::getFilestem() {
        std::size_t dir_pos = knfilename.rfind("/");
        std::string filename = knfilename.substr(dir_pos + 1);
        return (boost::format("%s.%.2lf")
                % filename.substr(0, filename.length() - 4)
                % myTarget).str();

    }

    std::string Application::getFilename(const char* const ext) {
        std::string ss(ext);
        std::size_t dir_pos = knfilename.rfind("/");
        std::string filename = std::string(RTQL8_DATA_PATH) + "replay" + knfilename.substr(dir_pos);
        return (boost::format("%s.%.2lf.%s")
                % filename.substr(0, filename.length() - 4)
                % myTarget
                % ss ).str();
    }

    
} // namespace gui

