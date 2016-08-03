#ifndef GUI_APPLICATION_H
#define GUI_APPLICATION_H

#include <vector>
#include <Eigen/Dense>
#include "common/app_hppcommon.h"
#include "boost/format.hpp"
#include "kinematics/FileInfoSkel.hpp"

namespace rtql8 {
    namespace renderer {
        class RenderInterface;
    } // namespace renderer

    namespace dynamics {
        class SkeletonDynamics;
        class ConstraintDynamics;
    } // namespace dynamics 

    namespace toolkit {
        struct SimReplay;
        class Simulator;
    } // namespace toolkit
} // namespace rtql8

namespace controller {
    class SimPack;
    class Knowledge;
    class AppCompositeController;
} // namespace controller

namespace gui {
    // class Interpreter;
    class GuiKinController;
    class Motion;
} // namespace gui

namespace solver {
    class Solver;
} // namespace solver

namespace gui {
    class Application {
    public:
        Application();
        virtual ~Application();

        void init(int window_width, int window_height);
        void reset();
        void reset(double target);
        bool step();
        void render(bool renderUI = true);

        bool interpret(const char* const instruction);
        bool interpret(boost::format& instruction);

        std::string printString();

        int replayNumFrames();
        void replayUpdateToFrame(int idx);
        void clearReplayFrames();
        void pushReplayFrame();
        void fillTheReplays();

        void set_solver_type(int evaluatorType);
        void solver_run(int evaluatorType);
        void solver_stop();
        void solver_request_next_style();
        void solver_request_terminate();
        int  solver_current_style();
        int  solver_num_styles();

        void perturb();
        void down();
        void up();

        void saveController(const char* const filename);
        void loadController(const char* const filename);

        double getTaskParam();
        void setControllerParams(const Eigen::VectorXd& params);
        void optimize();

        void test();

        void exportSkel();
        void exportMotion();

// Mouse event functions
        bool click(int button, int state, int x, int y);
        bool drag(int x, int y);

// Motion related functions
        void clearMotionArrays();
        bool onSimulationTerminated();
        void onSimulationBegin();
        bool autostart();
        bool isConRanged();
// Miscs
        bool getFlagUIButtonChecked() { return flagUIButtonChecked; }
        bool getFlagRemoveGround() { return flagRemoveGround; }

        std::string getFilestem();
        std::string getFilename(const char* const ext);
    public:
        controller::SimPack* simpack();
        rtql8::toolkit::Simulator* sim();
        controller::AppCompositeController* con();
        bool useMotionArrays;
    protected:
        MEMBER_PTR(controller::Knowledge*, kn);
        // MEMBER_PTR(controller::SimPack*, simpack);
        
        MEMBER_PTR(rtql8::renderer::RenderInterface*, ri);
        // MEMBER_PTR(rtql8::toolkit::Simulator*, sim);
        MEMBER_PTR(rtql8::toolkit::SimReplay*, replay);
        std::vector<Eigen::VectorXd> replayTargetPoses;
        std::vector<Eigen::Vector3d> replayHeadPositions;
        // MEMBER_PTR(controller::RootController*, con);
        // MEMBER_PTR(controller::LegPose*, leg);

        // MEMBER_PTR(Interpreter*, interpreter);
        MEMBER_PTR(GuiKinController*, kinect);
        MEMBER_PTR(solver::Solver*, solver);

        Motion* motionTargetPose;
        std::vector<Motion> motions;
        bool isSimTerminated;
        int  currentMotionArraySolverVersion;

        std::string knfilename;
        double myTarget;
        bool flagDebugRendering;
        bool flagGroundToChess;
        bool flagGroundToMatt;
        bool flagRemoveGround;
        bool flagUIButtonChecked;
        MEMBER_PTR(bool, isPlaying);

        rtql8::kinematics::FileInfoSkel<rtql8::dynamics::SkeletonDynamics>* barModel;
        rtql8::kinematics::FileInfoSkel<rtql8::dynamics::SkeletonDynamics>* mattModel;
    };

} // namespace gui

#endif // #ifndef GUI_APPLICATION_H
