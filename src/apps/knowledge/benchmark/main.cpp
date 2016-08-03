#include <Eigen/Dense>

#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/SkeletonDynamics.h"
#include "renderer/OpenGLRenderInterface.h"
#include "utils/Paths.h"
#include "utils/UtilsRotation.h"
#include "utils/Timer.h"
#include "toolkit/Toolkit.h"
#include "common/app_cppcommon.h"
#include "common/render_cppcommon.h"
#include "common/fullbody1.h"

#include "controller/controller"
#include "controller/SimPack.h"

#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/support/date_time.hpp>

#include <iostream>
using namespace std;

int main() {
    cout << "start benchmark" << endl;
    namespace logging = boost::log;
    namespace src = boost::log::sources;
    namespace expr = boost::log::expressions;
    namespace keywords = boost::log::keywords;
    // logging::add_file_log
    //     (
    //         keywords::file_name = "log_gui.log",
    //         keywords::format = "[%TimeStamp%]: %Message%"
                
    //         );
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

    LOG_INFO << "start logging..";

    controller::SimPack* simpack = new controller::SimPack();
    simpack->init();
    LOG_INFO << "create SIMPACK OK";

    controller::RootController* con= simpack->con();
    LOG_INFO << "retrieve controller OK";

    Eigen::VectorXd params = Eigen::VectorXd::Zero(con->dim());
    params << -0.6, 0.0, 0.5, 1.0, 0.0, -0.5;

    // ofstream fout(RTQL8_ROOT_PATH"build/regression.txt");
    // LOG_INFO << "is open = " << fout.is_open();
    // fout << "leg0, leg1, arm0, arm1, vf0, vf1, Cdot.x, Cdot.y, Cdot.z" << endl;
    const int MAX_LOOP = 50;

    rtql8::utils::Timer timer("runs");

    for (int i = 0; i < MAX_LOOP; i++) {
        params.tail<2>().setRandom();
        LOG_INFO << i << " : params = " << IO(params);
        con->set_params(params);
        simpack->reset();
        timer.startTimer();
        simpack->simulate();
        timer.stopTimer();
        Eigen::Vector3d Cdot = con->COMdot();
        LOG_INFO << "COMdot = " << Cdot;

        // fout << params(0);
        // for (int j = 1; j < params.size(); j++) fout << ", " << params(j);
        // for (int j = 0; j < Cdot.size(); j++) fout << ", " << Cdot(j);
        // fout << endl;
    }
    // fout.close();
    timer.printScreen();

    delete simpack;
    
    return 0;
}
