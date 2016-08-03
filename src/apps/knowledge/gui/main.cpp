#include <QApplication>
#include "window.h"
#include "toolkit/CppCommon.h"
#include "utils/LoadOpengl.h"

#include <csignal>
#include <cstdio>
#include <cstdlib>


int main(int argc, char *argv[])
{
    // // google::ParseCommandLineFlags(&argc, &argv, true);
    // google::InitGoogleLogging((const char*)argv[0]);

    // // // Define logging flag
    // FLAGS_alsologtostderr = true;
    // FLAGS_minloglevel = INFO;
    // FLAGS_log_dir = "./glog/";

    glutInit(&argc, argv);

    QApplication app(argc, argv);
    gui::Window window;
    window.show();
    window.activateWindow();
    window.raise();
    return app.exec();
}
