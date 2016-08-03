//#include "HLog.h"
//#include <iostream>
//#include <fstream>
//#include <string>
//#include <cstring>
//using namespace std;
//
//
//
//
//namespace hlog {
//    HLogger* HLogger::g_instance = NULL;
//
//    bool init(int security_level) {
//        HLogger::g_instance = new HLogger();
//
//        HLogger::g_instance->addStdoutStream(INFO);
//        HLogger::g_instance->addFileStream("log.info.txt", INFO);
//        HLogger::g_instance->addFileStream("log.warning.txt", WARNING);
//        HLogger::g_instance->addFileStream("log.error.txt", ERROR);
//        HLogger::g_instance->addFileStream("log.fatal.txt", FATAL);
//
//        return true;
//    }
//
//
//    HLogger* HLogger::instance() {
//        if (g_instance == NULL) {
//            hlog::init();
//        }
//        return g_instance;
//    }
//
//    bool HLogger::addStdoutStream(int lvl) {
//        streams.push_back(&cout);
//        stream_levels.push_back(lvl);
//        return true;
//    }
//    
//    bool HLogger::addFileStream(const char* const filename, int lvl) {
//        ofstream* pfout = new ofstream(filename);
//        if (pfout->is_open() == false) {
//            return false;
//        }
//        streams.push_back(pfout);
//        stream_levels.push_back(lvl);
//        return true;
//    }
//
//    HLogger* HLogger::startNewLine(int _lvl,
//                                   const char* const file,
//                                   int line,
//                                   const char* const func,
//                                   const char* const msg) {
//
//        this->current_level = _lvl;
//        std::string shorten_filename;
//        int n = strlen(file);
//        for (int i = 0; i < n; i++) {
//            if (file[i] == '/' || file[i] == '\\') {
//                shorten_filename = "";
//            } else {
//                shorten_filename += file[i];
//            }
//        }
//
//        std::string levelname = "?";
//        switch(_lvl) {
//        case INFO: levelname = "I"; break;
//        case WARNING: levelname = "W"; break;
//        case ERROR: levelname = "E"; break;
//        case FATAL: levelname = "F"; break;
//        }
//
//        time_t rawtime;
//        struct tm * timeinfo;
//
//        time (&rawtime);
//        timeinfo = localtime (&rawtime);
//        std::string timestr( asctime(timeinfo) );
//        timestr = timestr.substr(11, 8);
//
//        (*this) << endl;
//        (*this)<< "[" << levelname << timestr << " " << shorten_filename << ":" << line << " " << func << "()" << "] ";
//        if (msg) {
//            (*this) << msg;
//        }
//        return this;
//    }
//        
//    HLogger::HLogger() {
//    }
//    
//    HLogger::~HLogger() {
//    }
//
//    HLogger& operator<< (HLogger& hout, std::ostream& (*pfun)(std::ostream&))
//    {
//        for (int i = 0; i < hout.streams.size(); i++) {
//            if (!hout.enoughToLog(i)) {
//                continue;
//            }
//            std::ostream* pOut = hout.streams[i];
//        // if (!hout.enoughToLog()) return hout;
//        // for  (std::vector<std::ostream*>::iterator i = hout.streams.begin();
//        //       i != hout.streams.end(); ++i) {
//        //     std::ostream* pOut = *i;
//            pfun(*pOut);
//        }
//        return hout;
//    }
//
//    // HLogger& operator << (HLogger& log, 
//
//} // namespace hlog
