//#ifndef __HLOG_HEADER__
//#define __HLOG_HEADER__
//
//#include <iostream>
//#include <vector>
//
//#if defined(__clang__)
//	/* Clang/LLVM. ---------------------------------------------- */
//
//#elif defined(__ICC) || defined(__INTEL_COMPILER)
//	/* Intel ICC/ICPC. ------------------------------------------ */
//
//#elif defined(__GNUC__) || defined(__GNUG__)
//	/* GNU GCC/G++. --------------------------------------------- */
//#define FUNC() __FUNCTION__
//#define FILE() __FILE__
//#define LINE() __LINE__
//
//#elif defined(__HP_cc) || defined(__HP_aCC)
//	/* Hewlett-Packard C/aC++. ---------------------------------- */
//
//#elif defined(__IBMC__) || defined(__IBMCPP__)
//	/* IBM XL C/C++. -------------------------------------------- */
//
//#elif defined(_MSC_VER)
//	/* Microsoft Visual Studio. --------------------------------- */
//#define FUNC() __FUNCSIG__ 
//
//#elif defined(__PGI)
//	/* Portland Group PGCC/PGCPP. ------------------------------- */
//
//#elif defined(__SUNPRO_C) || defined(__SUNPRO_CC)
//	/* Oracle Solaris Studio. ----------------------------------- */
//#endif
//
//#define MEANINGLESS 100
//#define INFO 3
//#define WARNING 2
//#define ERROR 1 
//#define FATAL 0
//
//#define LOG(lvl) (*(hlog::HLogger::instance()->startNewLine( (lvl), FILE(), LINE(), FUNC() )))
//#define CHECK_NOTNULL(ptr) (*(hlog::HLogger::instance()->startNewLine( (ptr != NULL) ? MEANINGLESS : ERROR , FILE(), LINE(), FUNC() , "ASSERTION ERROR: "#ptr" is NULL" )))
//#define CHECK_EQ(lhs, rhs) (*(hlog::HLogger::instance()->startNewLine( ((lhs) == (rhs)) ? MEANINGLESS : ERROR , FILE(), LINE(), FUNC(), "ASSERTION ERROR: "#lhs" == "#rhs" failed. ")))  << lhs << " vs." << rhs
//#define CHECK_LT(lhs, rhs) (*(hlog::HLogger::instance()->startNewLine( ((lhs) < (rhs)) ? MEANINGLESS : ERROR , FILE(), LINE(), FUNC(), "ASSERTION ERROR: "#lhs" < "#rhs" failed. "))) << lhs << " vs." << rhs
//#define LOG_EVERY_N(lvl, cnt) static int hlog_counter_##__LINE__ = -1; ++hlog_counter_##__LINE__; (*(hlog::HLogger::instance()->startNewLine( (hlog_counter_##__LINE__ % cnt) == 0 ? lvl : MEANINGLESS, FILE(), LINE(), FUNC() )))
//
//namespace hlog {
//    bool init(int security_level = INFO);
//
//    class HLogger : public std::ostream {
//    public:
//        static HLogger* g_instance;
//        static HLogger* instance();
//
//        HLogger();
//        virtual ~HLogger();
//
//        bool addStdoutStream(int lvl);
//        bool addFileStream(const char* const filename, int lvl);
//        
//        HLogger* startNewLine(int _lvl,
//                              const char* const file,
//                              int line,
//                              const char* const func,
//                              const char* const msg = NULL);
//
//        std::vector<std::ostream*> streams;
//        std::vector<int> stream_levels;
//        int current_level;
//        bool enoughToLog(int idx) { return (current_level <= stream_levels[idx]); }
//    };
//
//    template<class T>
//    HLogger& operator << (HLogger& hout, T data) {
//        // if (!hout.enoughToLog()) return hout;
//        // for  (std::vector<std::ostream*>::iterator i = hout.streams.begin();
//        //       i != hout.streams.end(); ++i) {
//        for (int i = 0; i < hout.streams.size(); i++) {
//            if (!hout.enoughToLog(i)) {
//                continue;
//            }
//            std::ostream* pOut = hout.streams[i];
//            *pOut << data << std::flush;
//        }
//        return hout;
//    }
//
//    HLogger& operator<< (HLogger& hout, std::ostream& (*pfun)(std::ostream&));
//    // {
//    //     for  (std::vector<std::ostream*>::iterator i = hout.streams.begin();
//    //           i != hout.streams.end(); ++i) {
//    //         std::ostream* pOut = *i;
//    //         pfun(*pOut);
//    //     }
//    //     return hout;
//    // }
//
//} // namespace hlog
//
//
//#endif // #ifndef __HLOG_HEADER__
