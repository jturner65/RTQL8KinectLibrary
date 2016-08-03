#ifndef MPISOLVER2_DBXMLWRITER_H
#define MPISOLVER2_DBXMLWRITER_H

#include <Eigen/Dense>
#include "common/app_hppcommon.h"

namespace mpisolver2 {
    class CommandCenter;
    class Database;
} // namespace mpisolver2

namespace mpisolver2 {
    class DBXMLWriter {
    public:
        DBXMLWriter(CommandCenter* _cmd);
        virtual ~DBXMLWriter();

        void write(); // Write to default path
        void write(const char* const filename);

    protected:
        Database* db();
    protected:
        MEMBER_PTR(CommandCenter*, cmd);

    };

} // namespace mpisolver2

#endif // #ifndef MPISOLVER2_DBXMLWRITER_H
