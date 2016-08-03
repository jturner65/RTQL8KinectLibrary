#include "dbxmlwriter.h"

#include <algorithm>
#include <fstream>
#include <sstream>
#include "rapidxml.hpp"
#include "rapidxml_print.hpp"
#include "rapidxml_utils.hpp"
#include "boost/lexical_cast.hpp"

#include "utils/Paths.h"
#include "common/app_cppcommon.h"
#include "commandcenter.h"
#include "database.h"
#include "sample.h"


namespace mpisolver2 {
    DBXMLWriter::DBXMLWriter(CommandCenter* _cmd)
        : MEMBER_INIT(cmd, _cmd)
    {
    }

    DBXMLWriter::~DBXMLWriter() {
    }

    void DBXMLWriter::write() {
        LOG(INFO) << FUNCTION_NAME();
        this->write(RTQL8_DATA_PATH"xml/result.xml");
    }
    
#define DS(x) ( doc.allocate_string( (x) ) )
#define DS2(x) (doc.allocate_string(lexical_cast<std::string>((x)).c_str()))

    void DBXMLWriter::write(const char* const filename) {
        LOG(INFO) << FUNCTION_NAME();
        using namespace rapidxml;
        using boost::lexical_cast;
        db()->sort();
        
        try {
            xml_document<> doc;
            xml_node<> *root = doc.allocate_node(rapidxml::node_element, "result", "");
            doc.append_node(root);

            // const int MAX_NUM_TO_REPORT = 5;
            // int numToReport = std::min( db()->numSamples(), MAX_NUM_TO_REPORT );
            std::vector<Sample*> report;
            db()->selectSamplesToReport(report);
            LOG(INFO) << "# to report = " << report.size();
            for (int i = 0; i < report.size(); i++) {
                Sample* s = report[i];

                xml_node<>* node
                    = doc.allocate_node(node_element, DS("sample"), DS("") );
                root->append_node(node);

                {
                    std::stringstream gout;
                    gout << IO(s->params);
                    node->append_attribute(
                        doc.allocate_attribute(DS("gene"), DS(gout.str().c_str()))
                        );
                }

                {
                    double value = s->value;
                    node->append_attribute(
                        doc.allocate_attribute(DS("value"), DS2(value))
                        );
                }                

                {
                    node->append_attribute(
                        doc.allocate_attribute(DS("tag"), DS(s->tag.c_str()))
                        );
                }                

            }
            
            LOG(INFO) << FUNCTION_NAME() << " attempt to write to xml " << filename;
            std::ofstream fout(filename);
            fout << doc;
            fout.close();

        } catch (const std::exception& e) {
            LOG(INFO) << FUNCTION_NAME() << " NG : error = " << e.what() << endl;
        }


        LOG(INFO) << FUNCTION_NAME() << " OK";

        
    }



    Database* DBXMLWriter::db() {
        return cmd()->database();
    }


} // namespace mpisolver2
