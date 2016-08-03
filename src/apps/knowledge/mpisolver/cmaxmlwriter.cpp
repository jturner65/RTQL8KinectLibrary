#include "cmaxmlwriter.h"

#include <fstream>
#include "boost/lexical_cast.hpp"
#include "common/app_cppcommon.h"
#include "MPICMASearch.h"

namespace mpisolver {
    CMAXMLWriter::CMAXMLWriter(MPICMASearch* _cma)
        : MEMBER_INIT(cma, _cma)
        , root( NULL )
    {
        allocate();
    }
    
    CMAXMLWriter::~CMAXMLWriter() {
        deallocate();
    }

    void CMAXMLWriter::allocate() {
        root = doc.allocate_node(rapidxml::node_element, "optimization", "");
        doc.append_node(root);
    }

    void CMAXMLWriter::deallocate() {
    }

    bool CMAXMLWriter::addIteration(int iter) {
        using namespace rapidxml;
        using boost::lexical_cast;
        using boost::format;

#define DS(x) ( doc.allocate_string( (x) ) )
#define DS2(x) (doc.allocate_string(lexical_cast<std::string>((x)).c_str()))
        
        if (!root) {
            return false;
        }

        xml_node<>* node
            = doc.allocate_node(node_element, DS("iteration"), DS("") );
        root->append_node(node);

        {
            // char* attr_name =  doc.allocate_string("iter");
            // char* attr_iter =  doc.allocate_string(lexical_cast<std::string>(iter).c_str());
            // cout << "attr_iter = " << attr_iter << endl;
            // xml_attribute<>* attr = doc.allocate_attribute(DS("iter"), attr_iter);
            xml_attribute<>* attr = doc.allocate_attribute(DS("iter"), DS2(iter));
            node->append_attribute(attr);
        }

        for (int i = 0; i < cma()->numParents(); i++) {
            Eigen::VectorXd gene = cma()->parent(i);
            double value = cma()->parentValue(i);
            std::stringstream gout;
            gout << IO(gene);

            
            xml_node<>* node_parent
                = doc.allocate_node(node_element, DS("parent"), DS("") );
            node->append_node(node_parent);

            node_parent->append_attribute(
                doc.allocate_attribute(DS("index"), DS2(i))
                );

            node_parent->append_attribute(
                doc.allocate_attribute(DS("gene"), DS(gout.str().c_str()))
                );

            node_parent->append_attribute(
                doc.allocate_attribute(DS("value"), DS2(value))
                );
            
        }

        return true;
    }

    bool CMAXMLWriter::addResult() {
    }

    bool CMAXMLWriter::write(const char* const filename) {
        cout << "== current xml document ==" << endl << doc << endl << endl;
        std::ofstream fout(filename);
        fout << doc;
        fout.close();
        LOG(INFO) << FUNCTION_NAME() << " OK";
        return true;
    }

} // namespace mpisolver
