/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "knowledge.h"

#include <sstream>
#include <fstream>
#include <boost/functional/hash.hpp>
#include "common/app_cppcommon.h"
#include "rapidxml_helper.h"
#include "rig.h"
#include "app_composite_controller.h"
#include "utils/Timer.h"

#include "simpack.h"

namespace controller {

////////////////////////////////////////////////////////////
// class Knowledge implementation
    Knowledge::Knowledge()
        : MEMBER_INIT_NULL(xmlfile)
        , MEMBER_INIT_NULL(xmldoc)
        , MEMBER_INIT_NULL(simpack)
        , xmlstring(NULL)
    {
    }
    
    Knowledge::~Knowledge() {
    }

// Auxiliary functions
    std::string Knowledge::toString() const {
        std::string str;
        str += "==== Knowledge ====\n";
        BOOST_FOREACH(const ControllerEntry& e, entries){
            str += e.name + " : ";
            if (e.con != NULL) {
                str += e.con->toString();
            }
            str += "\n";
        }
        str += "===================\n";
        return str;
    }

    std::string Knowledge::hashString() {
        std::string input = saveXMLString(true);
        // cout << "input = " << input << endl;

        boost::hash<std::string> string_hash;
        size_t h = string_hash(input.c_str());
        std::string ret;
        int loopcount = 0;
        while(h > 0) {
            int code = h % (26 + 26 + 10);
            char c;
            h /= (26 + 26 + 10);
            if (code < 26) {
                c = 'A' + code;
            } else if (code < 26 + 26) {
                c = 'a' + (code - 26);
            } else {
                c = '0' + (code - 26 - 26);
            }
            ret += c;
            loopcount++;
            if (loopcount > 100) {
                LOG_FATAL << "invalid hashString";
                exit(1);
            }
        }
        // cout << "ret = " << ret << endl;
        return ret;
    }

// XML related members and functions
    bool Knowledge::loadXML(const char* const filename) {
        namespace rx = rapidxml;
        try {
            MEMBER_RELEASE_PTR(xmldoc);
            MEMBER_RELEASE_PTR(xmlfile);
            
            set_xmlfile( new rx::file<char>(filename) );
            set_xmldoc( new rx::xml_document<char>() );
            xmldoc()->parse<0>(xmlfile()->data());

            xmlfilename = filename;

            return loadXML();
        } catch (const std::exception& e) {
            LOG_ERROR << FUNCTION_NAME() << " : error = " << e.what() << endl;
        }
        return false;
    }

    bool Knowledge::loadXMLString(const char* const _xml) {
        if (xmlstring != NULL) {
            delete[] xmlstring;
        }
        int len = strlen(_xml);
        xmlstring = new char[len + 5];
        sprintf(xmlstring, "%s", _xml);
        
        namespace rx = rapidxml;
        try {
            MEMBER_RELEASE_PTR(xmldoc);
            MEMBER_RELEASE_PTR(xmlfile);
            
            set_xmlfile( NULL );
            set_xmldoc( new rx::xml_document<char>() );
            xmldoc()->parse<0>(xmlstring);

            xmlfilename = "<xmlstring>";

            return loadXML();
        } catch (const std::exception& e) {
            LOG_ERROR << FUNCTION_NAME() << " : error = " << e.what() << endl;
        }
        return false;
    }
    
    bool Knowledge::loadXML(bool verbose) {
        namespace rx = rapidxml;
        try {
            // MEMBER_RELEASE_PTR(xmldoc);
            // MEMBER_RELEASE_PTR(xmlfile);
            
            // set_xmlfile( new rx::file<char>(filename) );
            // set_xmldoc( new rx::xml_document<char>() );
            // xmldoc()->parse<0>(xmlfile()->data());

            rx::xml_node<char>* root = xmldoc()->first_node("knowledge");
            if (root == NULL) {
                LOG_WARNING << FUNCTION_NAME() << " : cannot find knowledge node";
                return false;
            }

            rx::xml_node<char>* node_simulation = root->first_node("simulation");
            if (node_simulation) {
                set_simpack(
                    (new SimPack())->readXML(node_simulation, verbose)
                    );
            }

            rx::xml_node<char>* node_primitives = root->first_node("primitives");
            if (node_primitives) {
                for (rx::xml_node<char> *nd = node_primitives->first_node("controller");
                     nd; nd = nd->next_sibling("controller")) {
                    // cout << "primitive controller = " << *nd << endl;
                    AppController* rig = Rig::readXML(nd);
                    this->addControllerEntry(nd, rig, verbose);
                    // LOG_INFO << "detect composite: " << rx::attr(nd, "name");
                }
                
            }
            
            for (rx::xml_node<char> *nd = root->first_node("composite");
                 nd; nd = nd->next_sibling("composite")) {
                // cout << "composite controller = " << *nd << endl;
                AppController* con = (new AppCompositeController())->readXML(this, nd);
                this->addControllerEntry(nd, con, verbose);
                // LOG_INFO << "detect composite: " << rx::attr(nd, "name");
            }

            // select(entries[entries.size() - 1]);

            VLOG_INFO << "hashstring = " << hashString();
            
            VLOG_INFO << FUNCTION_NAME() << " : OK";


            return true;
            
        } catch (const std::exception& e) {
            LOG_ERROR << FUNCTION_NAME() << " : error = " << e.what() << endl;
        }
        return false;
    }
        
    rapidxml::xml_node<char>* Knowledge::writeXML(rapidxml::xml_document<char>* pdoc,
                                                  bool hashMode) {
        namespace rx = rapidxml;
        try {
            rx::xml_node<>* nd = rx::alloc_node(pdoc, "knowledge");
            nd->append_node( simpack()->writeXML(pdoc) );
            
            rx::xml_node<>* nd_prim = add_node(pdoc, nd, "primitives");
            BOOST_FOREACH(const ControllerEntry& e, entries) {
                if (e.isPrimitive()) {
                    rx::xml_node<>* nd_con = e.con->writeXML(pdoc, hashMode);
                    nd_prim->append_node(nd_con);
                } else if (e.isComposite()) {
                    rx::xml_node<>* nd_con = e.con->writeXML(pdoc, hashMode);
                    if (e.isSelected) {
                        rx::add_attr_bool(pdoc, nd_con, "select", true);
                    }
                    nd->append_node(nd_con);
                }
            }
            return nd;
        } catch (const std::exception& e) {
            LOG_ERROR << FUNCTION_NAME() << " : error = " << e.what() << endl;
        }
        return NULL;
        
    }


    bool Knowledge::saveXML(const char* const filename) {
        LOG_INFO << FUNCTION_NAME();
        namespace rx = rapidxml;
        try {
            rx::xml_document<> doc;
            doc.append_node( this->writeXML(&doc) );

            cout << doc << endl;
            
            std::ofstream fout(filename);
            fout << doc << endl;
            fout.close();
            
        } catch (const std::exception& e) {
            LOG_ERROR << FUNCTION_NAME() << " : error = " << e.what() << endl;
        }
        return true;
    }

    std::string Knowledge::saveXMLString(bool hashMode) {
        std::stringstream sout;
        namespace rx = rapidxml;
        try {
            rx::xml_document<> doc;
            doc.append_node( this->writeXML(&doc, hashMode) );
            sout << doc;
            return sout.str();
        } catch (const std::exception& e) {
            LOG_ERROR << FUNCTION_NAME() << " : error = " << e.what() << endl;
            exit(1);
        }
        return "error";
    }

    Knowledge* Knowledge::duplicate() {
        LOG_INFO << FUNCTION_NAME() << " : " << xmlfilename;
        namespace rx = rapidxml;
        Knowledge* ret = new Knowledge();
        try {
            // MEMBER_RELEASE_PTR(xmldoc);
            // MEMBER_RELEASE_PTR(xmlfile);
            
            ret->set_xmlfile( NULL );
            rx::xml_document<char>* pdoc = new rx::xml_document<char>();
            ret->set_xmldoc( pdoc );
            pdoc->append_node( this->writeXML(pdoc) );
            ret->xmlfilename = "<xmlstring>";
            ret->loadXML(false);

            // ret->dumpControllerEntries();


        } catch (const std::exception& e) {
            LOG_ERROR << FUNCTION_NAME() << " : error = " << e.what() << endl;
        }

        return ret;
    }
        
// Controller Entries and related functions
    bool Knowledge::addControllerEntry(rapidxml::xml_node<char>* node,
                                       AppController* con, bool verbose)  {
        LOG_INFO << FUNCTION_NAME();
        namespace rx = rapidxml;
        ControllerEntry e;
        e.xmlnode = node;
        e.name = rx::attr(node, "name");
        // e.con = (new AppCompositeController())->readXML(this, node);
        e.con = con;
        e.isSelected = false;

        // LOG_INFO << "name = " << e.name;
        // LOG_INFO << "entries.size() = " << entries.size();

        bool is_selected = rx::attr_bool(node, "select", false);
        if (is_selected) {
            select(e);
        }

        entries.push_back(e);

        // VLOG_INFO << " entry added: name = " << e.name;
                 // << " " << e.con->toString();
        return true;
    }

    bool Knowledge::addControllerEntry(AppController* con, bool verbose)  {
        LOG_INFO << FUNCTION_NAME();
        namespace rx = rapidxml;
        ControllerEntry e;
        rx::xml_node<char>* nd = xmldoc()->first_node("knowledge");
        rx::xml_node<>* nd_con = con->writeXML(xmldoc());
        nd->append_node(nd_con);

        e.xmlnode = nd_con;
        e.name = con->name;
        // e.con = (new AppCompositeController())->readXML(this, node);
        e.con = con;
        e.isSelected = false;


        entries.push_back(e);

        // VLOG_INFO << " entry added: name = " << e.name;
                 // << " " << e.con->toString();
        return true;
    }


    void Knowledge::select(ControllerEntry& e) {
        if (simpack() == NULL) {
            return;
        }
        if (e.isComposite() == false) {
            return;
        }

        BOOST_FOREACH(ControllerEntry& ee, entries) {
            ee.isSelected = false;
        }
        AppCompositeController* con = dynamic_cast<AppCompositeController*>(e.con);
        simpack()->setController(con);
        // simpack()->reset();
        e.isSelected = true;
        LOG_INFO << "select controller [" << e.name << "] OK";
    }

    void Knowledge::select(const char* const name) {
        std::string str_name(name);
        BOOST_FOREACH(ControllerEntry& ee, entries) {
            // LOG_INFO << "entry.name = " << ee.name << " vs. " << str_name;
            if (ee.name == str_name) {
                select(ee);
                return;
            }
        }
        // LOG_FATAL << "failed to find a controller";
        // exit(0);
    }

    AppController* Knowledge::getController(const char* const name) {
        std::string str_name(name);
        BOOST_FOREACH(const ControllerEntry& e, entries) {
            if (e.name == str_name) {
                return e.con;
            }
        }
        return NULL;
    }

    AppController* Knowledge::getControllerDupWithoutPrev(const char* const name) {
        return getControllerDup(name, true);
    }

    AppController* Knowledge::getControllerDup(const char* const name,
                                               bool ignorePrev) {
        std::string str_name(name);
        BOOST_FOREACH(const ControllerEntry& e, entries) {
            if (e.name == str_name) {

#ifdef LOGGING_BOOST
                namespace logging = boost::log;
                logging::core::get()->set_filter
                    (
                        logging::trivial::severity >= logging::trivial::fatal
                        );            
#endif
                
                AppController* dup = NULL;
                if (e.isPrimitive()) {
                    dup = Rig::readXML(e.xmlnode);
                } else {
                    dup = (new AppCompositeController())
                        ->readXML(this, e.xmlnode, ignorePrev);
                }

#ifdef LOGGING_BOOST
                logging::core::get()->set_filter
                    (
                        logging::trivial::severity >= logging::trivial::info
                        );            
#endif

                return dup;
            }
        }
        return NULL;
    }

    AppController* Knowledge::createController(const char* const name) {
        cout << "createController:: xmldoc() = " << xmldoc() << endl;
        if (xmldoc() == NULL) {
            return NULL;
        }
        cout << "working on xml.." << endl;
        namespace rx = rapidxml;
        rx::xml_node<char>* root = xmldoc()->first_node("knowledge");
        rx::xml_node<char>* nd = rx::add_node(xmldoc(), root, "composite");
        rx::xml_node<char>* nd_eval = rx::add_node(xmldoc(), nd, "evaluate");
        rx::add_attr(xmldoc(), nd, "name", name);
        cout << "##################################################" << endl;
        cout << *xmldoc() << endl;
        cout << "##################################################" << endl;

        AppController* con = (new AppCompositeController())->readXML(this, nd);
        cout << "con->toString = " << con->toString() << endl;
        this->addControllerEntry(nd, con, true);
        // cout << "##################################################" << endl;
        // cout << *xmldoc() << endl;
        // cout << "##################################################" << endl;
        return con;
    }

    std::vector<std::string> Knowledge::getPrimitiveControllerNames() {
        std::vector<std::string> names;
        BOOST_FOREACH(const ControllerEntry& e, entries) {
            if (e.isComposite()) {
                continue;
            }
            names.push_back(e.name);
        }
        return names;
    }

    std::vector<std::string> Knowledge::getCompositeControllerNames() {
        std::vector<std::string> names;
        BOOST_FOREACH(const ControllerEntry& e, entries) {
            if (e.isPrimitive()) {
                continue;
            }
            names.push_back(e.name);
        }
        return names;
    }


    void Knowledge::turnOffLog() {
#ifdef LOGGING_BOOST
        namespace logging = boost::log;
        logging::core::get()->set_filter
            (
                logging::trivial::severity >= logging::trivial::fatal
                );            
#endif

    }

    void Knowledge::dumpControllerEntries() {
        for (int i = 0; i < entries.size(); i++) {
            ControllerEntry& e = entries[i];
            LOG_INFO << i << " : " << e.name << " selected = " << e.isSelected;
            LOG_INFO << e.con->toString();
        }

    }

// Optimization Hints
    std::string Knowledge::fetchHint() {
        if (hints.size() == 0) {
            return std::string("");
        }
        
        std::string front = hints.front();
        hints.pop_front();
        return front;
    }
    
    void Knowledge::pushHint(const char* const hint) {
        hints.push_back( std::string(hint) );
    }

    AppCompositeController* Knowledge::con() {
        return simpack()->con();
    }

// class Knowledge ends
////////////////////////////////////////////////////////////
    
////////////////////////////////////////////////////////////
// class Knowledge::ControllerEntry begins
    bool Knowledge::ControllerEntry::isPrimitive() const {
        return (dynamic_cast<Rig*>(con) != NULL);
    }

    bool Knowledge::ControllerEntry::isComposite() const {
        return (dynamic_cast<AppCompositeController*>(con) != NULL);
    }

// class Knowledge::ControllerEntry ends
////////////////////////////////////////////////////////////


    
} // namespace controller



