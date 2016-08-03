/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef controller_KNOWLEDGE_H
#define controller_KNOWLEDGE_H

#include <vector>
#include <deque>
#include <string>
#include "common/app_hppcommon.h"

namespace rapidxml {
    template<class Ch> class file;
    template<class Ch> class xml_document;
    template<class Ch> class xml_node;
} // namespace rapidxml

namespace controller {
    class SimPack;
    class AppController;
    class AppCompositeController;
} // namespace controller

namespace controller {
    
    class Knowledge {
    public:
        Knowledge();
        virtual ~Knowledge();

// Auxiliary functions
        std::string toString() const;
        std::string hashString();
// XML related members and functions
        std::string xmlfilename;
        MEMBER_PTR(rapidxml::file<char>*, xmlfile);
        MEMBER_PTR(rapidxml::xml_document<char>*, xmldoc);
        char* xmlstring;
        bool hasXML() { return (xmldoc() != NULL); }
        bool loadXML(const char* const filename);
        bool loadXMLString(const char* const _xml);
        bool loadXML(bool verbose = true);
        rapidxml::xml_node<char>* writeXML(rapidxml::xml_document<char>* pdoc, bool hashMode = false);
        bool saveXML(const char* const filename);
        std::string saveXMLString(bool hashMode = false);
        Knowledge* duplicate();

// Simulator
        MEMBER_PTR(SimPack*, simpack);
        AppCompositeController* con();
// Controller Entries and related functions
        struct ControllerEntry {
            rapidxml::xml_node<char>* xmlnode;
            std::string name;
            AppController* con;
            bool isSelected;

            bool isPrimitive() const;
            bool isComposite() const;
            
        };
        std::vector<ControllerEntry> entries;

        bool addControllerEntry(rapidxml::xml_node<char>* node,
                                AppController* con, bool verbose = true);
        bool addControllerEntry(AppController* con, bool verbose = true);
        void select(ControllerEntry& e);
        void select(const char* const name);
        AppController* getController(const char* const name);
        AppController* getControllerDupWithoutPrev(const char* const name);
        AppController* getControllerDup(const char* const name,
                                        bool ignorePrev = false);
        AppController* createController(const char* const name);

        std::vector<std::string> getPrimitiveControllerNames();
        std::vector<std::string> getCompositeControllerNames();
        
        void turnOffLog();
        void dumpControllerEntries();

// Optimization Hints
        bool hasHint() { return hints.size() > 0; }
        std::string fetchHint();
        void pushHint(const char* const hint);
        std::deque<std::string> hints;
        
    }; // class Knowledge
    
} // namespace controller

#endif // #ifndef controller_KNOWLEDGE_H

