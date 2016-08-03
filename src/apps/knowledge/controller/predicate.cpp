/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "predicate.h"
#include "common/app_cppcommon.h"
#include "rapidxml_helper.h"

#define MININUM_TRANSITION_TIME 0.1

namespace controller {
    
////////////////////////////////////////////////////////////
// class Predicate implementation
    Predicate::Predicate() {
    }
    
    Predicate::~Predicate() {
    }

    Predicate* Predicate::readXML(rapidxml::xml_node<char>* node) {
        if (tag(node) == "timeout") {
            return (new PredicateTimeout())->readXML(node);
        } else  if (tag(node) == "nocontacts") {
            return (new PredicateNoContacts())->readXML(node);
        } else  if (tag(node) == "anycontacts") {
            return (new PredicateAnyContacts())->readXML(node);
        } else  if (tag(node) == "highest") {
            return (new PredicateHighest())->readXML(node);
        } else  if (tag(node) == "lowest") {
            return (new PredicateLowest())->readXML(node);
        }
        LOG_ERROR << FUNCTION_NAME() << " unknown predicate: " << tag(node);
        return NULL;
    }

// class Predicate ends
////////////////////////////////////////////////////////////
    
////////////////////////////////////////////////////////////
// class PredicateTimeout implementation
    PredicateTimeout::PredicateTimeout()
        : Predicate()
        , MEMBER_INIT_NULL(to)
    {
    }
    
    PredicateTimeout::~PredicateTimeout() {
    }

    bool PredicateTimeout::value() {
        const double EPS = 0.00001;
        if (local_t() > to() - EPS) {
            return true;
        } else {
            return false;
        }
    }


// Auxiliary functions
    std::string PredicateTimeout::toString() const {
        return (boost::format("[PredicateTimeout %.4f]") % to()).str();
    }
// XML functions
    PredicateTimeout* PredicateTimeout::readXML(rapidxml::xml_node<char>* node) {
        set_to( attr_double(node, "to") );
        return this;
    }

    rapidxml::xml_node<char>* PredicateTimeout::writeXML(rapidxml::xml_document<char>* pdoc, bool hashMode) {
        namespace rx = rapidxml;
        rx::xml_node<char>* node = rx::alloc_node(pdoc, "timeout");
        rx::add_attr(pdoc, node, "to",  this->to());
        return node;
    }


// class PredicateTimeout ends
////////////////////////////////////////////////////////////

    
////////////////////////////////////////////////////////////
// class PredicateNoContacts implementation
    PredicateNoContacts::PredicateNoContacts()
        : Predicate()
    {
    }
    
    PredicateNoContacts::~PredicateNoContacts() {
    }

    bool PredicateNoContacts::value() {
        return (nContacts() == 0 && local_t() > MININUM_TRANSITION_TIME);
    }


// Auxiliary functions
    std::string PredicateNoContacts::toString() const {
        return (boost::format("[PredicateNoContacts ]") ).str();
    }
// XML functions
    PredicateNoContacts* PredicateNoContacts::readXML(rapidxml::xml_node<char>* node) {
        return this;
    }

    rapidxml::xml_node<char>* PredicateNoContacts::writeXML(rapidxml::xml_document<char>* pdoc, bool hashMode) {
        namespace rx = rapidxml;
        rx::xml_node<char>* node = rx::alloc_node(pdoc, "nocontacts");
        return node;
    }


// class PredicateNoContacts ends
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// class PredicateAnyContacts implementation
    PredicateAnyContacts::PredicateAnyContacts()
        : Predicate()
    {
    }
    
    PredicateAnyContacts::~PredicateAnyContacts() {
    }

    bool PredicateAnyContacts::value() {
        bool ret =  (nContacts() > 0 && local_t() > MININUM_TRANSITION_TIME);
        // if (ret) {
        //     LOG_INFO << "any contact!! " << nContacts() << " "
        //              << "t = " << t() << " "
        //              << "local_t = " << local_t() << " ";
        // }
        if (local_t() > 2.0) {
            return true;
        }
        return ret;
    }


// Auxiliary functions
    std::string PredicateAnyContacts::toString() const {
        return (boost::format("[PredicateAnyContacts ]") ).str();
    }
// XML functions
    PredicateAnyContacts* PredicateAnyContacts::readXML(rapidxml::xml_node<char>* node) {
        return this;
    }

    rapidxml::xml_node<char>* PredicateAnyContacts::writeXML(rapidxml::xml_document<char>* pdoc, bool hashMode) {
        namespace rx = rapidxml;
        rx::xml_node<char>* node = rx::alloc_node(pdoc, "anycontacts");
        return node;
    }


// class PredicateAnyContacts ends
////////////////////////////////////////////////////////////



////////////////////////////////////////////////////////////
// class PredicateHighest implementation
    PredicateHighest::PredicateHighest()
        : Predicate()
    {
    }
    
    PredicateHighest::~PredicateHighest() {
    }

    bool PredicateHighest::value() {
        double vy = COMdot().y();
        velocities.push_back(vy);
        if (velocities.size() > 10) {
            velocities.pop_front();
        }

        if (local_t() < 0.2) {
            return false;
        }

        for (int i = 0; i < velocities.size(); i++) {
            double v = velocities[i];
            if (i < 5) {
                if (v < 0) return false;
            } else {
                if (v > 0) return false;
            }
        }

        return true;
    }


// Auxiliary functions
    std::string PredicateHighest::toString() const {
        return (boost::format("[PredicateHighest ]") ).str();
    }
// XML functions
    PredicateHighest* PredicateHighest::readXML(rapidxml::xml_node<char>* node) {
        return this;
    }

    rapidxml::xml_node<char>* PredicateHighest::writeXML(rapidxml::xml_document<char>* pdoc, bool hashMode) {
        namespace rx = rapidxml;
        rx::xml_node<char>* node = rx::alloc_node(pdoc, "highest");
        return node;
    }


// class PredicateHighest ends
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// class PredicateLowest implementation
    PredicateLowest::PredicateLowest()
        : Predicate()
    {
    }
    
    PredicateLowest::~PredicateLowest() {
    }

    bool PredicateLowest::value() {
        double vy = COMdot().y();
        velocities.push_back(vy);
        if (velocities.size() > 300) {
            velocities.pop_front();
        }

        if (local_t() < 0.2) {
            return false;
        }

        for (int i = 0; i < velocities.size(); i++) {
            double v = velocities[i];
            if (i < 5) {
                if (v > 0) return false;
            } else {
                if (v < 0) return false;
            }
        }

        return true;
    }


// Auxiliary functions
    std::string PredicateLowest::toString() const {
        return (boost::format("[PredicateLowest ]") ).str();
    }
// XML functions
    PredicateLowest* PredicateLowest::readXML(rapidxml::xml_node<char>* node) {
        return this;
    }

    rapidxml::xml_node<char>* PredicateLowest::writeXML(rapidxml::xml_document<char>* pdoc, bool hashMode) {
        namespace rx = rapidxml;
        rx::xml_node<char>* node = rx::alloc_node(pdoc, "lowest");
        return node;
    }


// class PredicateLowest ends
////////////////////////////////////////////////////////////

    
} // namespace controller



