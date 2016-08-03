/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef controller_PREDICATE_H
#define controller_PREDICATE_H

#include "common/app_hppcommon.h"
#include "app_controller.h"
#include <deque>

namespace rapidxml {
    template<class Ch> class xml_node;
    template<class Ch> class xml_document;
} // namespace rapidxml

namespace controller {
    class Predicate;
    class PredicateTimeout;
    class PredicateNoContacts;
    class PredicateAnyContacts;
    class PredicateHighest;
    class PredicateLowest;
    

////////////////////////////////////////////////////////////
// class Predicate
    class Predicate : public AppController {
    public:
        Predicate();
        virtual ~Predicate();

        virtual bool value() = 0;
// XML functions
        static Predicate* readXML(rapidxml::xml_node<char>* node); // Like factory class
        virtual rapidxml::xml_node<char>* writeXML(rapidxml::xml_document<char>* pdoc, bool hashMode = false) = 0;
    protected:
        
    }; // class Predicate

////////////////////////////////////////////////////////////
// class PredicateTimeout
    class PredicateTimeout : public Predicate {
    public:
        PredicateTimeout();
        virtual ~PredicateTimeout();

        virtual bool value();
// Auxiliary functions
        virtual std::string toString() const;
// XML functions
        PredicateTimeout* readXML(rapidxml::xml_node<char>* node);
        virtual rapidxml::xml_node<char>* writeXML(rapidxml::xml_document<char>* pdoc, bool hashMode = false); 
    protected:
        MEMBER_VAR(double, to);
        
    }; // class PredicateTimeout
//
////////////////////////////////////////////////////////////
    
////////////////////////////////////////////////////////////
// class PredicateNoContacts
    class PredicateNoContacts : public Predicate {
    public:
        PredicateNoContacts();
        virtual ~PredicateNoContacts();

        virtual bool value();
// Auxiliary functions
        virtual std::string toString() const;
// XML functions
        PredicateNoContacts* readXML(rapidxml::xml_node<char>* node);
        virtual rapidxml::xml_node<char>* writeXML(rapidxml::xml_document<char>* pdoc, bool hashMode = false); 
    protected:
        
    }; // class PredicateNoContacts
//
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// class PredicateAnyContacts
    class PredicateAnyContacts : public Predicate {
    public:
        PredicateAnyContacts();
        virtual ~PredicateAnyContacts();

        virtual bool value();
// Auxiliary functions
        virtual std::string toString() const;
// XML functions
        PredicateAnyContacts* readXML(rapidxml::xml_node<char>* node);
        virtual rapidxml::xml_node<char>* writeXML(rapidxml::xml_document<char>* pdoc, bool hashMode = false); 
    protected:
        
    }; // class PredicateAnyContacts
//
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// class PredicateHighest
    class PredicateHighest : public Predicate {
    public:
        PredicateHighest();
        virtual ~PredicateHighest();

        virtual bool value();
// Auxiliary functions
        virtual std::string toString() const;
// XML functions
        PredicateHighest* readXML(rapidxml::xml_node<char>* node);
        virtual rapidxml::xml_node<char>* writeXML(rapidxml::xml_document<char>* pdoc, bool hashMode = false); 
    protected:
        std::deque<double> velocities;
    }; // class PredicateHighest
//
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// class PredicateLowest
    class PredicateLowest : public Predicate {
    public:
        PredicateLowest();
        virtual ~PredicateLowest();

        virtual bool value();
// Auxiliary functions
        virtual std::string toString() const;
// XML functions
        PredicateLowest* readXML(rapidxml::xml_node<char>* node);
        virtual rapidxml::xml_node<char>* writeXML(rapidxml::xml_document<char>* pdoc, bool hashMode = false); 
    protected:
        std::deque<double> velocities;
    }; // class PredicateLowest
//
////////////////////////////////////////////////////////////
    
    
} // namespace controller

#endif // #ifndef controller_PREDICATE_H


