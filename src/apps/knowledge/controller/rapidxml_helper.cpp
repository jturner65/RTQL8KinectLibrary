/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "boost/lexical_cast.hpp"
#include "boost/format.hpp"
#include <boost/algorithm/string.hpp>
#include <sstream>

#include "rapidxml.hpp"
#include "rapidxml_print.hpp"
#include "rapidxml_utils.hpp"

#include "rapidxml_helper.h"

#include "toolkit/Moreeigen.h"

namespace rapidxml {
    std::string tag(rapidxml::xml_node<>* node) {
        return boost::lexical_cast<std::string>(node->name());
    }

    bool has_attr(rapidxml::xml_node<>* node, const char* const name) {
        rapidxml::xml_attribute<>* attr = node->first_attribute(name);
        return (attr != NULL);
    }

    std::string attr(rapidxml::xml_node<>* node, const char* const name) {
        rapidxml::xml_attribute<>* attr = node->first_attribute(name);
        if (attr == NULL) return std::string("");
        return boost::lexical_cast<std::string>(attr->value());
    }
    
    bool attr_bool(rapidxml::xml_node<>* node, const char* const name) {
        std::string str = attr(node, name);
        boost::to_lower(str);
        if (str == "1" || str == "t" || str == "true") {
            return true;
        } else {
            return false;
        }
    }

    bool attr_bool(rapidxml::xml_node<>* node, const char* const name,
                   bool defaultValue) {
        if (has_attr(node, name)) {
            return attr_bool(node, name);
        } else {
            return defaultValue;
        }
        
    }

    double attr_double(rapidxml::xml_node<>* node, const char* const name) {
        return boost::lexical_cast<double>(node->first_attribute(name)->value());
    }

    double attr_double(rapidxml::xml_node<>* node, const char* const name,
                       double defaultValue) {
        if (has_attr(node, name)) {
            return attr_double(node, name);
        } else {
            return defaultValue;
        }
    }

    Eigen::Vector3d attr_v3(rapidxml::xml_node<>* node, const char* const name) {
        using rtql8::toolkit::moreeigen::IO;
        std::string str = attr(node, name);
        std::stringstream sout;
        sout << str;
        Eigen::Vector3d ret;
        sout >> IO(ret);
        return ret;
    }

    Eigen::VectorXd attr_vx(rapidxml::xml_node<>* node, const char* const name) {
        using rtql8::toolkit::moreeigen::IO;
        std::string str = attr(node, name);
        // std::cout << "str = " << str << std::endl;
        std::stringstream sout;
        sout << str;
        Eigen::VectorXd ret;
        sout >> IO(ret, -1);
        return ret;
    }
    
    Eigen::VectorXd attr_vx(rapidxml::xml_node<>* node,
                            const char* const name,
                            const Eigen::VectorXd& defaultValue) {
        if (has_attr(node, name)) {
            return attr_vx(node, name);
        } else {
            return defaultValue;
        }
    }

    char* alloc_str(rapidxml::xml_document<>* doc, const char* const str) {
        if (str) {
            return doc->allocate_string(str);
        }
        return NULL;
    }

    rapidxml::xml_node<>* alloc_node(rapidxml::xml_document<>* doc,
                                     const char* const name,
                                     const char* const value) {
        return doc->allocate_node(rapidxml::node_element,
                                  alloc_str(doc, name),
                                  alloc_str(doc, value));
    }

    rapidxml::xml_node<>* add_node(rapidxml::xml_document<>* doc,
                                   rapidxml::xml_node<>* node,
                                   const char* const name,
                                   const char* const value) {
        rapidxml::xml_node<>* new_node = alloc_node(doc, name, value);
        node->append_node( new_node );
        return new_node;
    }

    rapidxml::xml_attribute<>* alloc_attr(rapidxml::xml_document<>* doc,
                                          const char* const name,
                                          const char* const value) {
        return doc->allocate_attribute(alloc_str(doc, name),
                                       alloc_str(doc, value));
    }
    
    rapidxml::xml_attribute<>* alloc_attr(rapidxml::xml_document<>* doc,
                                          const char* const name,
                                          bool value) {
        if (value) {
            return alloc_attr(doc, name, "1");
        } else {
            return alloc_attr(doc, name, "0");
        }
    }

    rapidxml::xml_attribute<>* alloc_attr(rapidxml::xml_document<>* doc,
                                          const char* const name,
                                          double value) {
        // std::string value_str = boost::lexical_cast<std::string>(value);
        std::string value_str = (boost::format("%.6lf") % value).str();
        return alloc_attr(doc, name, value_str.c_str());
    }

    rapidxml::xml_attribute<>* alloc_attr(rapidxml::xml_document<>* doc,
                                          const char* const name,
                                          const Eigen::VectorXd& value) {
        using rtql8::toolkit::moreeigen::IO;
        std::stringstream sout;
        sout << IO(value);
        return alloc_attr(doc, name, sout.str().c_str());
    }

    void add_attr(rapidxml::xml_document<>* doc,
                  rapidxml::xml_node<>* node,
                  const char* const name,
                  const char* const value) {
        node->append_attribute( alloc_attr(doc, name, value) );
    }
    

    void add_attr(rapidxml::xml_document<>* doc,
                  rapidxml::xml_node<>* node,
                  const char* const name,
                  double value) {
        node->append_attribute( alloc_attr(doc, name, value) );
    }
    

    void add_attr(rapidxml::xml_document<>* doc,
                  rapidxml::xml_node<>* node,
                  const char* const name,
                  const Eigen::VectorXd& value) {
        node->append_attribute( alloc_attr(doc, name, value) );
    }

    void add_attr_int(rapidxml::xml_document<>* doc,
                      rapidxml::xml_node<>* node,
                      const char* const name,
                      int value) {
        std::string value_str = (boost::format("%d") % value).str();
        node->append_attribute( alloc_attr(doc, name, value_str.c_str()) );
    }

    void add_attr_bool(rapidxml::xml_document<>* doc,
                       rapidxml::xml_node<>* node,
                       const char* const name,
                       bool value) {
        node->append_attribute( alloc_attr(doc, name, value ? "1" : "0") );
    }

} // namespace rapidxml



