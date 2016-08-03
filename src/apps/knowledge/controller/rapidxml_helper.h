/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef controller_RAPIDXML_HELPER_H
#define controller_RAPIDXML_HELPER_H

#include <string>
#include <Eigen/Dense>
#include "rapidxml.hpp"
#include "rapidxml_print.hpp"
#include "rapidxml_utils.hpp"

namespace rapidxml {
    std::string tag(rapidxml::xml_node<>* node);
    bool has_attr(rapidxml::xml_node<>* node, const char* const name);
    std::string attr(rapidxml::xml_node<>* node, const char* const name);
    bool attr_bool(rapidxml::xml_node<>* node, const char* const name);
    bool attr_bool(rapidxml::xml_node<>* node, const char* const name,
                   bool defaultValue);
    double attr_double(rapidxml::xml_node<>* node, const char* const name);
    double attr_double(rapidxml::xml_node<>* node, const char* const name,
                       double defaultValue);

    Eigen::Vector3d attr_v3(rapidxml::xml_node<>* node, const char* const name);
    Eigen::VectorXd attr_vx(rapidxml::xml_node<>* node, const char* const name);
    Eigen::VectorXd attr_vx(rapidxml::xml_node<>* node,
                            const char* const name,
                            const Eigen::VectorXd& defaultValue);

    char* alloc_str(rapidxml::xml_document<>* doc, const char* const str);
    rapidxml::xml_node<>* alloc_node(rapidxml::xml_document<>* doc,
                                     const char* const name,
                                     const char* const value = NULL);

    rapidxml::xml_node<>* add_node(rapidxml::xml_document<>* doc,
                                   rapidxml::xml_node<>* node,
                                   const char* const name,
                                   const char* const value = NULL);
        
    rapidxml::xml_attribute<>* alloc_attr(rapidxml::xml_document<>* doc,
                                          const char* const name,
                                          const char* const value);
    // rapidxml::xml_attribute<>* alloc_attr(rapidxml::xml_document<>* doc,
    //                                       const char* const name,
    //                                       bool value);
    rapidxml::xml_attribute<>* alloc_attr(rapidxml::xml_document<>* doc,
                                          const char* const name,
                                          double value);

    rapidxml::xml_attribute<>* alloc_attr(rapidxml::xml_document<>* doc,
                                          const char* const name,
                                          const Eigen::VectorXd& value);

    void add_attr(rapidxml::xml_document<>* doc,
                  rapidxml::xml_node<>* node,
                  const char* const name,
                  const char* const value);

    void add_attr(rapidxml::xml_document<>* doc,
                  rapidxml::xml_node<>* node,
                  const char* const name,
                  double value);

    void add_attr(rapidxml::xml_document<>* doc,
                  rapidxml::xml_node<>* node,
                  const char* const name,
                  const Eigen::VectorXd& value);

    void add_attr_int(rapidxml::xml_document<>* doc,
                       rapidxml::xml_node<>* node,
                       const char* const name,
                       int value);

    void add_attr_bool(rapidxml::xml_document<>* doc,
                       rapidxml::xml_node<>* node,
                       const char* const name,
                       bool value);
                                   
    
} // namespace rapidxml

#endif // #ifndef controller_RAPIDXML_HELPER_H

