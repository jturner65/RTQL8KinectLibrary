/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef controller_SIMPACK_H
#define controller_SIMPACK_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include "common/app_hppcommon.h"

namespace rapidxml {
    template<class Ch> class xml_node;
    template<class Ch> class xml_document;
} // namespace rapidxml

namespace rtql8 {
    namespace toolkit {
        struct SimState;
        class Simulator;
    } // namespace toolkit
} // namespace rtql8

namespace controller {
    struct AppControllerState;
    class AppCompositeController;
} // namespace controller


namespace controller {
    
    class SimPack {
    public:
        SimPack();
        virtual ~SimPack();

// Main functions
        void reset();
        bool step();
        void simulate();

// Bar functions
        bool hasBar() { return barEnabled; }
        Eigen::Vector3d getBarPos() { return barPos; }
        void checkBar();
        void checkReleaseBar();

        Eigen::VectorXd targetPose();
        void setTargetPose(const Eigen::VectorXd& pose);

        void setController(AppCompositeController* c);
        double evaluateController(AppCompositeController* c, rtql8::toolkit::SimState* s);
        void evaluateController(AppCompositeController* c, rtql8::toolkit::SimState* s,
                                double* task, double* error);
        void evaluateController(AppCompositeController* c, rtql8::toolkit::SimState* s,
                                double task, double* error);


// Auxiliary functions
        virtual std::string toString() const;
// XML functions
        SimPack* readXML(rapidxml::xml_node<char>* node, bool verbose = true);
        rapidxml::xml_node<char>* writeXML(rapidxml::xml_document<char>* pdoc);

    protected:
        MEMBER_PTR(rtql8::toolkit::Simulator*, sim);
        MEMBER_PTR(rtql8::toolkit::SimState*, initSimState);
        MEMBER_PTR(AppCompositeController*, con);
        MEMBER_PTR(AppControllerState*, initConState);
        MEMBER_PTR(AppControllerState*, conState);

        std::vector<std::string> filenames;

        bool barEnabled;
        Eigen::Vector3d barPos;
        double barThreshold;
        
    }; // class SimPack
    
} // namespace controller

#endif // #ifndef controller_SIMPACK_H

