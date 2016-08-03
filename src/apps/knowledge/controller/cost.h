/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef controller_COST_H
#define controller_COST_H

#define COST_PENALTY 100.0
#include <vector>
#include "common/app_hppcommon.h"
#include "app_controller.h"

namespace rapidxml {
    template<class Ch> class xml_node;
    template<class Ch> class xml_document;
} // namespace rapidxml

namespace controller {
    class Cost;

////////////////////////////////////////////////////////////
// class Cost
    class Cost : public AppController {
    public:
        Cost();
        virtual ~Cost();

        enum CostTermID {
            COST_TERM_COM,
            COST_TERM_MAXCOM,
            COST_TERM_MINCOM,
            COST_TERM_COMDOT,
            COST_TERM_L,
            COST_TERM_LANDING,
            COST_TERM_UPRIGHT,
            COST_TERM_CONSTRAINT_POSITION,
            COST_TERM_BAR_DIST,
            COST_TERM_NONE
        };
    
    struct CostTerm {
            std::string name;
            CostTermID id;
            // double* v;
            // Eigen::Vector3d* v3;
            // Eigen::VectorXd* vx;
            Eigen::VectorXd* v;
            Eigen::VectorXd* v2;
            int index0, index1;  // Reserved for the constraints

            CostTerm();
            bool isRanged() const;

            static std::string TOSTR(const CostTermID& id);
            static CostTerm* createCost(const CostTermID _id,
                                        const Eigen::VectorXd& v0,
                                        const Eigen::VectorXd& v1);
            static CostTerm* createCost(const CostTermID _id,
                                        const Eigen::VectorXd& v0);
            static CostTerm* createConstraintPosition(const Eigen::VectorXd& dir,
                                                      int node0, int node1);

        };

// Evaluation functions
        void addTerm(CostTerm* t);
        bool findTerm(const CostTermID& id,
                      Eigen::Vector3d* v0, Eigen::Vector3d* v1);
        Eigen::VectorXd valueOfLandingTerm(const CostTerm* const t) const;
        Eigen::VectorXd valueOfUprightTerm(const CostTerm* const t) const;
        Eigen::VectorXd valueOfTerm(const CostTerm* const t) const;
        double valueOfConstraintPosition(const CostTerm* const t) const;
        double value() const;
        double value(double task) const;
        bool isRanged() const;
        void evaluateTaskAndError(double* task, double* error);
// Auxiliary functions
        virtual std::string toString() const;
// XML functions
        Cost* readXML(rapidxml::xml_node<char>* node);
        rapidxml::xml_node<char>* writeXML(rapidxml::xml_document<char>* pdoc,  bool hashMode = false);
    protected:
        std::vector<CostTerm*> terms;
// Optimization function
        double fibonacci_solve(double lower, double upper, int n_fibo, double tolerance);
    }; // class Cost

    
    
} // namespace controller

#endif // #ifndef controller_COST_H

