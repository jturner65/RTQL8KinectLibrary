/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef controller_RIG_H
#define controller_RIG_H

#include "toolkit/Toolkit.h"
#include "common/app_hppcommon.h"
#include "app_controller.h"

namespace rapidxml {
    template<class Ch> class xml_document;
    template<class Ch> class xml_node;
} // namespace rapidxml

/*
  <rig>
    <legpose axis="d" params="[0.0]" />
    <legpose axis="a" params="[0.0]" />
  </rig>
*/
namespace controller {
    class Rig;
    class RigJoints;
    class RigFeedbackJoints;
    class RigLegPose;
    class RigArmPose;
    class RigVF;
    class RigStiffJoints;
    
    class Rig : public AppController {
    public:
        Rig();
        virtual ~Rig();

// Inherited functions from AppController
        virtual int dim() { return 1; }
        virtual void setParams(const Eigen::VectorXd& p) { params = p; }
// Auxiliary functions
        virtual std::string toString() const;
// XML functions
        static Rig* readXML(rapidxml::xml_node<char>* node); // Like factory class
        virtual rapidxml::xml_node<char>* writeXML(rapidxml::xml_document<char>* pdoc, bool hashMode = false);

        void setDesiredParam(double _p);
        double cost();

    protected:
        // std::string name;
        Eigen::VectorXd params;
        MEMBER_VAR(double, upper);
        MEMBER_VAR(double, lower);

        MEMBER_VAR(bool, hasDesiredParam);
        MEMBER_VAR(double, desiredParam);
        
        
    }; // class Rig


////////////////////////////////////////////////////////////
// class RigJoints
    class RigJoints : public Rig {
    public:
        RigJoints();
        virtual ~RigJoints();

        virtual void changePose();
// XML functions
        RigJoints* readXML(rapidxml::xml_node<char>* node);
        // virtual rapidxml::xml_node<char>* writeXML(rapidxml::xml_document<char>* pdoc);
    protected:
        MEMBER_VAR(std::string, axis);
        
    }; // class RigJoints
//
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// class RigFeedbackJoints
    class RigFeedbackJoints : public Rig {
    public:
        RigFeedbackJoints();
        virtual ~RigFeedbackJoints();

        virtual void exertForce(); // This should be called per every single frame
// XML functions
        RigFeedbackJoints* readXML(rapidxml::xml_node<char>* node);
        // virtual rapidxml::xml_node<char>* writeXML(rapidxml::xml_document<char>* pdoc);
    protected:
        MEMBER_VAR(std::string, axis);
        
    }; // class RigFeedbackJoints
//
////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////
// class RigLegPose
    class RigLegPose : public Rig {
    public:
        RigLegPose();
        virtual ~RigLegPose();

        virtual void changePose();
// XML functions
        RigLegPose* readXML(rapidxml::xml_node<char>* node);
        // virtual rapidxml::xml_node<char>* writeXML(rapidxml::xml_document<char>* pdoc);
    protected:
        MEMBER_VAR(std::string, axis);
        
    }; // class RigLegPose
//
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// class RigArmPose
    class RigArmPose : public Rig {
    public:
        RigArmPose();
        virtual ~RigArmPose();

        virtual void changePose();
// XML functions
        RigArmPose* readXML(rapidxml::xml_node<char>* node);
        // virtual rapidxml::xml_node<char>* writeXML(rapidxml::xml_document<char>* pdoc);
    protected:
        MEMBER_VAR(std::string, axis);
        
    }; // class RigArmPose
//
////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////
// class RigVF
    class RigVF : public Rig {
    public:
        RigVF();
        virtual ~RigVF();

        virtual void exertForce();
// XML functions
        RigVF* readXML(rapidxml::xml_node<char>* node);
        // virtual rapidxml::xml_node<char>* writeXML(rapidxml::xml_document<char>* pdoc);
    protected:
        MEMBER_VAR(std::string, axis);
        
    }; // class RigVF
//
////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////
// class RigStiffJoints
    class RigStiffJoints : public Rig {
    public:
        RigStiffJoints();
        virtual ~RigStiffJoints();

        virtual void changePose();
// XML functions
        RigStiffJoints* readXML(rapidxml::xml_node<char>* node);
        // virtual rapidxml::xml_node<char>* writeXML(rapidxml::xml_document<char>* pdoc);
    protected:
        MEMBER_VAR(std::string, axis);
        
    }; // class RigStiffJoints
//
////////////////////////////////////////////////////////////
    
} // namespace controller

#endif // #ifndef controller_RIG_H

