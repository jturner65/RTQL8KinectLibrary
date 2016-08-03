/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "simpack.h"

#include "utils/Paths.h"
#include "toolkit/Toolkit.h"
#include "common/app_cppcommon.h"
#include "rapidxml_helper.h"

#include "app_controller_state.h"
#include "app_composite_controller.h"
#include "cost.h"
#include "dynamics/ConstraintDynamics.h"
#include "dynamics/PointConstraint.h"
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/SkeletonDynamics.h"


namespace controller {
////////////////////////////////////////////////////////////
// class SimPack implementation
    SimPack::SimPack()
        : MEMBER_INIT_NULL(sim)
        , MEMBER_INIT_NULL(initSimState)
        , MEMBER_INIT_NULL(initConState)
        , MEMBER_INIT_NULL(conState)
        , MEMBER_INIT_NULL(con)
        , barEnabled(false)
        , barThreshold(0.05)
    {
    }
    
    SimPack::~SimPack() {
    }

// Main functions
    void SimPack::reset() {
        sim()->reset();
        // sim()->setSimState( *initSimState() );
        rtql8::toolkit::SimState state = *initSimState();
        state.skels[1].q = con()->getInitialPose();
        sim()->setSimState( state );
        con()->reset();

        // Duplicate initial controller state
        if (conState()) { delete conState(); }
        set_conState( new AppControllerState() );
        *conState() = *initConState();
        conState()->target
            = conState()->pushedTarget = con()->getInitialTargetPose();
        
        con()->setConState( conState() );
        // LOG_INFO << FUNCTION_NAME() << " OK";
    }
    
    bool SimPack::step() {
        if (con()->isTerminated()) {
            return false;
        }
        if (barEnabled) {
            checkBar();
            checkReleaseBar();
        }
        con()->control();
        sim()->step();
        return true;
    }
    
    void SimPack::simulate() {
        while(1) {
            bool isSimulated = this->step();
            if (!isSimulated) {
                break;
            }
        }
    }

    void SimPack::checkBar() {
        using namespace rtql8::kinematics;
        using namespace rtql8::dynamics;
        if (con()->shouldReleaseBar()) {
            return;
        }
        
        ConstraintDynamics* constraints = sim()->getConstraintDynamics(1);
        bool leftGrapped = false;
        bool rightGrapped = false;
        for (int i = 0; i < constraints->numConstraints(); i++) {
            PointConstraint* c = (PointConstraint*)constraints->getConstraint(i);
            int idx = c->bodyIndex();
            if (idx == 15) {
                leftGrapped = true;
            } else if (idx == 19) {
                rightGrapped = true;
            }
        }

        double dt = sim()->getSimConfig().Dt;
        Eigen::Vector3d bar = this->barPos;
        SkeletonDynamics* skel = sim()->skel(1);
        BodyNodeDynamics* lhand = (BodyNodeDynamics*)skel->getNode(15);
        Eigen::Vector3d lpos(lhand->getWorldCOM());
        lpos(2) = 0.0;
        BodyNodeDynamics* rhand = (BodyNodeDynamics*)skel->getNode(19);
        Eigen::Vector3d rpos(rhand->getWorldCOM());
        rpos(2) = 0.0;

        double ldist = (bar - lpos).norm();
        if ( ldist < this->barThreshold && leftGrapped == false) {
            // cout << "Grap the left!!!!" << endl;
            PointConstraint *point1 = new PointConstraint(skel, lhand, lhand->getLocalCOM(), lhand->getWorldCOM(), true, dt);
            constraints->addConstraint(point1);

        }


        double rdist = (bar - rpos).norm();
        if ( rdist < this->barThreshold && rightGrapped == false) {
            // cout << "Grap the right!!!!" << endl;
            PointConstraint *point1 = new PointConstraint(skel, rhand, rhand->getLocalCOM(), rhand->getWorldCOM(), true, dt);
            constraints->addConstraint(point1);

        }

        double maxdist = std::max(ldist, rdist);
        rtql8::toolkit::SkelState& state = sim()->getSkelState(1);
        if (maxdist < state.minBarDist) {
            state.minBarDist = maxdist;
        }
        
    }

    void SimPack::checkReleaseBar() {
        using namespace rtql8::kinematics;
        using namespace rtql8::dynamics;
        if (con()->shouldReleaseBar() == false) {
            return;
        }
        ConstraintDynamics* constraints = sim()->getConstraintDynamics(1);
        if (constraints->numConstraints() == 0) {
            return;
        }
        // cout << "Release the bars!!!" << endl;
        sim()->clearAllConstraints();
    }


    Eigen::VectorXd SimPack::targetPose() {
        return conState()->qhat;
    }

    void SimPack::setTargetPose(const Eigen::VectorXd& pose) {
        conState()->qhat = pose;
    }

    void SimPack::setController(AppCompositeController* c) {
        set_con(c);
    }

    double SimPack::evaluateController(AppCompositeController* c, rtql8::toolkit::SimState* s) {
        // cout << FUNCTION_NAME() << endl;
        sim()->setSimState(*s);
        AppControllerState* cs = new AppControllerState();
        *cs = *initConState();
        // cout << "setConState.." << endl;
        con()->setConState( cs );
        // cout << "evaluateController = " << con()->toString() << endl;
        double ret = con()->evaluate();
        // cout << "ret = " << ret << endl;
        delete cs;
        return ret;
    }

    void SimPack::evaluateController(AppCompositeController* c, rtql8::toolkit::SimState* s,
                                     double* task, double* error) {
        // cout << FUNCTION_NAME() << endl;
        sim()->setSimState(*s);
        AppControllerState* cs = new AppControllerState();
        *cs = *initConState();
        con()->setConState( cs );
        con()->evaluateTaskAndError(task, error);
        // cout << "cost object = " << con()->cost()->toString() << endl;
    }

    void SimPack::evaluateController(AppCompositeController* c, rtql8::toolkit::SimState* s,
                                     double task, double* error) {
        // cout << FUNCTION_NAME() << endl;
        sim()->setSimState(*s);
        AppControllerState* cs = new AppControllerState();
        *cs = *initConState();
        con()->setConState( cs );
        double ret = con()->evaluate(task);
        (*error) = ret;
        // cout << "cost object = " << con()->cost()->toString() << endl;
    }


// Auxiliary functions
    std::string SimPack::toString() const {
        std::string ret("[SimPack]");
        return ret;
    }
    
// XML functions
    /*
      <simulation dt="0.0005" mu="1.0" cfm="0.0">
      <ground filename="/skel/ground1.skel" />
      <character filename="/skel/fullbody2.skel">
      <kp value="[0, 0, 0, 0, 0, 0, 300, 300, 200, 100, 100, 100, 300, 300, 200, 100, 100, 100, 300, 300, 200, 50, 50, 200, 100, 100, 50, 30, 200, 100, 100, 50, 30]" />
      <kd value="[0, 0, 0, 0, 0, 0, 24.4949, 24.4949, 22.3607, 28.2843, 28.2843, 28.2843, 24.4949, 24.4949, 22.3607, 28.2843, 28.2843, 28.2843, 17.3205, 17.3205, 14.1421, 7.07107, 7.07107, 14.1421, 10, 10, 7.07107, 5.47723, 14.1421, 10, 10, 7.07107, 5.47723]" />
      <torquelimit value="0, 0, 0, 0, 0, 0, 800.00, 600.00, 500.00, 100.00, 100.00, 100.00, 800.00, 600.00, 500.00, 100.00, 100.00, 100.00, 1000.00, 2000.00, 1000.00, 500.00, 500.00, 800.00, 700.00, 500.00, 300.00, 100.00, 800.00, 700.00, 500.00, 300.00, 100.00" />
      </character>
      </simulation>
      
     */
    SimPack* SimPack::readXML(rapidxml::xml_node<char>* node, bool verbose) {
        // Load simulation parameters
        double dt  = attr_double(node, "dt");
        double mu  = attr_double(node, "mu");
        double cfm = attr_double(node, "cfm");
        bool hasMaxmove = has_attr(node, "maxmove");
        double maxmove = 0.0;
        if (hasMaxmove){
            maxmove = attr_double(node, "maxmove");
        }

        int collisionRefreshRate = -1;
        if (has_attr(node, "collisionRefreshRate")) {
            collisionRefreshRate = attr_double(node, "collisionRefreshRate");
        }

        VLOG_INFO << FUNCTION_NAME() << " : " << dt << ", " << mu << ", " << cfm;
        VLOG_INFO << FUNCTION_NAME() << " : maxmove = " << maxmove;
        VLOG_INFO << FUNCTION_NAME() << " : collisionRefreshRate = " << maxmove;
        rtql8::toolkit::SimConfig config(dt, mu, cfm);
        config.collisionRefreshRate = collisionRefreshRate;
        
        rapidxml::xml_node<char>* node_gr = node->first_node("ground");
        rapidxml::xml_node<char>* node_ch = node->first_node("character");

        // Configure the skeletons
        std::string path(RTQL8_DATA_PATH);
        filenames.push_back( attr(node_gr, "filename") );
        filenames.push_back( attr(node_ch, "filename") );
        std::string ground    = path + attr(node_gr, "filename");
        std::string character = path + attr(node_ch, "filename");
        VLOG_INFO << FUNCTION_NAME() << " : ground = " << ground;
        VLOG_INFO << FUNCTION_NAME() << " : character = " << character;
        config.skels.push_back(
            rtql8::toolkit::SkelConfig(ground.c_str(), RTQL8_SKEL_IMMOBILE) );
        config.skels.push_back(
            rtql8::toolkit::SkelConfig(character.c_str()) );

        // Configure torque limit
        Eigen::VectorXd torqueLimit = attr_vx(node_ch->first_node("torquelimit"), "value");
        if (has_attr(node_ch->first_node("torquelimit"), "scale")) {
            double scale = attr_double(node_ch->first_node("torquelimit"), "scale");
            LOG_INFO << "scale = " << scale;
            torqueLimit *= scale;
        }
        VLOG_INFO << "torqueLimit = " << IO(torqueLimit);
        config.skels.getLast().torqueLimit = torqueLimit;

        // Create a simulator
        set_sim( new rtql8::toolkit::Simulator(config, false) );
        VLOG_INFO << "create simulator OK : " << sim();

        // Create an initial state
        set_initSimState( new rtql8::toolkit::SimState );
        *initSimState() = sim()->createEmptyState();

        initSimState()->skels[1].q <<
            0, 0.98, 0, 0.02, 0, 0.0, // Root dofs
            0.2, 0.1, -0.5, 0.2, 0, 0, // Left leg
            0.2, -0.1, -0.5, 0.2, 0, 0, // Right leg
            0.0, -0.25, 0, 0, 0, // Spine to Head
            0, 0, 0, 0, 0, // Left arm
            0, 0, 0, 0, 0; // Right arm
        sim()->setSimState(*initSimState());

        VLOG_INFO << "set initial simstate OK";

        // Create an inital controller state
        Eigen::VectorXd kp = attr_vx(node_ch->first_node("kp"), "value");
        Eigen::VectorXd kd = attr_vx(node_ch->first_node("kd"), "value");
        if (has_attr(node_ch->first_node("kp"), "scale")) {
            double scale = attr_double(node_ch->first_node("kp"), "scale");
            LOG_INFO << "kp.scale = " << scale;
            kp *= scale;
        }
        if (has_attr(node_ch->first_node("kd"), "scale")) {
            double scale = attr_double(node_ch->first_node("kd"), "scale");
            LOG_INFO << "kd.scale = " << scale;
            kd *= scale;
        }
        VLOG_INFO << "kp = " << IO(kp);
        VLOG_INFO << "kd = " << IO(kd);

        // ////////////////////////////////////////////////////////////////
        // // Debug code for modifying kp and kd
        // for (int i = 0; i < kp.size(); i++) {
        //     cout << i << " : ";
        //     cout << kp(i) << " " << kd(i) << " --> ";
        //     double a = kp(i) * 1.5;
        //     double b = sqrt(a);
        //     cout << a << " " << b << endl;
        //     kp(i) = a;
        //     kd(i) = b;
        // }
        // cout << "<kp value=\"" << IO(kp) << "\" />" << endl;
        // cout << "<kd value=\"" << IO(kd) << "\" />" << endl;
        // exit(0);
        // ////////////////////////////////////////////////////////////////
        set_initConState(
            new AppControllerState( initSimState()->skels[1].q, kp, kd )
            );

        if (hasMaxmove) {
            initConState()->maxmove = maxmove;
        }

        rapidxml::xml_node<char>* node_bar = node->first_node("bar");
        if (node_bar) {
            this->barEnabled = true;
            this->barPos = attr_vx(node_bar, "pos");
            this->barThreshold = attr_double(node_bar, "threshold");
            LOG_INFO << "Bar detected: pos = " << IO(barPos)
                     << " threshold= " << barThreshold;
        }


        const int SID = 1;
        initConState()->sim = sim();
        initConState()->skelId = SID;
        VLOG_INFO << "set initial controller state OK: nDofs = " << initConState()->nDofs();

        VLOG_INFO << FUNCTION_NAME() << " OK";

        return this;
    }
    

    rapidxml::xml_node<char>* SimPack::writeXML(rapidxml::xml_document<char>* pdoc) {
        namespace rx = rapidxml;
        rx::xml_node<char>* node = rx::alloc_node(pdoc, "simulation");
        rtql8::toolkit::SimConfig& cfg = sim()->getSimConfig();
        rx::add_attr(pdoc, node, "dt",  cfg.Dt);
        rx::add_attr(pdoc, node, "mu",  cfg.Mu);
        rx::add_attr(pdoc, node, "cfm", cfg.Cfm);
        double maxmove = initConState()->maxmove;
        // cout << "maxmove = " << maxmove
        //      << "( default = " << DEFAULT_MAXMOVE << ")" << endl;
        if (fabs(DEFAULT_MAXMOVE - maxmove) > 0.001) {
            // cout << "add maxmove to node" << endl;
            rx::add_attr(pdoc, node, "maxmove",  maxmove);
        }
        if (cfg.collisionRefreshRate != -1) {
            rx::add_attr_int(pdoc, node, "collisionRefreshRate", cfg.collisionRefreshRate);
        }

        if (barEnabled) {
            rx::xml_node<char>* node_bar = rx::add_node(pdoc, node, "bar");
            rx::add_attr(pdoc, node_bar, "pos", barPos);
            rx::add_attr(pdoc, node_bar, "threshold", barThreshold);
        }
        
        rx::xml_node<char>* node_ground = rx::add_node(pdoc, node, "ground");
        rx::add_attr(pdoc, node_ground, "filename", filenames[0].c_str());

        rx::xml_node<char>* node_character = rx::add_node(pdoc, node, "character");
        rx::add_attr(pdoc, node_character, "filename", filenames[1].c_str());

        {
            rx::xml_node<char>* node_kp = rx::add_node(pdoc, node_character, "kp");
            rx::add_attr(pdoc, node_kp, "value", initConState()->kp);
            rx::xml_node<char>* node_kd = rx::add_node(pdoc, node_character, "kd");
            rx::add_attr(pdoc, node_kd, "value", initConState()->kd);
            rx::xml_node<char>* node_torquelimit = rx::add_node(pdoc, node_character, "torquelimit");
            rx::add_attr(pdoc, node_torquelimit, "value",cfg.skels.getLast().torqueLimit);
        }
        

        return node;
    }

// class SimPack ends
////////////////////////////////////////////////////////////
    
    
    
} // namespace controller



