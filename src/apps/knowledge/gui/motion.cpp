/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#include "motion.h"
#include "common/app_cppcommon.h"
#include "common/render_cppcommon.h"

// rtql8 headers
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/SkeletonDynamics.h"
#include "toolkit/Toolkit.h"

namespace gui {
    
////////////////////////////////////////////////////////////
// class Motion implementation
    Motion::Motion()
        : MEMBER_INIT(packed, false)
        , MEMBER_INIT_NULL(replay)
    {
    }
    
    Motion::Motion(rtql8::toolkit::SimReplay* _replay)
        : MEMBER_INIT(packed, false)
        , MEMBER_INIT_ARG(replay)
    {
        set_offset( Eigen::Vector3d::Zero() );
    }
    
    Motion::~Motion() {
    }

    void Motion::pack() {
        LOG_INFO << "pack() : # frames = " << replay()->frames.size();
        const int SKEL = 1;
        const int STEP = 30;
        for (int i = 0; i < replay()->frames.size(); i += STEP) {
            Eigen::VectorXd pose = replay()->frames[i].skels[SKEL].q;
            poses.push_back(pose);
        }
        set_packed(true);
        LOG_INFO << "pack() OK";
    }

    int Motion::g_index = 0;

    void Motion::globalStep(std::vector<Motion>& motions) {
        int sz = 0;
        for (int i = 0; i < motions.size(); i++) {
            Motion& motion = motions[i];
            sz = std::max(sz, (int)motion.poses.size());
        }
        g_index++;
        if (g_index > 2 * sz) {
            g_index = 0;
        }
    }

    void Motion::step() {
        // index = (index + 1) % (2 * poses.size());
        index = g_index;
        if (index < poses.size()) {
            Eigen::VectorXd q = poses[index];
            skel()->setPose( q );
        } else {
            if (poses.size() > 0) {
                Eigen::VectorXd q = poses[poses.size() - 1];
                skel()->setPose( q );
            }
        }

    }
    
    void Motion::render(rtql8::renderer::RenderInterface* ri) {
        glBlock() {
            rtql8::toolkit::glTranslated( offset() );
            skel()->draw(ri);
        }
    }

    void Motion::renderHeadTrajectory() {
        glColor4d(0.0, 0.9, 0.9, 0.5);
        for (int i = 0; i < heads.size(); i += 100) {
            glBlock() {
                rtql8::toolkit::glTranslated( offset() );
                rtql8::toolkit::glTranslated( heads[i] );
                glutSolidSphere(0.03, 4, 4);
            }
        }
    }

    void Motion::render(rtql8::renderer::RenderInterface* ri,
                        const Eigen::VectorXd& pose) {
        skel()->setPose(pose);
        glBlock() {
            rtql8::toolkit::glTranslated( offset() );
            skel()->draw(ri);
        }

    }

// Skeleton static members and functions

    void Motion::initSkeleton(const char* const filename) {
        g_model = new rtql8::kinematics::FileInfoSkel
            <rtql8::dynamics::SkeletonDynamics>();
        bool result = g_model->loadFile(filename, rtql8::kinematics::SKEL);
        if (!result) {
            LOG_FATAL << "Cannot load a skeleton for visualization";
            exit(0);
        }
        LOG_INFO << "load a skeleton for visualization OK.";
    }

    rtql8::dynamics::SkeletonDynamics* Motion::skel() {
        return g_model->getSkel();
    }

    rtql8::kinematics::FileInfoSkel<rtql8::dynamics::SkeletonDynamics>* Motion::g_model = NULL;


// class Motion ends
////////////////////////////////////////////////////////////
    

    
} // namespace gui



