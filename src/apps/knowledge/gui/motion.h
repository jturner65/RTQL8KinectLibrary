/* RTQL8, Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Sehoon Ha <sha9@gatech.edu>
 * Georgia Tech Graphics Lab
 */

#ifndef GUI_MOTION_H
#define GUI_MOTION_H

#include <Eigen/Dense>
#include "common/app_hppcommon.h"
#include "kinematics/FileInfoSkel.hpp"

namespace rtql8 {
    namespace renderer {
        class RenderInterface;
    } // namespace renderer

    namespace dynamics {
        class SkeletonDynamics;
    } // namespace dynamics 

    namespace toolkit {
        struct SimReplay;
    } // namespace toolkit
} // namespace rtql8


namespace gui {

    class Motion {
    public:
        Motion();
        Motion(rtql8::toolkit::SimReplay* _replay);
        virtual ~Motion();

        void pack();
        static void globalStep(std::vector<Motion>& motions);
        void step();
        void render(rtql8::renderer::RenderInterface* ri);
        void render(rtql8::renderer::RenderInterface* ri,
                    const Eigen::VectorXd& pose);
        void renderHeadTrajectory();
        void setHeadPositions(std::vector<Eigen::Vector3d>& h) { heads = h; }
        
    protected:
        std::vector<Eigen::VectorXd> poses;
        std::vector<Eigen::Vector3d> heads;
        int index;
        static int g_index;
        MEMBER_VAR(bool, packed);
        MEMBER_VAR(Eigen::Vector3d, offset);
        MEMBER_PTR(rtql8::toolkit::SimReplay*, replay);

// Skeleton static members and functions
    public:
        static void initSkeleton(const char* const filename);
        static rtql8::kinematics::FileInfoSkel<rtql8::dynamics::SkeletonDynamics>* g_model;
        static rtql8::dynamics::SkeletonDynamics* skel();
    }; // class Motion
    
} // namespace gui

#endif // #ifndef GUI_MOTION_H

