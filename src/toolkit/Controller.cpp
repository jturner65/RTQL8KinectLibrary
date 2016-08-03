#include "Controller.h"
#include "CppCommon.h"

#include <cstdarg>

// rtql8 headers
#include "dynamics/BodyNodeDynamics.h"
#include "dynamics/SkeletonDynamics.h"
#include "dynamics/ContactDynamics.h"
#include "utils/UtilsRotation.h"
// rtql8 extension headers
#include "Simulator.h"

namespace rtql8 {
    namespace toolkit {
        Controller::Controller()
            : mSim(NULL), mSkelId(-1)
        {
        }

        Controller::Controller(Simulator* _sim, int _skelId) 
            : mSim(NULL), mSkelId(-1)
        {
            setSimulator(_sim);
            setSkelId(_skelId);
        }
        
        Controller::~Controller() {
            // Do not release simulator
        }
        
            
        void Controller::setSimulator(Simulator* _sim) {
            this->mSim = _sim;
        }
        
        Simulator* Controller::getSimulator() const {
            return this->mSim;
        }
        
        void Controller::setSkelId(int skelId) {
            this->mSkelId = skelId;
        }
        
        int Controller::getSkelId() const {
            return this->mSkelId;
        }
        
        dynamics::SkeletonDynamics* Controller::skel() const {
            if (!getSimulator()) return NULL;
            if (getSkelId() < 0 || getSkelId() >= getSimulator()->nSkels()) return NULL;
            return getSimulator()->skel(getSkelId());
        }

        const SimConfig& Controller::getSimConfig() const {
            return getSimulator()->getSimConfig();
        }

        const SkelConfig& Controller::getSkelConfig() const {
            return getSimulator()->getSkelConfig(getSkelId());
        }

        const SimState& Controller::getSimState() const {
            return getSimulator()->getSimState();
        }
        
        const SkelState& Controller::getSkelState() const {
            return getSimulator()->getSkelState(getSkelId());
        }

        void Controller::control() {
            if (!skel()) {
                // Should I warn you?
                return;
            }
            Eigen::VectorXd torque = computeTorque();
            skel()->setInternalForces(torque);
        }

        int Controller::nDofs() const {
            return skel()->getNumDofs();
        }

        int Controller:: nNodes() const {
            return skel()->getNumNodes();
        }

        double Controller::t() const {
            return getSimState().t;
        }

        double Controller::h() const {
            return getSimulator()->getSimConfig().Dt;
        }
        
        Eigen::VectorXd Controller::q() const {
            return getSkelState().q;
        }
        
        Eigen::VectorXd Controller::qdot() const {
            return getSkelState().qdot;
        }

        
        Eigen::MatrixXd Controller::M() const {
            return skel()->getMassMatrix();
        }
        
        Eigen::VectorXd Controller::C() const {
            return skel()->getCombinedVector();
        }
        
        double Controller::m() const {
            return skel()->getMass();
        }
        
        Eigen::Vector3d Controller::COM() const {
            return getSkelState().COM;
        }

        Eigen::Vector3d Controller::MAXCOM() const {
            return getSkelState().MAXCOM;
        }

        Eigen::Vector3d Controller::MINCOM() const {
            return getSkelState().MINCOM;
        }

        Eigen::Vector3d Controller::COMdot() const {
            return P() / m();
        }
        
        Eigen::Vector3d Controller::P() const {
            return getSkelState().P;
        }
        
        Eigen::Vector3d Controller::L() const {
            return getSkelState().L / m();
        }
        
        Eigen::Vector3d Controller::COP() const {
            return getSkelState().COP;
        }
        
        int Controller::nContacts() const {
            // return getSkelState().nT;
            return getSimState().nT();
        }

        rtql8::dynamics::BodyNodeDynamics* Controller::node(int nodeId) const {
            return dynamic_cast<rtql8::dynamics::BodyNodeDynamics*>(skel()->getNode(nodeId));
        }

        Eigen::Vector3d Controller::POS(int nodeId) const {
            return skel()->getNode(nodeId)->evalWorldPos( Eigen::Vector3d::Zero() );
        }

        Eigen::Vector3d Controller::APOS(int nNodes, ...) const {
            Eigen::Vector3d pos(Eigen::Vector3d::Zero());
            va_list ap;
            va_start(ap, nNodes);
            for (int i = 0; i < nNodes; i++) {
                int nodeId = va_arg(ap, int);
                pos += POS(nodeId);
            }
            pos /= (double)nNodes;
            va_end(ap);
            return pos;
        }

        Eigen::VectorXd Controller::zero() const {
            return Eigen::VectorXd::Zero( nDofs() );
        }

        double Controller::barDist() const {
            return getSkelState().minBarDist;
        }
        
        
        Eigen::Matrix3d Controller::getLocalFrame() const {
            using namespace rtql8::utils::rotation;
            Eigen::Vector3d em = q().segment<3>(3);
            Eigen::Quaterniond quat = expToQuat(em);
            Eigen::Matrix3d R = quatToMatrix(quat);
            Eigen::Vector3d euler = matrixToEuler(R, XZY);
            double yRot = euler(2);
            Eigen::Matrix3d M = eulerToMatrixY(yRot);
            return M;
        }

        Eigen::Vector3d Controller::LOCAL(const Eigen::Vector3d& v) const {
            Eigen::Matrix3d M = getLocalFrame();
            return M.inverse() * v;
        }


    } // namespace toolkit
} // namespace rtql8

