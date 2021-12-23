#include "jiminy/core/robot/Robot.h"
#include "jiminy/core/Macros.h"

#include "jiminy/core/utilities/Pinocchio.h"
#include "jiminy/core/robot/AbstractTransmission.h"


namespace jiminy
{
    AbstractTransmissionBase::AbstractTransmissionBase(std::string const & name) :
    baseTransmissionOptions_(nullptr),
    transmissionOptionsHolder_(),
    isInitialized_(false),
    isAttached_(false),
    robot_(),
    notifyRobot_(),
    name_(name),
    transmissionIdx_(-1),
    jointNames_(),
    jointModelIndices_(),
    jointTypes_(),
    jointPositionIndices_(),
    jointVelocityIndices_(),
    motorNames_(),
    motorIndices_(),
    motors_(),
    forwardJacobian_(),
    backwardJacobian_()
    {
        // TODO initialize the options
    }

    AbstractTransmissionBase::~AbstractTransmissionBase(void)
    {
        // Detach the transmission before deleting it if necessary
        if (isAttached_)
        {
            detach();
        }
    }

    hresult_t AbstractTransmissionBase::initialize(std::vector<std::string> const & jointNames,
                                                   std::vector<std::string> const & motorNames)
    {
        // Copy reference to joint and motors names
        hresult_t returnCode = hresult_t::SUCCESS;
        jointNames_ = jointNames;
        motorNames_ = motorNames;
        isInitialized_ = true;
        returnCode = refreshProxies();
        if (returnCode != hresult_t::SUCCESS)
        {
            jointNames_.clear();
            motorNames_.clear();
            isInitialized_ = false;
        }

        // TODO check whether transmission is invertible or not

        // Make sure the joint is not already attached to a transmission
        auto robot = robot_.lock();
        std::vector<std::string> actuatedJointNames = robot->getActuatedJointNames();
        for (std::string const & transmissionJoint : getJointNames())
        {
            auto transmissionJointIt = std::find(actuatedJointNames.begin(), actuatedJointNames.end(), transmissionJoint);
            if (transmissionJointIt != actuatedJointNames.end())
            {
                PRINT_ERROR("Joint already attached to another transmission");
                return hresult_t::ERROR_GENERIC;
            }
        }

        // Propagte the actuated joints
        if (notifyRobot_)
            {
                // problem here, it does not work I think it is the const
                returnCode = notifyRobot_(actuatedJointNames, robot->getTransmissions());
            }
        return returnCode;
    }

    hresult_t AbstractTransmissionBase::attach(std::weak_ptr<Robot const> robot,
                                               std::function<hresult_t(std::vector<std::string> & /*actuatedJointNames*/,
                                                                       transmissionsHolder_t & /*transmissionsHolder*/)> notifyRobot)
    {
        // Make sure the transmission is not already attached
        if (isAttached_)
        {
            PRINT_ERROR("Transmission already attached to a robot. Please 'detach' method before attaching it.");
            return hresult_t::ERROR_GENERIC;
        }

        // Make sure the robot still exists
        if (robot.expired())
        {
            PRINT_ERROR("Robot pointer expired or unset.");
            return hresult_t::ERROR_GENERIC;
        }


        // Copy references to the robot and shared data
        robot_ = robot;
        notifyRobot_ = notifyRobot;

        // Update the flag
        isAttached_ = true;

        return hresult_t::SUCCESS;
    }

    hresult_t AbstractTransmissionBase::detach(void)
    {
        if (!isAttached_)
        {
            PRINT_ERROR("Transmission not attached to any robot.");
            return hresult_t::ERROR_GENERIC;
        }

        // Clear the references to the robot
        robot_.reset();
        notifyRobot_ = nullptr;

        // Unset the Id
        transmissionIdx_ = -1;

        // Reset constant members
        jointNames_.clear();
        jointModelIndices_.clear();
        jointTypes_.clear();
        jointPositionIndices_.clear();
        jointVelocityIndices_.clear();
        motorNames_.clear();
        motorIndices_.clear();

        // Update the flags
        isAttached_ = false;
        isInitialized_ = false;

        return hresult_t::SUCCESS;
    }

    hresult_t AbstractTransmissionBase::setOptions(configHolder_t const & /*transmissionOptions*/)
    {
        // TODO SetOptions
        return hresult_t::SUCCESS;
    }

    configHolder_t AbstractTransmissionBase::getOptions(void) const
    {
        return transmissionOptionsHolder_;
    }

    hresult_t AbstractTransmissionBase::refreshProxies(void)
    {
        hresult_t returnCode = hresult_t::SUCCESS;

        if (!isAttached_)
        {
            PRINT_ERROR("Transmission not attached to any robot. Impossible to refresh proxies.");
            returnCode = hresult_t::ERROR_INIT_FAILED;
        }

        auto robot = robot_.lock();
        if (returnCode == hresult_t::SUCCESS)
        {
            if (!robot)
            {
                PRINT_ERROR("Robot has been deleted. Impossible to refresh proxies.");
                returnCode = hresult_t::ERROR_GENERIC;
            }
        }

        if (returnCode == hresult_t::SUCCESS)
        {
            if (!isInitialized_)
            {
                PRINT_ERROR("Transmission not initialized. Impossible to refresh proxies.");
                returnCode = hresult_t::ERROR_INIT_FAILED;
            }
        }

        if (returnCode == hresult_t::SUCCESS)
        {
            if (!robot->getIsInitialized())
            {
                PRINT_ERROR("Robot not initialized. Impossible to refresh proxies.");
                returnCode = hresult_t::ERROR_INIT_FAILED;
            }
        }

        jointIndex_t jointModelIdxTmp;
        for (unsigned int i = 0; i < jointNames_.size(); i++)
        {
            if (returnCode == hresult_t::SUCCESS)
            {
                // jointModelIndices_ is empty at this point, therefore inject a 0 and later get the right index
                jointModelIndices_.push_back(jointModelIdxTmp);
                returnCode = ::jiminy::getJointModelIdx(robot->pncModel_, jointNames_[i], jointModelIndices_[i]);
            }
        }

        joint_t jointType;
        for (unsigned int i = 0; i < jointNames_.size(); i++)
        {
            if (returnCode == hresult_t::SUCCESS)
            {
                // jointTypes_ is empty at this point, therefore inject a 0 and later get the right type
                jointTypes_.push_back(jointType);
                returnCode = getJointTypeFromIdx(robot->pncModel_, jointModelIndices_[i], jointTypes_[i]);
            }
        }

        for (unsigned int i = 0; i < jointNames_.size(); i++)
        {
            if (returnCode == hresult_t::SUCCESS)
            {
                // Transmissions are only supported for linear and rotary joints
                if (jointTypes_[i] != joint_t::LINEAR && jointTypes_[i] != joint_t::ROTARY && jointTypes_[i] != joint_t::ROTARY_UNBOUNDED)
                {
                    PRINT_ERROR("A transmission can only be associated with a 1-dof linear or rotary joint.");
                    returnCode = hresult_t::ERROR_BAD_INPUT;
                }
            }
        }

        // Populate motorIndices_
        std::weak_ptr<AbstractMotorBase const> motor;
        for (std::string const & motorName : getMotorNames())
        {
            if (returnCode == hresult_t::SUCCESS)
            {
                returnCode = robot->getMotor(motorName, motor);
                auto motorPtr = motor.lock();
                if (!motorPtr)
                {
                    PRINT_ERROR("No motor found with this name.");
                    returnCode = hresult_t::ERROR_GENERIC;
                }
                std::size_t idx = (motorPtr->getIdx());
                motorIndices_.push_back(idx);
            }
        }

        // Populate jointPositionIndices_ and jointVelocityIndices_
        jointPositionIndices_.clear();
        jointVelocityIndices_.clear();
        for (std::string const & jointName : jointNames_)
        {
            std::vector<int32_t> jointPositionIdx;
            if (!robot->pncModel_.existJointName(jointName))
            {
                PRINT_ERROR("Joint '", jointName, "' not found in robot model.");
                return hresult_t::ERROR_BAD_INPUT;
            }
            if (returnCode == hresult_t::SUCCESS)
            {
                returnCode = getJointPositionIdx(robot->pncModel_, jointName, jointPositionIdx);
            }
            if (returnCode == hresult_t::SUCCESS)
            {
                jointPositionIndices_.insert(jointPositionIndices_.end(), jointPositionIdx.begin(), jointPositionIdx.end());
            }

            // Extract velocity indices
            std::vector<int32_t> jointVelocityIdx;
            jointIndex_t const & jointModelIdx = robot->pncModel_.getJointId(jointName);
            int32_t const & jointVelocityFirstIdx = robot->pncModel_.joints[jointModelIdx].idx_v();
            int32_t const & jointNv = robot->pncModel_.joints[jointModelIdx].nv();
            jointVelocityIdx.resize(jointNv);
            std::iota(jointVelocityIdx.begin(), jointVelocityIdx.end(), jointVelocityFirstIdx);
            jointVelocityIndices_.insert(jointVelocityIndices_.end(), jointVelocityIdx.begin(), jointVelocityIdx.end());
        }

        return returnCode;
    }

    bool_t const & AbstractTransmissionBase::getIsInitialized(void) const
    {
        return isInitialized_;
    }

    std::string const & AbstractTransmissionBase::getName(void) const
    {
        return name_;
    }

    int32_t const & AbstractTransmissionBase::getIdx(void) const
    {
        return transmissionIdx_;
    }

    std::vector<std::string> const & AbstractTransmissionBase::getJointNames(void) const
    {
        return jointNames_;
    }

    std::vector<jointIndex_t> const & AbstractTransmissionBase::getJointModelIndices(void) const
    {
        return jointModelIndices_;
    }

    std::vector<joint_t> const & AbstractTransmissionBase::getJointTypes(void) const
    {
        return jointTypes_;
    }

    std::vector<int32_t> const & AbstractTransmissionBase::getJointPositionIndices(void) const
    {
        return jointPositionIndices_;
    }

    std::vector<int32_t> const & AbstractTransmissionBase::getJointVelocityIndices(void) const
    {

        return jointVelocityIndices_;
    }

    std::vector<std::string> const & AbstractTransmissionBase::getMotorNames(void) const
    {
        return motorNames_;
    }

    std::vector<std::size_t> const & AbstractTransmissionBase::getMotorIndices(void) const
    {
        return motorIndices_;
    }

    matrixN_t const & AbstractTransmissionBase::getJacobian(void) const
    {
        return forwardJacobian_;
    }

    matrixN_t const & AbstractTransmissionBase::getInverseJacobian(void) const
    {
        return backwardJacobian_;
    }

    hresult_t AbstractTransmissionBase::computeForward(float64_t const & /*t*/,
                                                       vectorN_t & /*q*/,
                                                       vectorN_t & /*v*/,
                                                       vectorN_t & /*a*/,
                                                       vectorN_t & /*uJoint*/)
    {
        // TODO
        return hresult_t::SUCCESS;

    }

    hresult_t AbstractTransmissionBase::computeBackward(float64_t const & /*t*/,
                                                        vectorN_t const & /*q*/,
                                                        vectorN_t const & /*v*/,
                                                        vectorN_t const & /*a*/,
                                                        vectorN_t const & /*uJoint*/)
    {
        // TODO
        return hresult_t::SUCCESS;
    }

}
