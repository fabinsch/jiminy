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
        // TODO check whether transmission is invertible or not

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


        // Make sure the joint is not already attached to a transmission
        auto robot = robot_.lock();
        std::vector<std::string> actuatedJointNames = robot->getActuatedJointNames();
        for (std::string const & transmissionJoint : jointNames)
        {
            auto transmissionJointIt = std::find(actuatedJointNames.begin(), actuatedJointNames.end(), transmissionJoint);
            if (transmissionJointIt != actuatedJointNames.end())
            {
                PRINT_ERROR("Joint already attached to another transmission");
                return hresult_t::ERROR_GENERIC;
            }
        }

        return returnCode;
    }

    hresult_t AbstractTransmissionBase::attach(std::weak_ptr<Robot const> robot)
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

        jointIndex_t jointModelIdx;
        for (unsigned int i = 0; i < jointNames_.size(); i++)
        {
            if (returnCode == hresult_t::SUCCESS)
            {
                returnCode = ::jiminy::getJointModelIdx(robot->pncModel_, jointNames_[i], jointModelIdx);
                jointModelIndices_.push_back(jointModelIdx);
            }
        }

        joint_t jointType;
        for (unsigned int i = 0; i < jointNames_.size(); i++)
        {
            if (returnCode == hresult_t::SUCCESS)
            {
                returnCode = getJointTypeFromIdx(robot->pncModel_, jointModelIndices_[i], jointType);
                jointTypes_.push_back(jointType);
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
                std::size_t motorIdx = motorPtr->getIdx();
                motorIndices_.push_back(motorIdx);
            }
        }

        // Populate jointPositionIndices_ and jointVelocityIndices_
        jointPositionIndices_.clear();
        jointVelocityIndices_.clear();
        for (std::string const & jointName : jointNames_)
        {
            std::vector<int32_t> jointPositionIdx;
            std::vector<int32_t> jointVelocityIdx;
            if (!robot->pncModel_.existJointName(jointName))
            {
                PRINT_ERROR("Joint '", jointName, "' not found in robot model.");
                return hresult_t::ERROR_BAD_INPUT;
            }
            if (returnCode == hresult_t::SUCCESS)
            {
                // Cannot fail if the joint exists
                getJointPositionIdx(robot->pncModel_, jointName, jointPositionIdx);
                getJointVelocityIdx(robot->pncModel_, jointName, jointVelocityIdx);

                jointPositionIndices_.insert(jointPositionIndices_.end(), jointPositionIdx.begin(), jointPositionIdx.end());
                jointVelocityIndices_.insert(jointVelocityIndices_.end(), jointVelocityIdx.begin(), jointVelocityIdx.end());
            }
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

    hresult_t AbstractTransmissionBase::reset(void)
    {
        // Make sure the motor is attached to a robot
        if (!isAttached_)
        {
            PRINT_ERROR("Motor not attached to any robot.");
            return hresult_t::ERROR_GENERIC;
        }

        // Make sure the robot still exists
        if (robot_.expired())
        {
            PRINT_ERROR("Robot has been deleted. Impossible to reset the motors.");
            return hresult_t::ERROR_GENERIC;
        }

        forwardJacobian_.setZero();
        backwardJacobian_.setZero();

        // // Update transmission scope information
        refreshProxies();

        return hresult_t::SUCCESS;
    }


}
