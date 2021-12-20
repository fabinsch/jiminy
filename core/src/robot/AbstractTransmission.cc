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
        for (std::string const & transmissionJoint : getJointNames())
        {
            auto transmissionJointIt = std::find(actuatedJointNames.begin(), actuatedJointNames.end(), transmissionJoint);
            if (transmissionJointIt != actuatedJointNames.end())
            {
                PRINT_ERROR("Joint already attached to another transmission");
                return hresult_t::ERROR_GENERIC;
            }

            // Update list of actuated joints
            robot->addActuatedJointName(transmissionJoint)
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

        // TODO Delete motor and joint references

        // Update the flag
        isAttached_ = false;

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

        for (unsigned int i = 0; i < jointNames_.size(); i++)
        {
            if (returnCode == hresult_t::SUCCESS)
            {
                // jointModelIndices_ is empty at this point, therefore inject a 0 and later get the right index
                jointModelIndices_.push_back(0);
                returnCode = ::jiminy::getJointModelIdx(robot->pncModel_, jointNames_[i], jointModelIndices_[i]);
            }
        }


        for (unsigned int i = 0; i < jointNames_.size(); i++)
        {
            joint_t tmp;
            if (returnCode == hresult_t::SUCCESS)
            {
                // jointTypes_ is empty at this point, therefore inject a 0 and later get the right type
                jointTypes_.push_back(tmp);
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
                auto motorTemp = motor.lock();
                std::size_t idx = (motorTemp->getIdx());
                motorIndices_.push_back(idx);
            }
        }

        // PopulatjointPositionIndices_
        std::vector<int32_t> jointPositionIndices;
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
                jointPositionIndices.insert(jointPositionIndices.end(), jointPositionIdx.begin(), jointPositionIdx.end());
            }
        }
        int32_t jointPositionSize = static_cast<int32_t>(jointPositionIndices.size());
        jointPositionIndices_.resize(jointPositionSize);
        for (int32_t i = 0; i <  jointPositionSize; ++i)
        {
            jointPositionIndices_[i] = jointPositionIndices[i];
        }

        // Populate jointVelocityIndices_
        std::vector<int32_t> jointVelocityIndices;
        for (std::string const & jointName : jointNames_)
        {
            std::vector<int32_t> jointVelocityIdx;
            if (!robot->pncModel_.existJointName(jointName))
            {
                PRINT_ERROR("Joint '", jointName, "' not found in robot model.");
                return hresult_t::ERROR_BAD_INPUT;
            }
            jointIndex_t const & jointModelIdx = robot->pncModel_.getJointId(jointName);
            int32_t const & jointVelocityFirstIdx = robot->pncModel_.joints[jointModelIdx].idx_v();
            int32_t const & jointNv = robot->pncModel_.joints[jointModelIdx].nv();
            jointVelocityIdx.resize(jointNv);
            std::iota(jointVelocityIdx.begin(), jointVelocityIdx.end(), jointVelocityFirstIdx);
            jointVelocityIndices.insert(jointVelocityIndices.end(), jointVelocityIdx.begin(), jointVelocityIdx.end());
        }
        int32_t jointVelocitySize = static_cast<int32_t>(jointVelocityIndices.size());
        jointVelocityIndices_.resize(jointVelocitySize);
        for (int32_t i = 0; i <  jointVelocitySize; ++i)
        {
            jointVelocityIndices_[i] = jointVelocityIndices[i];
        }

        // TODO: is this still necessary here ?
        // for (unsigned int i = 0; i < jointNames_.size(); i++)
        // {
        //     if (returnCode == hresult_t::SUCCESS)
        //     {
        //         ::jiminy::getJointPositionIdx(robot->pncModel_, jointNames_[i], jointPositionIndices_[i]);
        //         ::jiminy::getJointVelocityIdx(robot->pncModel_, jointNames_[i], jointVelocityIndices_[i]);
        //     }
        // }

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
        // Extract motor configuration and velocity from all motors attached
        // to the robot for this transmission

        // ujoint is effort on the level of the joint, not transmission
        // if you know torque at motor level, use transmission to compute torque at joint level
        // you know state of the system (joints), then compute the state of the motors

        // Gather the corresponding data of the motors attached to the transmission
        // int32_t numberOfMotors = motors_.size();
        // vectorN_t qMotor(numberOfMotors);
        // vectorN_t vMotor(numberOfMotors);
        // vectorN_t aMotor(numberOfMotors);
        // vectorN_t uMotor(numberOfMotors);

        // for (int32_t i = 0; i < numberOfMotors; i++)
        // {
        //     auto motorTemp = motors_[i].lock();
        //     qMotor[i] = motorTemp->getPosition();
        //     vMotor[i] = motorTemp->getVelocity();
        //     aMotor[i] = motorTemp->getAcceleration();
        //     uMotor[i] = motorTemp->getEffort();
        // }
        // // Compute the transmission effect based on the current configuration
        // computeTransform(qMotor, vMotor, forwardTransform_);

        // // Apply transformation from motor to joint level
        // // TODO take care of motor position which can be 1 or 2 dimensional
        // q.noalias() = forwardTransform_ * qMotor;
        // v.noalias() = forwardTransform_ * vMotor;
        // a.noalias() = forwardTransform_ * aMotor;
        // uJoint.noalias() = forwardTransform_ * uMotor;
        return hresult_t::SUCCESS;

    }

    hresult_t AbstractTransmissionBase::computeBackward(float64_t const & /*t*/,
                                                        vectorN_t const & /*q*/,
                                                        vectorN_t const & /*v*/,
                                                        vectorN_t const & /*a*/,
                                                        vectorN_t const & /*uJoint*/)
    {
        // Extract motor configuration and velocity from all motors attached
        // to the robot for this transmission
        // auto qMotors = q.segment(jointPositionIndices_);
        // auto vMotors = v.segment(jointVelocityIndices_);

        // // Compute the transmission effect based on the current configuration
        // computeInverseTransform(qMotors, vMotors, backwardTransform_);

        // // Gather the corresponding data of the motors attached to the transmission
        // int32_t numberOfMotors = motors_.size();
        // vectorN_t qTemp(numberOfMotors);
        // vectorN_t vTemp(numberOfMotors);
        // vectorN_t aTemp(numberOfMotors);
        // vectorN_t uTemp(numberOfMotors);

        // // TODO similar way than forward
        // auto motors = motors_.lock();
        // motors->q = backwardTransform_ * q;
        // motors->v = backwardTransform_ * v;
        // motors->a = backwardTransform_ * a;
        // motors->u = backwardTransform_ * uJoint;

        return hresult_t::SUCCESS;
    }

}
