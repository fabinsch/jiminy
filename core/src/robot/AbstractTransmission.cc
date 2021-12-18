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
    jointModelIndices_(-1),
    jointTypes_(),
    jointPositionIndices_(-1),
    jointVelocityIndices_(-1),
    motorNames_(),
    motorIndices_(),
    motors_(),
    forwardTransform_(),
    backwardTransform_()
    {
        // Empty.
    }

    AbstractTransmissionBase::~AbstractTransmissionBase(void)
    {
        // Empty.
    }
}
