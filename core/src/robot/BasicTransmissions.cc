#include <algorithm>

#include "jiminy/core/utilities/Helpers.h"

#include "jiminy/core/robot/BasicTransmissions.h"


namespace jiminy
{
    SimpleTransmission::SimpleTransmission(std::string const & name) :
    AbstractTransmissionBase(name)
    {
        std::cout << "construct simple transmission" << std::endl;
    }

    hresult_t SimpleTransmission::initialize(std::vector<std::string> const & jointNames,
                                             std::vector<std::string> const & motorNames)
    {
        if (jointNames.size() != 1 || motorNames.size() != 1)
        {
            PRINT_ERROR("A simple transmission only supports 1-to-1 mappings of motors and joints.");
        }
        hresult_t returnCode = hresult_t::SUCCESS;
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
        returnCode = AbstractTransmissionBase::initialize(jointNames, motorNames);

        return returnCode;
    }

    hresult_t SimpleTransmission::setOptions(configHolder_t const & /*transmissionOptions*/)
    {
        return hresult_t::SUCCESS;
    }

    void SimpleTransmission::computeJacobian(vectorN_t const & /*q*/,
                                             matrixN_t & /*J*/)
    {
        // TODO
    }

    void SimpleTransmission::computeInverseJacobian(std::vector<vectorN_t> const & /*qJoints*/,
                                                    matrixN_t & /*Jinv*/)
    {
        // TODO
    }

    void SimpleTransmission::computeTransform(vectorN_t const & /*qMotors*/,
                                              std::vector<vectorN_t> & /*qJoints*/)
    {
        // TODO
    }
    void SimpleTransmission::computeInverseTransform(std::vector<vectorN_t> const & /*qJoints*/,
                                                     vectorN_t & /*qMotors*/)
    {
        // TODO
    }
    void SimpleTransmission::computeEffortTransmission(void)
    {
        // TODO
    }
}
