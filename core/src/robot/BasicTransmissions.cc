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

    hresult_t SimpleTransmission::setOptions(configHolder_t const & /*transmissionOptions*/)
    {
        return hresult_t::SUCCESS;
    }

    void SimpleTransmission::computeJacobian(vectorN_t const & /*q*/,
                                             matrixN_t & /*Jac*/)
    {
        // TODO
    }

    void SimpleTransmission::computeInverseJacobian(vectorN_t const & /*q*/,
                                                    matrixN_t & /*Jac*/)
    {
        // TODO
    }

    void SimpleTransmission::computeTransform(vectorN_t const & /*qMotors*/,
                                              vectorN_t       & /*qJoints*/)
    {
        // TODO
    }
    void SimpleTransmission::computeInverseTransform(vectorN_t const & /*qJoints*/,
                                                     vectorN_t       & /*qMotors*/)
    {
        // TODO
    }
    void SimpleTransmission::computeEffortTransmission(void)
    {
        // TODO
    }
}
