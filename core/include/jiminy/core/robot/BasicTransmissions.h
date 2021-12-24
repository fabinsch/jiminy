#ifndef JIMINY_BASIC_TRANSMISSIONS_H
#define JIMINY_BASIC_TRANSMISSIONS_H

#include "jiminy/core/robot/AbstractTransmission.h"


namespace jiminy
{
    class SimpleTransmission : public AbstractTransmissionBase
    {
    public:
        ///////////////////////////////////////////////////////////////////////////////////////////////
        /// \brief      Dictionary gathering the configuration options shared between transmissions
        ///////////////////////////////////////////////////////////////////////////////////////////////
        virtual configHolder_t getDefaultTransmissionOptions(void) override
        {
            // Add extra options or update default values
            configHolder_t config = AbstractTransmissionBase::getDefaultTransmissionOptions();
            config["mechanicalReduction"] = 0.0;

            return config;
        };

        struct transmissionOptions_t : public abstractTransmissionOptions_t
        {
            float64_t const mechanicalReduction;    ///< Gear reduction ratio motor to joint

            transmissionOptions_t(configHolder_t const & options) :
            abstractTransmissionOptions_t(options),
            mechanicalReduction(boost::get<float64_t>(options.at("mechanicalReduction")))
            {
                // Empty.
            }
        };

    public:
        SimpleTransmission(std::string const & name);
        virtual ~SimpleTransmission(void) = default;

        auto shared_from_this() { return shared_from(this); }
        auto shared_from_this() const { return shared_from(this); }

        virtual hresult_t initialize(std::vector<std::string> const & jointNames,
                                     std::vector<std::string> const & motorNames);

        virtual hresult_t setOptions(configHolder_t const & transmissionOptions) final override;

        virtual void computeJacobian(vectorN_t const & qMotors,
                                     matrixN_t & J) final override;

        virtual void computeInverseJacobian(std::vector<vectorN_t> const & qJoints,
                                            matrixN_t & Jinv) final override;
    private:
        virtual void computeTransform(vectorN_t const & qMotors,
                                      std::vector<vectorN_t> & qJoints) final override;
        virtual void computeInverseTransform(std::vector<vectorN_t> const & qJoints,
                                             vectorN_t & qMotors) final override;

        virtual void computeEffortTransmission(void) final override;
    };
}

#endif //end of JIMINY_BASIC_TRANSMISSIONS_H