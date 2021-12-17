///////////////////////////////////////////////////////////////////////////////////////////////
///
/// \brief      Generic interface for any transmission.
///
/// \details    Any transmission must inherit from this base class and implement its virtual methods.
///
/// \remark     Each transmission added to a Jiminy Robot is downcasted as an instance of
///             AbstractTransmissionBase and polymorphism is used to call the actual implementations.
///
///////////////////////////////////////////////////////////////////////////////////////////////

#ifndef JIMINY_ABSTRACT_TRANSMISSION_H
#define JIMINY_ABSTRACT_TRANSMISSION_H

#include <memory>

#include "jiminy/core/Macros.h"
#include "jiminy/core/Types.h"
#include "jiminy/core/robot/AbstractMotor.h"


namespace jiminy
{
    class Robot;

    class AbstractTransmissionBase : public std::enable_shared_from_this<AbstractTransmissionBase>
    {
        /* AKA AbstractSensorBase */
        friend Robot;

    public:
        ///////////////////////////////////////////////////////////////////////////////////////////////
        /// \brief      Dictionary gathering the configuration options shared between transmissions
        ///////////////////////////////////////////////////////////////////////////////////////////////
        virtual configHolder_t getDefaultTransmissionOptions(void)
        {
            configHolder_t config;

            // TODO this here should not be just a float, can be a function
            config["mechanicalReduction"] = 0.0;
            return config;
        };

        struct abstractTransmissionOptions_t
        {
            float64_t const mechanicalReduction;

            abstractTransmissionOptions_t(configHolder_t const & options) :
            // TODO modify type, more generic options
            mechanicalReduction(boost::get<float64_t>(options.at("mechanicalReduction")))
            {
                // Empty.
            }
        };

    public:
        ///////////////////////////////////////////////////////////////////////////////////////////////
        /// \brief      Forbid the copy of the class
        ///////////////////////////////////////////////////////////////////////////////////////////////
        AbstractTransmissionBase(AbstractTransmissionBase const & abstractTransmission) = delete;
        AbstractTransmissionBase & operator = (AbstractTransmissionBase const & other) = delete;

        auto shared_from_this() { return shared_from(this); }
        auto shared_from_this() const { return shared_from(this); }

        ///////////////////////////////////////////////////////////////////////////////////////////////
        /// \brief      Constructor
        ///
        /// \param[in]  robot   Robot
        /// \param[in]  name    Name of the transmission
        ///////////////////////////////////////////////////////////////////////////////////////////////
        AbstractTransmissionBase(std::string const & name);
        virtual ~AbstractTransmissionBase(void);

    public:
        std::unique_ptr<abstractTransmissionOptions_t const> baseTransmissionOptions_;  ///< Structure with the parameters of the transmission

    protected:
        configHolder_t transmissionOptionsHolder_;                   ///< Dictionary with the parameters of the transmission
        bool_t isInitialized_;                                       ///< Flag to determine whether the transmission has been initialized or not
        bool_t isAttached_;                                          ///< Flag to determine whether the transmission is attached to a robot
        std::weak_ptr<Robot const> robot_;                           ///< Robot for which the command and internal dynamics
        std::string name_;                                           ///< Name of the transmission
        int32_t transmissionIdx_;                                    ///< Index of the transmission in the transmission buffer
        std::vector<std::string> jointNames_;                        ///< Names of the connected joints
        std::vector<jointIndex_t> jointModelIndices_;                ///< Indices of the connected joints in the full model
        std::vector<joint_t> jointTypes_;                            ///< Types of the connected joints
        vectorN_t jointPositionIndices_;                             ///< Indices of the joint position
        vectorN_t jointVelocityIndices_;                             ///< Indices of the joint velocities
        std::vector<std::string> motorNames_;                        ///< Names of the connected motors
        std::vector<int32_t> motorIndices_;                          ///< Indices of the connected motors in the buffer
        std::vector<std::weak_ptr<AbstractMotorBase> > motors_;      ///< Buffer holding all connected motors
        matrixN_t forwardTransform_;                                 ///< Transformation matrix from motor to joint quantities
        matrixN_t backwardTransform_;                                ///< Transformation matrix from joint to motor quantities
    };
}

#endif //end of JIMINY_ABSTRACT_TRANSMISSION_H