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

    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     /// \brief    Initialize
    //     ///
    //     /// \details  Initialize the transmission with the names of connected motors and actuated joints.
    //     ///
    //     /// \param[in]  jointNames    Names of the joints connected to the transmission
    //     /// \param[in]  motorNames    Names of the motors connected to the transmission
    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     virtual hresult_t initialize(std::vector<std::string> const & jointNames,
    //                                  std::vector<std::string> const & motorNames);

    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     /// \brief    Refresh the proxies.
    //     ///
    //     /// \remark   This method is not intended to be called manually. The Robot to which the
    //     ///           transmission is added is taking care of it when its own `refresh` method is called.
    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     virtual hresult_t refreshProxies(void);

    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     /// \brief    Reset the internal state of the transmissions.
    //     ///
    //     /// \details  This method resets the internal state of the transmission.
    //     ///
    //     /// \remark   This method is not intended to be called manually. The Robot to which the
    //     ///           transmission is added is taking care of it when its own `reset` method is called.
    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     virtual hresult_t resetAll(void);

    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     /// \brief      Get the configuration options of the transmission.
    //     ///
    //     /// \return     Dictionary with the parameters of the transmission
    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     configHolder_t getOptions(void) const;

    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     /// \brief      Set the configuration options of the transmission.
    //     ///
    //     /// \param[in]  transmissionOptions   Dictionary with the parameters of the transmission
    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     virtual hresult_t setOptions(configHolder_t const & transmissionOptions);

    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     /// \brief      Get isInitialized_.
    //     ///
    //     /// \details    It is a flag used to determine if the transmission has been initialized.
    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     bool_t const & getIsInitialized(void) const;

    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     /// \brief      Get name_.
    //     ///
    //     /// \details    It is the name of the transmission.
    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     std::string const & getName(void) const;

    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     /// \brief      Get transmissionIdx_.
    //     ///
    //     /// \details    It is the index of the transmission.
    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     int32_t const & getIdx(void) const;

    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     /// \brief      Get jointNames_.
    //     ///
    //     /// \details    These are the names of the joints associated with the transmission.
    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     std::vector<std::string> const & getJointNames(void) const;

    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     /// \brief      Get jointModelIndices_.
    //     ///
    //     /// \details    These are the indices of the joints associated with the transmission in the kinematic tree.
    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     std::vector<jointIndex_t >const & getJointModelIndices(void) const;

    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     /// \brief      Get jointTypes_.
    //     ///
    //     /// \details    It is the type of joints associated with the transmission.
    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     std::vector<joint_t> const & getJointTypes(void) const;

    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     /// \brief      Get jointPositionIndices_.
    //     ///
    //     /// \details    It is the index of the joints associated with the transmission in the configuration vector.
    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     vectorN_t const & getJointPositionIndices(void) const;

    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     /// \brief      Get jointVelocityIndices_.
    //     ///
    //     /// \details    It is the index of the joints associated with the transmission in the velocity vector.
    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     vectorN_t const & getJointVelocityIndices(void) const;

    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     /// \brief      Get motorNames_.
    //     ///
    //     /// \details    It is the name of the motors associated with the transmission.
    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     std::vector<std::string> const & getMotorNames(void) const;

    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     /// \brief      Get getMotorIndices.
    //     ///
    //     /// \details    It is the name of the motors associated with the transmission.
    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     std::vector<int32_t> const & getMotorIndices(void) const;

    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     /// \brief      Get forwardTransform_.
    //     ///
    //     /// \details    It is the transformation matrix to convert quantities from motor to joint-level
    //     ///             based on the configuration of the motors.
    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     matrixN_t const & getForwardTransform(void) const;

    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     /// \brief      Get backwardTransform_.
    //     ///
    //     /// \details    It is the transformation matrix to convert quantities from joint to motor-level
    //     ///             based on the configuration of the joints.
    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     matrixN_t const & getInverseTransform(void) const;

    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     /// \brief      Compute forward transmission.
    //     ///
    //     /// \details    Compute forward transmission from motor to joint level. This method updates the
    //     ///             position q, velocity v ,acceleration a and uJoint of the corresponding joints
    //     ///             attached to the transmission.
    //     ///
    //     /// \param[in]  t         Time
    //     /// \param[in]  q         Vector containing position of all joints in the model
    //     /// \param[in]  v         Vector containing velocity of all joints in the model
    //     /// \param[in]  a         Vector containing acceleration of all joints in the model
    //     /// \param[in]  uJoint    Vector containing torque of all joints in the model
    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     hresult_t computeForward(float64_t const & t,
    //                              vectorN_t & q,
    //                              vectorN_t & v,
    //                              vectorN_t & a,
    //                              vectorN_t & uJoint);

    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     /// \brief      Compute backward transmission.
    //     ///
    //     /// \details    Compute backward transmission from joint to motor. This method updates the
    //     ///             position, velocity ,acceleration and effort of the corresponding motors
    //     ///             attached to the transmission.
    //     ///
    //     /// \param[in]  t         Time
    //     /// \param[in]  q         Vector containing position of all joints in the model
    //     /// \param[in]  v         Vector containing velocity of all joints in the model
    //     /// \param[in]  a         Vector containing acceleration of all joints in the model
    //     /// \param[in]  uJoint    Vector containing torque of all joints in the model
    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     hresult_t computeBackward(float64_t const & t,
    //                               vectorN_t const & q,
    //                               vectorN_t const & v,
    //                               vectorN_t const & a,
    //                               vectorN_t const & uJoint);

    // protected:
    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     /// \brief      Request the transmission to update its actual state based of the input data.
    //     ///
    //     /// \details    It assumes that the internal state of the robot is consistent with the
    //     ///             input arguments. This transformation is used to compute the forward transformation
    //     ///             of the transmission transforming motor position, velocity and torque to joint
    //     ///             position, velocity and torque.
    //     ///
    //     /// \param[in]  q        Current position of the motors.
    //     /// \param[in]  v        Current velocity of the motors.
    //     /// \param[out] out      Matrix transforming quantities from motor to joint level
    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     virtual void computeTransform(Eigen::VectorBlock<vectorN_t const> const & q,
    //                                   Eigen::VectorBlock<vectorN_t const> const & v,
    //                                   matrixN_t & out) = 0;

    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     /// \brief      Request the transmission to update its actual state based of the input data.
    //     ///
    //     /// \details    It assumes that the internal state of the robot is consistent with the
    //     ///             input arguments.
    //     ///
    //     /// \param[in]  q        Current position of the joints.
    //     /// \param[in]  v        Current velocity of the joints.
    //     /// \param[out] out      Matrix transforming quantities from joint to motor level
    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     virtual void computeInverseTransform(Eigen::VectorBlock<vectorN_t const> const & q,
    //                                          Eigen::VectorBlock<vectorN_t const> const & v,
    //                                          matrixN_t & out) = 0;

    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     /// \brief      Compute energy dissipation in the transmission.
    //     ///
    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     virtual void computeEffortTransmission(void) = 0;

    // private:
    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     /// \brief    Attach the transmission to a robot
    //     ///
    //     /// \details  This method must be called before initializing the transmission.
    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     hresult_t attach(std::weak_ptr<Robot const> robot);

    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     /// \brief    Detach the transmission from the robot
    //     ///////////////////////////////////////////////////////////////////////////////////////////////
    //     hresult_t detach(void);

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