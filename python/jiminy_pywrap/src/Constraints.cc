#include "jiminy/core/robot/Model.h"
#include "jiminy/core/robot/AbstractConstraint.h"
#include "jiminy/core/robot/JointConstraint.h"
#include "jiminy/core/robot/FixedFrameConstraint.h"
#include "jiminy/core/robot/SphereConstraint.h"
#include "jiminy/core/robot/WheelConstraint.h"

#include "jiminy/python/Functors.h"
#include "jiminy/python/Constraints.h"

#include <boost/python.hpp>


namespace jiminy
{
    // ***************************** PyConstraintVisitor ***********************************

    // Using an intermediary class is a trick to enable defining bp::base<...> in conjunction with bp::wrapper<...>
    class AbstractConstraintImpl: public AbstractConstraintTpl<AbstractConstraintImpl> {};

    // Explicit template specialization must appear in exactly the same namespace than its template declaration
    template<>
    std::string const AbstractConstraintTpl<AbstractConstraintImpl>::type_("UserConstraint");

namespace python
{
    namespace bp = boost::python;

    class AbstractConstraintWrapper: public AbstractConstraintImpl, public bp::wrapper<AbstractConstraintImpl>
    {
    public:
        hresult_t reset(vectorN_t const & q,
                        vectorN_t const & v)
        {
            bp::override func = this->get_override("reset");
            if (func)
            {
                func(FctPyWrapperArgToPython(q),
                     FctPyWrapperArgToPython(v));
            }
            return hresult_t::SUCCESS;
        }

        hresult_t computeJacobianAndDrift(vectorN_t const & q,
                                          vectorN_t const & v)
        {
            bp::override func = this->get_override("compute_jacobian_and_drift");
            if (func)
            {
                func(FctPyWrapperArgToPython(q),
                     FctPyWrapperArgToPython(v));
            }
            return hresult_t::SUCCESS;
        }
    };

    struct PyConstraintVisitor
        : public bp::def_visitor<PyConstraintVisitor>
    {
    public:
        template<class PyClass>
        void visit(PyClass & cl) const
        {
            cl
                .add_property("type", bp::make_function(&AbstractConstraintBase::getType,
                                      bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("is_enabled", bp::make_function(&AbstractConstraintBase::getIsEnabled,
                                            bp::return_value_policy<bp::copy_const_reference>()),
                                            &PyConstraintVisitor::setIsEnable)
                .add_property("baumgarte_freq", &AbstractConstraintBase::getBaumgarteFreq,
                                                &AbstractConstraintBase::setBaumgarteFreq)
                .add_property("jacobian", &PyConstraintVisitor::getJacobian)
                .add_property("drift", &PyConstraintVisitor::getDrift)
                ;
        }

        static std::shared_ptr<FixedFrameConstraint> fixedFrameConstraintFactory(std::string const & frameName,
                                                                                 bool_t const & isTranslationFixed,
                                                                                 bool_t const & isRotationFixed)
        {
            return std::make_shared<FixedFrameConstraint>(frameName, isTranslationFixed, isRotationFixed);
        }

        static void setIsEnable(AbstractConstraintBase & self,
                                bool_t const & value)
        {
            if (value)
            {
                self.enable();
            }
            else
            {
                self.disable();
            }
        }

        static bp::object getJacobian(AbstractConstraintBase const & self)
        {
            // Do not use automatic converter for efficiency
            return convertToPython<matrixN_t const>(self.getJacobian(), false);
        }

        static bp::object getDrift(AbstractConstraintBase const & self)
        {
            // Do not use automatic converter for efficiency
            return convertToPython<vectorN_t const>(self.getDrift(), false);
        }

        static bp::object getReferenceConfiguration(JointConstraint & self)
        {
            // Do not use automatic converter for efficiency
            return convertToPython<vectorN_t>(self.getReferenceConfiguration(), false);
        }

        static void setReferenceConfiguration(JointConstraint & self, vectorN_t const & value)
        {
            self.setReferenceConfiguration(value);
        }

    public:
        ///////////////////////////////////////////////////////////////////////////////
        /// \brief Expose.
        ///////////////////////////////////////////////////////////////////////////////
        static void expose()
        {
            bp::class_<AbstractConstraintBase,
                       std::shared_ptr<AbstractConstraintBase>,
                       boost::noncopyable>("AbstractConstraintBase", bp::no_init)
                .def(PyConstraintVisitor())
                .def("reset", &AbstractConstraintBase::reset,
                              (bp::arg("self"), "q", "v"))
                .def("compute_jacobian_and_drift", &AbstractConstraintBase::computeJacobianAndDrift,
                                                   (bp::arg("self"), "q", "v"));

            bp::class_<AbstractConstraintWrapper, bp::bases<AbstractConstraintBase>,
                       std::shared_ptr<AbstractConstraintWrapper>,
                       boost::noncopyable>("BaseConstraint")
                .def_readonly("type", &AbstractConstraintWrapper::type_)
                .def("reset", bp::pure_virtual(&AbstractConstraintBase::reset))
                .def("compute_jacobian_and_drift", bp::pure_virtual(&AbstractConstraintBase::computeJacobianAndDrift));

            bp::class_<JointConstraint, bp::bases<AbstractConstraintBase>,
                       std::shared_ptr<JointConstraint>,
                       boost::noncopyable>("JointConstraint", bp::init<std::string>())
                .def_readonly("type", &JointConstraint::type_)
                .add_property("joint_name", bp::make_function(&JointConstraint::getJointName,
                                            bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("joint_idx", bp::make_function(&JointConstraint::getJointIdx,
                                           bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("reference_configuration", &PyConstraintVisitor::getReferenceConfiguration,
                                                         &PyConstraintVisitor::setReferenceConfiguration);

            bp::class_<FixedFrameConstraint, bp::bases<AbstractConstraintBase>,
                       std::shared_ptr<FixedFrameConstraint>,
                       boost::noncopyable>("FixedFrameConstraint", bp::no_init)
                .def("__init__", bp::make_constructor(&PyConstraintVisitor::fixedFrameConstraintFactory,
                                 bp::default_call_policies(), (bp::arg("frame_name"),
                                                               bp::arg("is_translation_fixed")=true,
                                                               bp::arg("is_rotation_fixed")=true)))
                .def_readonly("type", &FixedFrameConstraint::type_)
                .add_property("frame_name", bp::make_function(&FixedFrameConstraint::getFrameName,
                                            bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("frame_idx", bp::make_function(&FixedFrameConstraint::getFrameIdx,
                                           bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("is_translation_fixed", bp::make_function(&FixedFrameConstraint::getIsTranslationFixed,
                                                      bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("is_rotation_fixed", bp::make_function(&FixedFrameConstraint::getIsRotationFixed,
                                                   bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("reference_transform", bp::make_function(&FixedFrameConstraint::getReferenceTransform,
                                                     bp::return_internal_reference<>()),
                                                     &FixedFrameConstraint::setReferenceTransform);

            bp::class_<SphereConstraint, bp::bases<AbstractConstraintBase>,
                       std::shared_ptr<SphereConstraint>,
                       boost::noncopyable>("SphereConstraint", bp::init<std::string, float64_t>())
                .def_readonly("type", &SphereConstraint::type_)
                .add_property("frame_name", bp::make_function(&SphereConstraint::getFrameName,
                                            bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("frame_idx", bp::make_function(&SphereConstraint::getFrameIdx,
                                           bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("reference_transform", bp::make_function(&SphereConstraint::getReferenceTransform,
                                                     bp::return_internal_reference<>()),
                                                     &SphereConstraint::setReferenceTransform);

            bp::class_<WheelConstraint, bp::bases<AbstractConstraintBase>,
                       std::shared_ptr<WheelConstraint>,
                       boost::noncopyable>("WheelConstraint", bp::init<std::string, float64_t, vector3_t, vector3_t>())
                .def_readonly("type", &WheelConstraint::type_)
                .add_property("frame_name", bp::make_function(&WheelConstraint::getFrameName,
                                            bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("frame_idx", bp::make_function(&WheelConstraint::getFrameIdx,
                                           bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("reference_transform", bp::make_function(&WheelConstraint::getReferenceTransform,
                                                     bp::return_internal_reference<>()),
                                                     &WheelConstraint::setReferenceTransform);

        }
    };

    BOOST_PYTHON_VISITOR_EXPOSE(Constraint)

    // ***************************** PyConstraintsHolderVisitor ***********************************


    struct PyConstraintsHolderVisitor
        : public bp::def_visitor<PyConstraintsHolderVisitor>
    {
    public:
        ///////////////////////////////////////////////////////////////////////////////
        /// \brief Expose C++ API through the visitor.
        ///////////////////////////////////////////////////////////////////////////////
        template<class PyClass>
        void visit(PyClass & cl) const
        {
            cl
                .add_property("bounds_joints", &PyConstraintsHolderVisitor::getBoundJoints)
                .add_property("contact_frames", &PyConstraintsHolderVisitor::getContactFrames)
                .add_property("collision_bodies", &PyConstraintsHolderVisitor::getCollisionBodies)
                .add_property("registered", &PyConstraintsHolderVisitor::getRegistered)
                ;
        }

        static bp::dict getBoundJoints(constraintsHolder_t & self)
        {
            bp::dict boundJoints;
            for (auto & constraintItem : self.boundJoints)
            {
                boundJoints[constraintItem.first] = constraintItem.second;
            }
            return boundJoints;
        }

        static bp::dict getContactFrames(constraintsHolder_t & self)
        {
            bp::dict contactFrames;
            for (auto & constraintItem : self.contactFrames)
            {
                contactFrames[constraintItem.first] = constraintItem.second;
            }
            return contactFrames;
        }

        static bp::list getCollisionBodies(constraintsHolder_t & self)
        {
            bp::list collisionBodies;
            for (auto & constraintsMap : self.collisionBodies)
            {
                bp::dict constraintsMapPy;
                for (auto & constraintItem : constraintsMap)
                {
                    constraintsMapPy[constraintItem.first] = constraintItem.second;
                }
                collisionBodies.append(constraintsMapPy);
            }
            return collisionBodies;
        }

        static bp::dict getRegistered(constraintsHolder_t & self)
        {
            bp::dict registered;
            for (auto & constraintItem : self.registered)
            {
                registered[constraintItem.first] = constraintItem.second;
            }
            return registered;
        }

        ///////////////////////////////////////////////////////////////////////////////
        /// \brief Expose.
        ///////////////////////////////////////////////////////////////////////////////
        static void expose()
        {
            bp::class_<constraintsHolder_t,
                       std::shared_ptr<constraintsHolder_t>,
                       boost::noncopyable>("ConstraintsHolder", bp::no_init)
                .def(PyConstraintsHolderVisitor());
        }
    };

    BOOST_PYTHON_VISITOR_EXPOSE(ConstraintsHolder)

}  // End of namespace python.
}  // End of namespace jiminy.