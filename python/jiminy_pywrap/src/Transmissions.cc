#include "jiminy/core/robot/BasicTransmissions.h"

#include <boost/python.hpp>

#include "jiminy/python/Utilities.h"
#include "jiminy/python/Transmissions.h"


namespace jiminy
{
namespace python
{
    namespace bp = boost::python;

    // ***************************** PyAbstractTransmissionVisitor ***********************************

    struct PyAbstractTransmissionVisitor
        : public bp::def_visitor<PyAbstractTransmissionVisitor>
    {
    public:
        ///////////////////////////////////////////////////////////////////////////////
        /// \brief Expose C++ API through the visitor.
        ///////////////////////////////////////////////////////////////////////////////

        template<class PyClass>
        void visit(PyClass & cl) const
        {
            cl
                .add_property("is_initialized", bp::make_function(&AbstractInvertibleTransmissionBase::getIsInitialized,
                                                bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("name", bp::make_function(&AbstractInvertibleTransmissionBase::getName,
                                        bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("idx", bp::make_function(&AbstractInvertibleTransmissionBase::getIdx,
                                        bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("joint_names", bp::make_function(&AbstractInvertibleTransmissionBase::getJointNames,
                                             bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("joint_indices", bp::make_function(&AbstractInvertibleTransmissionBase::getJointModelIndices,
                                               bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("joint_types", bp::make_function(&AbstractInvertibleTransmissionBase::getJointTypes,
                                             bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("joint_position_indices", bp::make_function(&AbstractInvertibleTransmissionBase::getJointPositionIndices,
                                                        bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("joint_velocity_indices", bp::make_function(&AbstractInvertibleTransmissionBase::getJointVelocityIndices,
                                                        bp::return_value_policy<bp::copy_const_reference>()))
                .add_property("jacobian", bp::make_function(&AbstractInvertibleTransmissionBase::getJacobian,
                                                   bp::return_value_policy<result_converter<false> >()))
                .add_property("inverse_jacobian", bp::make_function(&AbstractInvertibleTransmissionBase::getInverseJacobian,
                                                   bp::return_value_policy<result_converter<false> >()))

                .def("set_options", &PyAbstractTransmissionVisitor::setOptions)
                .def("get_options", &AbstractInvertibleTransmissionBase::getOptions)
                .def("compute_transform", &AbstractInvertibleTransmissionBase::computeTransform)
                .def("compute_inverse_transform", &AbstractInvertibleTransmissionBase::computeInverseTransform)
                ;
        }

    public:
        static void setOptions(AbstractInvertibleTransmissionBase       & self,
                               bp::dict          const & configPy)
        {
            configHolder_t config = self.getOptions();
            convertFromPython(configPy, config);
            self.setOptions(config);
        }

        ///////////////////////////////////////////////////////////////////////////////
        /// \brief Expose.
        ///////////////////////////////////////////////////////////////////////////////
        static void expose()
        {
            bp::class_<AbstractInvertibleTransmissionBase,
                       std::shared_ptr<AbstractInvertibleTransmissionBase>,
                       boost::noncopyable>("AbstractTransmission", bp::no_init)
                .def(PyAbstractTransmissionVisitor());
        }
    };

    BOOST_PYTHON_VISITOR_EXPOSE(AbstractTransmission)

    // ***************************** PySimpleTransmissionVisitor ***********************************

    struct PySimpleTransmissionVisitor
        : public bp::def_visitor<PySimpleTransmissionVisitor>
    {
    public:
        ///////////////////////////////////////////////////////////////////////////////
        /// \brief Expose C++ API through the visitor.
        ///////////////////////////////////////////////////////////////////////////////

        template<class PyClass>
        class PyTransmissionVisitorImpl
        {
        public:
            using TTransmission = typename PyClass::wrapped_type;

            static void visit(PyClass & cl)
            {
                cl
                    .def("initialize", &TTransmission::initialize)
                    ;
            }
        };

    public:
        template<class PyClass>
        void visit(PyClass & cl) const
        {
            PyTransmissionVisitorImpl<PyClass>::visit(cl);
        }

        ///////////////////////////////////////////////////////////////////////////////
        /// \brief Expose.
        ///////////////////////////////////////////////////////////////////////////////
        static void expose()
        {
            bp::class_<SimpleTransmission, bp::bases<AbstractInvertibleTransmissionBase>,
                       std::shared_ptr<SimpleTransmission>,
                       boost::noncopyable>("SimpleTransmission",
                       bp::init<std::string const &>(
                       bp::args("self", "transmission_name")))
                .def(PySimpleTransmissionVisitor());
        }
    };

    BOOST_PYTHON_VISITOR_EXPOSE(SimpleTransmission)
}  // End of namespace python.
}  // End of namespace jiminy.