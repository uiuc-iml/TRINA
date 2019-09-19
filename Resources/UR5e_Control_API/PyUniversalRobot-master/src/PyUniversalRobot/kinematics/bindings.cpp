#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <kinematics.hpp>

namespace py = pybind11;

PYBIND11_MODULE(kinematics, m) {

    py::class_<Kinematics>(m, "Kinematics")
        .def(py::init<std::string, double, double, double, double, double, double>())

        .def_property_readonly("name", &Kinematics::name)
        .def_property_readonly("dimension", &Kinematics::dimension)

        .def("limits", (const std::pair<Eigen::VectorXd, Eigen::VectorXd>&(Kinematics::*)() const)&Kinematics::limits)
        .def("limits", (std::pair<double, double>(Kinematics::*)(size_t) const)&Kinematics::limits)

        .def("forward", (Eigen::Matrix4d(Kinematics::*)(const Eigen::VectorXd&) const)&Kinematics::forward)
        .def("forward", (Eigen::Matrix4d(Kinematics::*)(const Eigen::VectorXd&, size_t) const)&Kinematics::forward)
        .def("forward_all", &Kinematics::forward_all)

        .def("inverse", (std::vector<Eigen::VectorXd>(Kinematics::*)(const Eigen::Matrix4d&) const)&Kinematics::inverse)
        .def("inverse", (std::vector<Eigen::VectorXd>(Kinematics::*)(const Eigen::Matrix4d&, const Eigen::VectorXd&) const)&Kinematics::inverse)

        .def("inverse_nearest", &Kinematics::inverse_nearest)

        .def("jacobian", (Eigen::MatrixXd(Kinematics::*)(const Eigen::VectorXd&) const)&Kinematics::jacobian)
        .def("jacobian", (Eigen::MatrixXd(Kinematics::*)(const Eigen::VectorXd&, const Eigen::Vector3d&) const)&Kinematics::jacobian)

        .def("hessian", (Eigen::MatrixXd(Kinematics::*)(const Eigen::VectorXd&, const Eigen::VectorXd&) const)&Kinematics::hessian)
        .def("hessian", (Eigen::MatrixXd(Kinematics::*)(const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::Vector3d&) const)&Kinematics::hessian)

        .def("gradient", &Kinematics::gradient)
    ;

    m.attr("UR3") = Kinematics("UR3", 0.1519, -0.24365, -0.21325, 0.11235, 0.08535, 0.0819);
    m.attr("UR5") = Kinematics("UR5", 0.089159, -0.42500, -0.39225, 0.10915, 0.09465, 0.0823);
    m.attr("UR10") = Kinematics("UR10", 0.1273, -0.612, -0.5723, 0.163941, 0.1157, 0.0922);

#ifdef VERSION_INFO
    m.attr("__version__") = VERSION_INFO;
#else
    m.attr("__version__") = "dev";
#endif
}
