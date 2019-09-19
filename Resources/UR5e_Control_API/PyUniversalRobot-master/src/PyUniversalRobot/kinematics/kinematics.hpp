#pragma once
#ifndef BINDINGS_HPP
#define BINDINGS_HPP

#include <vector>

#include <Eigen/Geometry>

#include <accelerator.hpp>

class Kinematics : protected UniversalRobotsAccelerator {
public:

    Kinematics(std::string name, double d1, double a2, double a3, double d4, double d5, double d6);

    //
    // model
    //

    const std::string& name() const;
    size_t dimension() const;

    //
    // joints
    //

    const std::pair<Eigen::VectorXd, Eigen::VectorXd>& limits() const;
    std::pair<double, double> limits(size_t idx) const;

    //
    // kinematics
    //

    Eigen::Matrix4d forward(const Eigen::VectorXd& q) const;
    Eigen::Matrix4d forward(const Eigen::VectorXd& q, size_t idx) const;
    std::vector<Eigen::Matrix4d> forward_all(const Eigen::VectorXd& q) const;

    std::vector<Eigen::VectorXd> inverse(const Eigen::Matrix4d& T) const;
    std::vector<Eigen::VectorXd> inverse(const Eigen::Matrix4d& T, const Eigen::VectorXd& q_hint) const;

    Eigen::VectorXd inverse_nearest(const Eigen::Matrix4d& T, const Eigen::VectorXd& q_ref) const;

    //
    // dynamics
    //

    Eigen::MatrixXd jacobian(const Eigen::VectorXd& q) const;
    Eigen::MatrixXd jacobian(const Eigen::VectorXd& q, const Eigen::Vector3d& offset) const;

    Eigen::MatrixXd hessian(const Eigen::VectorXd& q, const Eigen::VectorXd& qd) const;
    Eigen::MatrixXd hessian(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::Vector3d& offset) const;

    //
    // gradient
    //

    Eigen::MatrixXd gradient(const Eigen::VectorXd& q) const;

private:

    void _check_vector(const Eigen::VectorXd& q) const;
    void _check_index(size_t idx) const;

    std::string _name;
    std::pair<Eigen::VectorXd, Eigen::VectorXd> _limits;
    
};

#endif