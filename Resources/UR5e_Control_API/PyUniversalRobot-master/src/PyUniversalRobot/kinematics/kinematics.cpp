#include <kinematics.hpp>

#define _USE_MATH_DEFINES
#include <math.h>

Kinematics::Kinematics(std::string name, double d1, double a2, double a3, double d4, double d5, double d6)
    : UniversalRobotsAccelerator(d1, a2, a3, d4, d5, d6) {

    _name = std::move(name);

    _limits.first.resize(dimension());
    _limits.second.resize(dimension());
    UniversalRobotsAccelerator::limits(_limits.first.data(), _limits.second.data());
}

const std::string& Kinematics::name() const {
    return _name;
}

size_t Kinematics::dimension() const {
    return UniversalRobotsAccelerator::dimension();
}

const std::pair<Eigen::VectorXd, Eigen::VectorXd>& Kinematics::limits() const {
    return _limits;
}

std::pair<double, double> Kinematics::limits(size_t idx) const {
    _check_index(idx);
    return { _limits.first[idx], _limits.second[idx] };
}

Eigen::Matrix4d Kinematics::forward(const Eigen::VectorXd& q) const {
    _check_vector(q);

    Eigen::Matrix4d T;
    UniversalRobotsAccelerator::forward(q.data(), T.data());

    // convert from row to column major
    T.transposeInPlace();
    return T;
}

Eigen::Matrix4d Kinematics::forward(const Eigen::VectorXd& q, size_t idx) const {
    _check_vector(q);
    _check_index(idx);

    Eigen::Matrix4d T;

    std::vector<double*> ptr;
    for(size_t i = 0; i < ptr.size(); i++) {
        ptr.push_back(i == idx ? T.data() : nullptr);
    }

    UniversalRobotsAccelerator::forward_all(q.data(), ptr.data());

    // convert from row to column major
    T.transposeInPlace();
    return T;
}

std::vector<Eigen::Matrix4d> Kinematics::forward_all(const Eigen::VectorXd& q) const {
    _check_vector(q);

    std::vector<Eigen::Matrix4d> Ts;
    Ts.resize(dimension());

    std::vector<double*> ptr;
    for(auto& T : Ts) {
        ptr.push_back(T.data());
    }

    UniversalRobotsAccelerator::forward_all(q.data(), ptr.data());

    // convert from row to column major
    for(auto& T : Ts) {
        T.transposeInPlace();
    }

    return Ts;
}

std::vector<Eigen::VectorXd> Kinematics::inverse(const Eigen::Matrix4d& T) const {
    return inverse(T, Eigen::VectorXd::Zero(dimension()));
}
std::vector<Eigen::VectorXd> Kinematics::inverse(const Eigen::Matrix4d& T, const Eigen::VectorXd& q_hint) const {
    _check_vector(q_hint);

    std::vector<Eigen::VectorXd> qs;
    qs.resize(max_results());

    std::vector<double*> ptr;
    for(auto& q : qs) {
        q.resize(dimension());
        ptr.push_back(q.data());
    }

    // convert from column major to row major
    Eigen::Matrix4d T2 = T;
    T2.transposeInPlace();
    size_t n = UniversalRobotsAccelerator::inverse(T2.data(), ptr.data(), q_hint.data());

    qs.resize(n);
    return qs;
}

static long _lfloor(double x) {
    return std::lround(std::floor(x));
}
static double _mod(double x, double d) {
    return x - d * _lfloor(x / d);
}

Eigen::VectorXd Kinematics::inverse_nearest(const Eigen::Matrix4d& T, const Eigen::VectorXd& q_ref) const {
    auto qs = inverse(T, q_ref);
    if(qs.empty()) {
        throw std::runtime_error("no solution");
    }

    // adjust revolute joints as close to reference as possible
    for(auto& q : qs) {
        for(Eigen::Index i = 0; i < q.size(); i++) {
            auto n = _lfloor(_limits.first[i] / 2 / M_PI);
            auto qfrac = _mod(q[i], 2 * M_PI);

            auto dist = std::abs(q[i] - q_ref[i]);
            do {
                auto qq = n * 2 * M_PI + qfrac;
                if(qq > _limits.second[i]) {
                    break;
                }

                if(qq >= _limits.first[i]) {
                    auto d = std::abs(qq - q_ref[i]);
                    if(d < dist) {
                        dist = d;
                        q[i] = qq;
                    }
                }

                n++;
            } while(true);

        }
    }

    // choose closest configuration
    return *std::min_element(qs.begin(), qs.end(), [&](const auto& qa, const auto& qb) {
        return (qa - q_ref).squaredNorm() < (qb - q_ref).squaredNorm();
    });
}

Eigen::MatrixXd Kinematics::jacobian(const Eigen::VectorXd& q) const {
    return jacobian(q, Eigen::Vector3d::Zero());
}
Eigen::MatrixXd Kinematics::jacobian(const Eigen::VectorXd& q, const Eigen::Vector3d& offset) const {
    _check_vector(q);

    // start with transposed array for row order output
    Eigen::MatrixXd J(dimension(), 6);

    UniversalRobotsAccelerator::jacobian(q.data(), offset.data(), J.data());

    // convert from row to column major
    J.transposeInPlace();
    return J;
}

Eigen::MatrixXd Kinematics::hessian(const Eigen::VectorXd& q, const Eigen::VectorXd& qd) const {
    return hessian(q, qd, Eigen::Vector3d::Zero());
}
Eigen::MatrixXd Kinematics::hessian(const Eigen::VectorXd& q, const Eigen::VectorXd& qd, const Eigen::Vector3d& offset) const {
    _check_vector(q);
    _check_vector(qd);

    // start with transposed array for row order output
    Eigen::MatrixXd H(dimension(), 6);

    UniversalRobotsAccelerator::hessian(q.data(), qd.data(), offset.data(), H.data());

    // convert from row to column major
    H.transposeInPlace();
    return H;
}

Eigen::MatrixXd Kinematics::gradient(const Eigen::VectorXd& q) const {
    _check_vector(q);

    // start with transposed array for row order output
    Eigen::MatrixXd G(dimension(), 12);

    UniversalRobotsAccelerator::gradient(q.data(), G.data());

    // convert from row to column major
    G.transposeInPlace();
    return G;
}

void Kinematics::_check_vector(const Eigen::VectorXd& q) const {
    if(q.size() != dimension()) {
        throw std::invalid_argument("vector length != dimension");
    }
    if(q.hasNaN()) {
        throw std::invalid_argument("NaNs in joint vector");
    }
}

void Kinematics::_check_index(size_t idx) const {
    if(idx >= dimension()) {
        throw std::out_of_range("index exceeded dimension");
    }
}
