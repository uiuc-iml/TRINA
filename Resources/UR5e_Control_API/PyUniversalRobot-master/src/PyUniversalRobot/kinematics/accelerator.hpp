#pragma once
#ifndef ACCELERATOR_HPP
#define ACCELERATOR_HPP

class UniversalRobotsAccelerator {
public:
    UniversalRobotsAccelerator(double d1, double a2, double a3, double d4, double d5, double d6);
    virtual ~UniversalRobotsAccelerator();

    size_t dimension() const;
    size_t max_results() const;

    void limits(double* q_min, double* q_max) const;

    void forward(const double* q, double* T) const;
    void forward_all(const double* q, double* const* Ts) const;
    size_t inverse(const double* T, double* const* qs, const double* q_hint) const;

    void jacobian(const double* q, const double* p, double* J) const;
    void hessian(const double* q, const double* p, const double* qd, double* H) const;
    void gradient(const double* q, double* G) const;

private:
    const double d1, a2, a3, d4, d5, d6;

    void forward_all(const double* q, double* T1, double* T2, double* T3, double* T4, double* T5, double* T6) const;

    size_t inverse(const double* T6, double* q_sols, double q6_des) const;

    const size_t MaxInverseResults = 8;
    const size_t JointCount = 6;
};

#endif