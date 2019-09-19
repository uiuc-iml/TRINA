#include <kinematics.hpp>

#define _USE_MATH_DEFINES
#include <math.h>

#define ZERO_THRESH 0.00000001
#define SIGN(x) ( ( (x) > 0 ) - ( (x) < 0 ) )

#define PI M_PI

UniversalRobotsAccelerator::UniversalRobotsAccelerator(double d1, double a2, double a3, double d4, double d5, double d6)
    : d1(d1), a2(a2), a3(a3), d4(d4), d5(d5), d6(d6) {

}

UniversalRobotsAccelerator::~UniversalRobotsAccelerator() {

}

size_t UniversalRobotsAccelerator::dimension() const {
    return JointCount;
}

size_t UniversalRobotsAccelerator::max_results() const {
    return MaxInverseResults;
}

void UniversalRobotsAccelerator::limits(double* q_min, double* q_max) const {
    for(size_t i = 0; i < JointCount; i++) {
        q_min[i] = -2 * M_PI;
        q_max[i] = 2 * M_PI;
    }
}

void UniversalRobotsAccelerator::forward(const double* q, double* T) const {
    forward_all(q, 0, 0, 0, 0, 0, T);
}

void UniversalRobotsAccelerator::forward_all(const double* q, double* const* Ts) const {
    forward_all(q, Ts[0], Ts[1], Ts[2], Ts[3], Ts[4], Ts[5]);
}

void UniversalRobotsAccelerator::forward_all(const double* q, double* T1, double* T2, double* T3, double* T4, double* T5, double* T6) const {
    double c1 = cos(q[0]), s1 = sin(q[0]);
    double c2 = cos(q[1]), s2 = sin(q[1]);
    double c3 = cos(q[2]), s3 = sin(q[2]);
    double c4 = cos(q[3]), s4 = sin(q[3]);
    double c5 = cos(q[4]), s5 = sin(q[4]);
    double c6 = cos(q[5]), s6 = sin(q[5]);
    double c23 = cos(q[1] + q[2]), s23 = sin(q[1] + q[2]);
    double c234 = cos(q[1] + q[2] + q[3]), s234 = sin(q[1] + q[2] + q[3]);

    if(T1 != NULL) {
         T1[0] = c1;
         T1[1] = 0;
         T1[2] = s1;
         T1[3] = 0;
         T1[4] = s1;
         T1[5] = 0;
         T1[6] = -c1;
         T1[7] = 0;
         T1[8] = 0;
         T1[9] = 1;
         T1[10] = 0;
         T1[11] = d1;
         T1[12] = 0;
         T1[13] = 0;
         T1[14] = 0;
         T1[15] = 1;
    }

    if(T2 != NULL) {
         T2[0] = c1*c2;
         T2[1] = -c1*s2;
         T2[2] = s1;
         T2[3] = a2*c1*c2;
         T2[4] = c2*s1;
         T2[5] = -s1*s2;
         T2[6] = -c1;
         T2[7] = a2*c2*s1;
         T2[8] = s2;
         T2[9] = c2;
         T2[10] = 0;
         T2[11] = a2*s2 + d1;
         T2[12] = 0;
         T2[13] = 0;
         T2[14] = 0;
         T2[15] = 1;
    }

    if(T3 != NULL) {
         T3[0] = c1*c23;
         T3[1] = -c1*s23;
         T3[2] = s1;
         T3[3] = c1*(a2*c2 + a3*c23);
         T3[4] = c23*s1;
         T3[5] = -s1*s23;
         T3[6] = -c1;
         T3[7] = s1*(a2*c2 + a3*c23);
         T3[8] = s23;
         T3[9] = c23;
         T3[10] = 0;
         T3[11] = a2*s2 + a3*s23 + d1;
         T3[12] = 0;
         T3[13] = 0;
         T3[14] = 0;
         T3[15] = 1;
    }

    if(T4 != NULL) {
         T4[0] = c1*c234;
         T4[1] = s1;
         T4[2] = c1*s234;
         T4[3] = a2*c1*c2 + a3*c1*c23 + d4*s1;
         T4[4] = c234*s1;
         T4[5] = -c1;
         T4[6] = s1*s234;
         T4[7] = a2*c2*s1 + a3*c23*s1 - c1*d4;
         T4[8] = s234;
         T4[9] = 0;
         T4[10] = -c234;
         T4[11] = a2*s2 + a3*s23 + d1;
         T4[12] = 0;
         T4[13] = 0;
         T4[14] = 0;
         T4[15] = 1;
    }

    if(T5 != NULL) {
         T5[0] = c1*c234*c5 + s1*s5;
         T5[1] = -c1*s234;
         T5[2] = -c1*c234*s5 + c5*s1;
         T5[3] = a2*c1*c2 + a3*c1*c23 + c1*d5*s234 + d4*s1;
         T5[4] = -c1*s5 + c234*c5*s1;
         T5[5] = -s1*s234;
         T5[6] = -c1*c5 - c234*s1*s5;
         T5[7] = a2*c2*s1 + a3*c23*s1 - c1*d4 + d5*s1*s234;
         T5[8] = c5*s234;
         T5[9] = c234;
         T5[10] = -s234*s5;
         T5[11] = a2*s2 + a3*s23 - c234*d5 + d1;
         T5[12] = 0;
         T5[13] = 0;
         T5[14] = 0;
         T5[15] = 1;
    }

    if(T6 != NULL) {
         T6[0] = -c1*s234*s6 + c6*(c1*c234*c5 + s1*s5);
         T6[1] = -c1*c6*s234 - s6*(c1*c234*c5 + s1*s5);
         T6[2] = -c1*c234*s5 + c5*s1;
         T6[3] = a2*c1*c2 + a3*c1*c23 - c1*c234*d6*s5 + c1*d5*s234 + c5*d6*s1 + d4*s1;
         T6[4] = c6*(-c1*s5 + c234*c5*s1) - s1*s234*s6;
         T6[5] = -c6*s1*s234 + s6*(c1*s5 - c234*c5*s1);
         T6[6] = -c1*c5 - c234*s1*s5;
         T6[7] = a2*c2*s1 + a3*c23*s1 - c1*c5*d6 - c1*d4 - c234*d6*s1*s5 + d5*s1*s234;
         T6[8] = c234*s6 + c5*c6*s234;
         T6[9] = c234*c6 - c5*s234*s6;
         T6[10] = -s234*s5;
         T6[11] = a2*s2 + a3*s23 - c234*d5 + d1 - d6*s234*s5;
         T6[12] = 0;
         T6[13] = 0;
         T6[14] = 0;
         T6[15] = 1;
    }
}

size_t UniversalRobotsAccelerator::inverse(const double* T, double* const* qs, const double* q_hint) const {
    std::vector<double> qs_all(JointCount * MaxInverseResults);
    size_t n = inverse(T, qs_all.data(), q_hint[5]);

    const double* ptr = qs_all.data();
    for(size_t i = 0; i < n; i++) {
        std::copy(ptr, ptr + JointCount, qs[i]);
        ptr += JointCount;
    }

    return n;
}

size_t UniversalRobotsAccelerator::inverse(const double* T6, double* q_sols, double q6_des) const {
    size_t num_sols = 0;

    double T00 = T6[0];
    double T01 = T6[1];
    double T02 = T6[2];
    double T03 = T6[3];
    double T10 = T6[4];
    double T11 = T6[5];
    double T12 = T6[6];
    double T13 = T6[7];
    double T20 = T6[8];
    double T21 = T6[9];
    double T22 = T6[10];
    double T23 = T6[11];

    ////////////////////////////// shoulder rotate joint (q1) //////////////////////////////
    double q1[2];
    {
        double A = d6 * T12 - T13;
        double B = d6 * T02 - T03;
        double R = A * A + B * B;
        if(fabs(A) < ZERO_THRESH) {
            double div;
            if(fabs(fabs(d4) - fabs(B)) < ZERO_THRESH) {
                div = -SIGN(d4) * SIGN(B);
            } else {
                div = -d4 / B;
            }
            double arcsin = asin(div);
            if(fabs(arcsin) < ZERO_THRESH) {
                arcsin = 0.0;
            }
            if(arcsin < 0.0) {
                q1[0] = arcsin + 2.0 * PI;
            } else {
                q1[0] = arcsin;
            }
            q1[1] = PI - arcsin;

        } else if(fabs(B) < ZERO_THRESH) {
            double div;
            if(fabs(fabs(d4) - fabs(A)) < ZERO_THRESH) {
                div = SIGN(d4) * SIGN(A);
            } else {
                div = d4 / A;
            }
            double arccos = acos(div);
            q1[0] = arccos;
            q1[1] = 2.0 * PI - arccos;
        } else if(d4 * d4 > R) {
            return num_sols;
        } else {
            double arccos = acos(d4 / sqrt(R));
            double arctan = atan2(-B, A);
            double pos = arccos + arctan;
            double neg = -arccos + arctan;
            if(fabs(pos) < ZERO_THRESH) {
                pos = 0.0;
            }
            if(fabs(neg) < ZERO_THRESH) {
                neg = 0.0;
            }
            if(pos >= 0.0) {
                q1[0] = pos;
            } else {
                q1[0] = 2.0 * PI + pos;
            }
            if(neg >= 0.0) {
                q1[1] = neg;
            } else {
                q1[1] = 2.0 * PI + neg;
            }
        }
    }
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////// wrist 2 joint (q5) //////////////////////////////
    double q5[2][2];
    {
        for(int i = 0; i < 2; i++) {
            double numer = (T03 * sin(q1[i]) - T13 * cos(q1[i]) - d4);
            double div;
            if(fabs(fabs(numer) - fabs(d6)) < ZERO_THRESH) {
                div = SIGN(numer) * SIGN(d6);
            } else {
                div = numer / d6;
            }
            double arccos = acos(div);
            q5[i][0] = arccos;
            q5[i][1] = 2.0 * PI - arccos;
        }
    }
    ////////////////////////////////////////////////////////////////////////////////
    {
        for(int i = 0; i < 2; i++) {
            for(int j = 0; j < 2; j++) {
                double c1 = cos(q1[i]), s1 = sin(q1[i]);
                double c5 = cos(q5[i][j]), s5 = sin(q5[i][j]);
                double q6;
                ////////////////////////////// wrist 3 joint (q6) //////////////////////////////
                if(fabs(s5) < ZERO_THRESH) {
                    q6 = q6_des;
                } else {
                    q6 = atan2(SIGN(s5) * -(T01 * s1 - T11 * c1),
                        SIGN(s5) * (T00 * s1 - T10 * c1));
                    if(fabs(q6) < ZERO_THRESH) {
                        q6 = 0.0;
                    }
                    if(q6 < 0.0) {
                        q6 += 2.0 * PI;
                    }
                }
                ////////////////////////////////////////////////////////////////////////////////
                double q2[2], q3[2], q4[2];
                ///////////////////////////// RRR joints (q2,q3,q4) ////////////////////////////
                double c6 = cos(q6), s6 = sin(q6);
                double x04x = -s5 * (T02 * c1 + T12 * s1)
                    - c5
                    * (s6 * (T01 * c1 + T11 * s1)
                        - c6 * (T00 * c1 + T10 * s1));
                double x04y = c5 * (T20 * c6 - T21 * s6) - T22 * s5;
                double p13x = d5
                    * (s6 * (T00 * c1 + T10 * s1)
                        + c6 * (T01 * c1 + T11 * s1))
                    - d6 * (T02 * c1 + T12 * s1) + T03 * c1 + T13 * s1;
                double p13y = T23 - d1 - d6 * T22 + d5 * (T21 * c6 + T20 * s6);
                double c3 = (p13x * p13x + p13y * p13y - a2 * a2 - a3 * a3)
                    / (2.0 * a2 * a3);
                if(fabs(fabs(c3) - 1.0) < ZERO_THRESH) {
                    c3 = SIGN(c3);
                } else if(fabs(c3) > 1.0) {
                    // TODO NO SOLUTION
                    continue;
                }
                double arccos = acos(c3);
                q3[0] = arccos;
                q3[1] = 2.0 * PI - arccos;
                double denom = a2 * a2 + a3 * a3 + 2 * a2 * a3 * c3;
                double s3 = sin(arccos);
                double A = (a2 + a3 * c3), B = a3 * s3;
                q2[0] = atan2((A * p13y - B * p13x) / denom,
                    (A * p13x + B * p13y) / denom);
                q2[1] = atan2((A * p13y + B * p13x) / denom,
                    (A * p13x - B * p13y) / denom);
                double c23_0 = cos(q2[0] + q3[0]);
                double s23_0 = sin(q2[0] + q3[0]);
                double c23_1 = cos(q2[1] + q3[1]);
                double s23_1 = sin(q2[1] + q3[1]);
                q4[0] = atan2(c23_0 * x04y - s23_0 * x04x,
                    x04x * c23_0 + x04y * s23_0);
                q4[1] = atan2(c23_1 * x04y - s23_1 * x04x,
                    x04x * c23_1 + x04y * s23_1);
                ////////////////////////////////////////////////////////////////////////////////
                for(int k = 0; k < 2; k++) {
                    if(fabs(q2[k]) < ZERO_THRESH) {
                        q2[k] = 0.0;
                    } else if(q2[k] < 0.0) {
                        q2[k] += 2.0 * PI;
                    }
                    if(fabs(q4[k]) < ZERO_THRESH) {
                        q4[k] = 0.0;
                    } else if(q4[k] < 0.0) {
                        q4[k] += 2.0 * PI;
                    }
                    q_sols[num_sols * 6 + 0] = q1[i];
                    q_sols[num_sols * 6 + 1] = q2[k];
                    q_sols[num_sols * 6 + 2] = q3[k];
                    q_sols[num_sols * 6 + 3] = q4[k];
                    q_sols[num_sols * 6 + 4] = q5[i][j];
                    q_sols[num_sols * 6 + 5] = q6;
                    num_sols++;
                }
            }
        }
    }
    return num_sols;
}

void UniversalRobotsAccelerator::jacobian(const double* q, const double* p, double* J) const {
    double c1 = cos(q[0]), s1 = sin(q[0]);
    double c2 = cos(q[1]), s2 = sin(q[1]);
    double c3 = cos(q[2]), s3 = sin(q[2]);
    double c4 = cos(q[3]), s4 = sin(q[3]);
    double c5 = cos(q[4]), s5 = sin(q[4]);
    double c6 = cos(q[5]), s6 = sin(q[5]);
    double c23 = cos(q[1] + q[2]), s23 = sin(q[1] + q[2]);
    double c234 = cos(q[1] + q[2] + q[3]), s234 = sin(q[1] + q[2] + q[3]);
    double c45 = cos(q[3] + q[4]), s45 = sin(q[3] + q[4]);

    J[0] = -a2*c2*s1 - a3*c23*s1 + c1*c5*d6 + c1*c5*p[2] + c1*c6*p[0]*s5 + c1*d4 - c1*p[1]*s5*s6 - c234*c5*c6*p[0]*s1 + c234*c5*p[1]*s1*s6 + c234*d6*s1*s5 + c234*p[2]*s1*s5 + c6*p[1]*s1*s234 - d5*s1*s234 + p[0]*s1*s234*s6;
    J[1] = -c1*(a2*s2 + a3*s23 + c234*c6*p[1] - c234*d5 + c234*p[0]*s6 + c5*c6*p[0]*s234 - c5*p[1]*s234*s6 - d6*s234*s5 - p[2]*s234*s5);
    J[2] = c1*(-a3*s23 - c234*c6*p[1] + c234*d5 - c234*p[0]*s6 - c5*c6*p[0]*s234 + c5*p[1]*s234*s6 + d6*s234*s5 + p[2]*s234*s5);
    J[3] = c1*(-c234*c6*p[1] + c234*d5 - c234*p[0]*s6 - c5*c6*p[0]*s234 + c5*p[1]*s234*s6 + d6*s234*s5 + p[2]*s234*s5);
    J[4] = -c1*c234*(c5*d6 + c5*p[2] + c6*p[0]*s5 - p[1]*s5*s6) - s1*(-c5*c6*p[0] + c5*p[1]*s6 + d6*s5 + p[2]*s5);
    J[5] = -c1*s234*(c6*p[0] - p[1]*s6) - (c6*p[1] + p[0]*s6)*(c1*c234*c5 + s1*s5);
    J[6] = a2*c1*c2 + a3*c1*c23 + c1*c234*c5*c6*p[0] - c1*c234*c5*p[1]*s6 - c1*c234*d6*s5 - c1*c234*p[2]*s5 - c1*c6*p[1]*s234 + c1*d5*s234 - c1*p[0]*s234*s6 + c5*d6*s1 + c5*p[2]*s1 + c6*p[0]*s1*s5 + d4*s1 - p[1]*s1*s5*s6;
    J[7] = -s1*(a2*s2 + a3*s23 + c234*c6*p[1] - c234*d5 + c234*p[0]*s6 + c5*c6*p[0]*s234 - c5*p[1]*s234*s6 - d6*s234*s5 - p[2]*s234*s5);
    J[8] = s1*(-a3*s23 - c234*c6*p[1] + c234*d5 - c234*p[0]*s6 - c5*c6*p[0]*s234 + c5*p[1]*s234*s6 + d6*s234*s5 + p[2]*s234*s5);
    J[9] = s1*(-c234*c6*p[1] + c234*d5 - c234*p[0]*s6 - c5*c6*p[0]*s234 + c5*p[1]*s234*s6 + d6*s234*s5 + p[2]*s234*s5);
    J[10] = c1*(-c5*c6*p[0] + c5*p[1]*s6 + d6*s5 + p[2]*s5) - c234*s1*(c5*d6 + c5*p[2] + c6*p[0]*s5 - p[1]*s5*s6);
    J[11] = -s1*s234*(c6*p[0] - p[1]*s6) - (-c1*s5 + c234*c5*s1)*(c6*p[1] + p[0]*s6);
    J[12] = 0;
    J[13] = a2*c2 + a3*c23 + c234*c5*c6*p[0] - c234*c5*p[1]*s6 - c234*d6*s5 - c234*p[2]*s5 - c6*p[1]*s234 + d5*s234 - p[0]*s234*s6;
    J[14] = a3*c23 + c234*c5*c6*p[0] - c234*c5*p[1]*s6 - c234*d6*s5 - c234*p[2]*s5 - c6*p[1]*s234 + d5*s234 - p[0]*s234*s6;
    J[15] = c234*c5*c6*p[0] - c234*c5*p[1]*s6 - c234*d6*s5 - c234*p[2]*s5 - c6*p[1]*s234 + d5*s234 - p[0]*s234*s6;
    J[16] = -s234*(c5*d6 + c5*p[2] + c6*p[0]*s5 - p[1]*s5*s6);
    J[17] = c234*(c6*p[0] - p[1]*s6) - c5*s234*(c6*p[1] + p[0]*s6);
    J[18] = 0;
    J[19] = s1;
    J[20] = s1;
    J[21] = s1;
    J[22] = c1*s234;
    J[23] = -c1*c234*s5 + c5*s1;
    J[24] = 0;
    J[25] = -c1;
    J[26] = -c1;
    J[27] = -c1;
    J[28] = s1*s234;
    J[29] = -c1*c5 - c234*s1*s5;
    J[30] = 1;
    J[31] = 0;
    J[32] = 0;
    J[33] = 0;
    J[34] = -c234;
    J[35] = -s234*s5;
}

void UniversalRobotsAccelerator::hessian(const double* q, const double* qd, const double* p, double* H) const {
    double c1 = cos(q[0]), s1 = sin(q[0]);
    double c2 = cos(q[1]), s2 = sin(q[1]);
    double c3 = cos(q[2]), s3 = sin(q[2]);
    double c4 = cos(q[3]), s4 = sin(q[3]);
    double c5 = cos(q[4]), s5 = sin(q[4]);
    double c6 = cos(q[5]), s6 = sin(q[5]);
    double c23 = cos(q[1] + q[2]), s23 = sin(q[1] + q[2]);
    double c234 = cos(q[1] + q[2] + q[3]), s234 = sin(q[1] + q[2] + q[3]);
    double c45 = cos(q[3] + q[4]), s45 = sin(q[3] + q[4]);

    H[0] = -a2*c1*c2*qd[0] + a2*qd[1]*s1*s2 - a3*c1*c23*qd[0] + a3*s1*s23*(qd[1] + qd[2]) - c1*c234*c5*c6*p[0]*qd[0] + c1*c234*c5*p[1]*qd[0]*s6 + c1*c234*d6*qd[0]*s5 + c1*c234*p[2]*qd[0]*s5 + c1*c5*c6*p[0]*qd[4] - c1*c5*p[1]*qd[4]*s6 + c1*c6*p[1]*qd[0]*s234 - c1*c6*p[1]*qd[5]*s5 - c1*d5*qd[0]*s234 - c1*d6*qd[4]*s5 + c1*p[0]*qd[0]*s234*s6 - c1*p[0]*qd[5]*s5*s6 - c1*p[2]*qd[4]*s5 + c234*c5*c6*p[1]*qd[5]*s1 + c234*c5*d6*qd[4]*s1 + c234*c5*p[0]*qd[5]*s1*s6 + c234*c5*p[2]*qd[4]*s1 + c234*c6*p[0]*qd[4]*s1*s5 + c234*c6*p[1]*s1*(qd[1] + qd[2] + qd[3]) - c234*d5*s1*(qd[1] + qd[2] + qd[3]) + c234*p[0]*s1*s6*(qd[1] + qd[2] + qd[3]) - c234*p[1]*qd[4]*s1*s5*s6 + c5*c6*p[0]*s1*s234*(qd[1] + qd[2] + qd[3]) - c5*d6*qd[0]*s1 - c5*p[1]*s1*s234*s6*(qd[1] + qd[2] + qd[3]) - c5*p[2]*qd[0]*s1 - c6*p[0]*qd[0]*s1*s5 + c6*p[0]*qd[5]*s1*s234 - d4*qd[0]*s1 - d6*s1*s234*s5*(qd[1] + qd[2] + qd[3]) + p[1]*qd[0]*s1*s5*s6 - p[1]*qd[5]*s1*s234*s6 - p[2]*s1*s234*s5*(qd[1] + qd[2] + qd[3]);
    H[1] = c1*(-a2*c2*qd[1] - a3*c23*(qd[1] + qd[2]) - c234*c5*c6*p[0]*(qd[1] + qd[2] + qd[3]) + c234*c5*p[1]*s6*(qd[1] + qd[2] + qd[3]) - c234*c6*p[0]*qd[5] + c234*d6*s5*(qd[1] + qd[2] + qd[3]) + c234*p[1]*qd[5]*s6 + c234*p[2]*s5*(qd[1] + qd[2] + qd[3]) + c5*c6*p[1]*qd[5]*s234 + c5*d6*qd[4]*s234 + c5*p[0]*qd[5]*s234*s6 + c5*p[2]*qd[4]*s234 + c6*p[0]*qd[4]*s234*s5 + c6*p[1]*s234*(qd[1] + qd[2] + qd[3]) - d5*s234*(qd[1] + qd[2] + qd[3]) + p[0]*s234*s6*(qd[1] + qd[2] + qd[3]) - p[1]*qd[4]*s234*s5*s6) + qd[0]*s1*(a2*s2 + a3*s23 + c234*c6*p[1] - c234*d5 + c234*p[0]*s6 + c5*c6*p[0]*s234 - c5*p[1]*s234*s6 - d6*s234*s5 - p[2]*s234*s5);
    H[2] = c1*(-a3*c23*(qd[1] + qd[2]) - c234*c5*c6*p[0]*(qd[1] + qd[2] + qd[3]) + c234*c5*p[1]*s6*(qd[1] + qd[2] + qd[3]) - c234*c6*p[0]*qd[5] + c234*d6*s5*(qd[1] + qd[2] + qd[3]) + c234*p[1]*qd[5]*s6 + c234*p[2]*s5*(qd[1] + qd[2] + qd[3]) + c5*c6*p[1]*qd[5]*s234 + c5*d6*qd[4]*s234 + c5*p[0]*qd[5]*s234*s6 + c5*p[2]*qd[4]*s234 + c6*p[0]*qd[4]*s234*s5 + c6*p[1]*s234*(qd[1] + qd[2] + qd[3]) - d5*s234*(qd[1] + qd[2] + qd[3]) + p[0]*s234*s6*(qd[1] + qd[2] + qd[3]) - p[1]*qd[4]*s234*s5*s6) + qd[0]*s1*(a3*s23 + c234*c6*p[1] - c234*d5 + c234*p[0]*s6 + c5*c6*p[0]*s234 - c5*p[1]*s234*s6 - d6*s234*s5 - p[2]*s234*s5);
    H[3] = c1*(-c234*c5*c6*p[0]*(qd[1] + qd[2] + qd[3]) + c234*c5*p[1]*s6*(qd[1] + qd[2] + qd[3]) - c234*c6*p[0]*qd[5] + c234*d6*s5*(qd[1] + qd[2] + qd[3]) + c234*p[1]*qd[5]*s6 + c234*p[2]*s5*(qd[1] + qd[2] + qd[3]) + c5*c6*p[1]*qd[5]*s234 + c5*d6*qd[4]*s234 + c5*p[0]*qd[5]*s234*s6 + c5*p[2]*qd[4]*s234 + c6*p[0]*qd[4]*s234*s5 + c6*p[1]*s234*(qd[1] + qd[2] + qd[3]) - d5*s234*(qd[1] + qd[2] + qd[3]) + p[0]*s234*s6*(qd[1] + qd[2] + qd[3]) - p[1]*qd[4]*s234*s5*s6) - qd[0]*s1*(-c234*c6*p[1] + c234*d5 - c234*p[0]*s6 - c5*c6*p[0]*s234 + c5*p[1]*s234*s6 + d6*s234*s5 + p[2]*s234*s5);
    H[4] = c1*c234*(-c5*c6*p[0]*qd[4] + c5*p[1]*qd[4]*s6 + c6*p[1]*qd[5]*s5 + d6*qd[4]*s5 + p[0]*qd[5]*s5*s6 + p[2]*qd[4]*s5) - c1*qd[0]*(-c5*c6*p[0] + c5*p[1]*s6 + d6*s5 + p[2]*s5) + c1*s234*(qd[1] + qd[2] + qd[3])*(c5*d6 + c5*p[2] + c6*p[0]*s5 - p[1]*s5*s6) + c234*qd[0]*s1*(c5*d6 + c5*p[2] + c6*p[0]*s5 - p[1]*s5*s6) - s1*(c5*c6*p[1]*qd[5] + c5*d6*qd[4] + c5*p[0]*qd[5]*s6 + c5*p[2]*qd[4] + c6*p[0]*qd[4]*s5 - p[1]*qd[4]*s5*s6);
    H[5] = -c1*c234*(c6*p[0] - p[1]*s6)*(qd[1] + qd[2] + qd[3]) + c1*qd[5]*s234*(c6*p[1] + p[0]*s6) + qd[0]*s1*s234*(c6*p[0] - p[1]*s6) - qd[5]*(c6*p[0] - p[1]*s6)*(c1*c234*c5 + s1*s5) + (c6*p[1] + p[0]*s6)*(c1*c234*qd[4]*s5 + c1*c5*s234*(qd[1] + qd[2] + qd[3]) - c1*qd[0]*s5 + c234*c5*qd[0]*s1 - c5*qd[4]*s1);
    H[6] = -a2*c1*qd[1]*s2 - a2*c2*qd[0]*s1 - a3*c1*s23*(qd[1] + qd[2]) - a3*c23*qd[0]*s1 - c1*c234*c5*c6*p[1]*qd[5] - c1*c234*c5*d6*qd[4] - c1*c234*c5*p[0]*qd[5]*s6 - c1*c234*c5*p[2]*qd[4] - c1*c234*c6*p[0]*qd[4]*s5 - c1*c234*c6*p[1]*(qd[1] + qd[2] + qd[3]) + c1*c234*d5*(qd[1] + qd[2] + qd[3]) - c1*c234*p[0]*s6*(qd[1] + qd[2] + qd[3]) + c1*c234*p[1]*qd[4]*s5*s6 - c1*c5*c6*p[0]*s234*(qd[1] + qd[2] + qd[3]) + c1*c5*d6*qd[0] + c1*c5*p[1]*s234*s6*(qd[1] + qd[2] + qd[3]) + c1*c5*p[2]*qd[0] + c1*c6*p[0]*qd[0]*s5 - c1*c6*p[0]*qd[5]*s234 + c1*d4*qd[0] + c1*d6*s234*s5*(qd[1] + qd[2] + qd[3]) - c1*p[1]*qd[0]*s5*s6 + c1*p[1]*qd[5]*s234*s6 + c1*p[2]*s234*s5*(qd[1] + qd[2] + qd[3]) - c234*c5*c6*p[0]*qd[0]*s1 + c234*c5*p[1]*qd[0]*s1*s6 + c234*d6*qd[0]*s1*s5 + c234*p[2]*qd[0]*s1*s5 + c5*c6*p[0]*qd[4]*s1 - c5*p[1]*qd[4]*s1*s6 + c6*p[1]*qd[0]*s1*s234 - c6*p[1]*qd[5]*s1*s5 - d5*qd[0]*s1*s234 - d6*qd[4]*s1*s5 + p[0]*qd[0]*s1*s234*s6 - p[0]*qd[5]*s1*s5*s6 - p[2]*qd[4]*s1*s5;
    H[7] = -c1*qd[0]*(a2*s2 + a3*s23 + c234*c6*p[1] - c234*d5 + c234*p[0]*s6 + c5*c6*p[0]*s234 - c5*p[1]*s234*s6 - d6*s234*s5 - p[2]*s234*s5) + s1*(-a2*c2*qd[1] - a3*c23*(qd[1] + qd[2]) - c234*c5*c6*p[0]*(qd[1] + qd[2] + qd[3]) + c234*c5*p[1]*s6*(qd[1] + qd[2] + qd[3]) - c234*c6*p[0]*qd[5] + c234*d6*s5*(qd[1] + qd[2] + qd[3]) + c234*p[1]*qd[5]*s6 + c234*p[2]*s5*(qd[1] + qd[2] + qd[3]) + c5*c6*p[1]*qd[5]*s234 + c5*d6*qd[4]*s234 + c5*p[0]*qd[5]*s234*s6 + c5*p[2]*qd[4]*s234 + c6*p[0]*qd[4]*s234*s5 + c6*p[1]*s234*(qd[1] + qd[2] + qd[3]) - d5*s234*(qd[1] + qd[2] + qd[3]) + p[0]*s234*s6*(qd[1] + qd[2] + qd[3]) - p[1]*qd[4]*s234*s5*s6);
    H[8] = -c1*qd[0]*(a3*s23 + c234*c6*p[1] - c234*d5 + c234*p[0]*s6 + c5*c6*p[0]*s234 - c5*p[1]*s234*s6 - d6*s234*s5 - p[2]*s234*s5) + s1*(-a3*c23*(qd[1] + qd[2]) - c234*c5*c6*p[0]*(qd[1] + qd[2] + qd[3]) + c234*c5*p[1]*s6*(qd[1] + qd[2] + qd[3]) - c234*c6*p[0]*qd[5] + c234*d6*s5*(qd[1] + qd[2] + qd[3]) + c234*p[1]*qd[5]*s6 + c234*p[2]*s5*(qd[1] + qd[2] + qd[3]) + c5*c6*p[1]*qd[5]*s234 + c5*d6*qd[4]*s234 + c5*p[0]*qd[5]*s234*s6 + c5*p[2]*qd[4]*s234 + c6*p[0]*qd[4]*s234*s5 + c6*p[1]*s234*(qd[1] + qd[2] + qd[3]) - d5*s234*(qd[1] + qd[2] + qd[3]) + p[0]*s234*s6*(qd[1] + qd[2] + qd[3]) - p[1]*qd[4]*s234*s5*s6);
    H[9] = c1*qd[0]*(-c234*c6*p[1] + c234*d5 - c234*p[0]*s6 - c5*c6*p[0]*s234 + c5*p[1]*s234*s6 + d6*s234*s5 + p[2]*s234*s5) + s1*(-c234*c5*c6*p[0]*(qd[1] + qd[2] + qd[3]) + c234*c5*p[1]*s6*(qd[1] + qd[2] + qd[3]) - c234*c6*p[0]*qd[5] + c234*d6*s5*(qd[1] + qd[2] + qd[3]) + c234*p[1]*qd[5]*s6 + c234*p[2]*s5*(qd[1] + qd[2] + qd[3]) + c5*c6*p[1]*qd[5]*s234 + c5*d6*qd[4]*s234 + c5*p[0]*qd[5]*s234*s6 + c5*p[2]*qd[4]*s234 + c6*p[0]*qd[4]*s234*s5 + c6*p[1]*s234*(qd[1] + qd[2] + qd[3]) - d5*s234*(qd[1] + qd[2] + qd[3]) + p[0]*s234*s6*(qd[1] + qd[2] + qd[3]) - p[1]*qd[4]*s234*s5*s6);
    H[10] = -c1*c234*qd[0]*(c5*d6 + c5*p[2] + c6*p[0]*s5 - p[1]*s5*s6) + c1*(c5*c6*p[1]*qd[5] + c5*d6*qd[4] + c5*p[0]*qd[5]*s6 + c5*p[2]*qd[4] + c6*p[0]*qd[4]*s5 - p[1]*qd[4]*s5*s6) + c234*s1*(-c5*c6*p[0]*qd[4] + c5*p[1]*qd[4]*s6 + c6*p[1]*qd[5]*s5 + d6*qd[4]*s5 + p[0]*qd[5]*s5*s6 + p[2]*qd[4]*s5) - qd[0]*s1*(-c5*c6*p[0] + c5*p[1]*s6 + d6*s5 + p[2]*s5) + s1*s234*(qd[1] + qd[2] + qd[3])*(c5*d6 + c5*p[2] + c6*p[0]*s5 - p[1]*s5*s6);
    H[11] = -c1*qd[0]*s234*(c6*p[0] - p[1]*s6) - c234*s1*(c6*p[0] - p[1]*s6)*(qd[1] + qd[2] + qd[3]) + qd[5]*s1*s234*(c6*p[1] + p[0]*s6) - qd[5]*(-c1*s5 + c234*c5*s1)*(c6*p[0] - p[1]*s6) + (c6*p[1] + p[0]*s6)*(-c1*c234*c5*qd[0] + c1*c5*qd[4] + c234*qd[4]*s1*s5 + c5*s1*s234*(qd[1] + qd[2] + qd[3]) - qd[0]*s1*s5);
    H[12] = 0;
    H[13] = -a2*qd[1]*s2 - a3*s23*(qd[1] + qd[2]) - c234*c5*c6*p[1]*qd[5] - c234*c5*d6*qd[4] - c234*c5*p[0]*qd[5]*s6 - c234*c5*p[2]*qd[4] - c234*c6*p[0]*qd[4]*s5 - c234*c6*p[1]*(qd[1] + qd[2] + qd[3]) + c234*d5*(qd[1] + qd[2] + qd[3]) - c234*p[0]*s6*(qd[1] + qd[2] + qd[3]) + c234*p[1]*qd[4]*s5*s6 - c5*c6*p[0]*s234*(qd[1] + qd[2] + qd[3]) + c5*p[1]*s234*s6*(qd[1] + qd[2] + qd[3]) - c6*p[0]*qd[5]*s234 + d6*s234*s5*(qd[1] + qd[2] + qd[3]) + p[1]*qd[5]*s234*s6 + p[2]*s234*s5*(qd[1] + qd[2] + qd[3]);
    H[14] = -a3*s23*(qd[1] + qd[2]) - c234*c5*c6*p[1]*qd[5] - c234*c5*d6*qd[4] - c234*c5*p[0]*qd[5]*s6 - c234*c5*p[2]*qd[4] - c234*c6*p[0]*qd[4]*s5 - c234*c6*p[1]*(qd[1] + qd[2] + qd[3]) + c234*d5*(qd[1] + qd[2] + qd[3]) - c234*p[0]*s6*(qd[1] + qd[2] + qd[3]) + c234*p[1]*qd[4]*s5*s6 - c5*c6*p[0]*s234*(qd[1] + qd[2] + qd[3]) + c5*p[1]*s234*s6*(qd[1] + qd[2] + qd[3]) - c6*p[0]*qd[5]*s234 + d6*s234*s5*(qd[1] + qd[2] + qd[3]) + p[1]*qd[5]*s234*s6 + p[2]*s234*s5*(qd[1] + qd[2] + qd[3]);
    H[15] = -c234*c5*c6*p[1]*qd[5] - c234*c5*d6*qd[4] - c234*c5*p[0]*qd[5]*s6 - c234*c5*p[2]*qd[4] - c234*c6*p[0]*qd[4]*s5 - c234*c6*p[1]*(qd[1] + qd[2] + qd[3]) + c234*d5*(qd[1] + qd[2] + qd[3]) - c234*p[0]*s6*(qd[1] + qd[2] + qd[3]) + c234*p[1]*qd[4]*s5*s6 - c5*c6*p[0]*s234*(qd[1] + qd[2] + qd[3]) + c5*p[1]*s234*s6*(qd[1] + qd[2] + qd[3]) - c6*p[0]*qd[5]*s234 + d6*s234*s5*(qd[1] + qd[2] + qd[3]) + p[1]*qd[5]*s234*s6 + p[2]*s234*s5*(qd[1] + qd[2] + qd[3]);
    H[16] = -c234*(qd[1] + qd[2] + qd[3])*(c5*d6 + c5*p[2] + c6*p[0]*s5 - p[1]*s5*s6) + s234*(-c5*c6*p[0]*qd[4] + c5*p[1]*qd[4]*s6 + c6*p[1]*qd[5]*s5 + d6*qd[4]*s5 + p[0]*qd[5]*s5*s6 + p[2]*qd[4]*s5);
    H[17] = -c234*c5*(c6*p[1] + p[0]*s6)*(qd[1] + qd[2] + qd[3]) - c234*qd[5]*(c6*p[1] + p[0]*s6) - c5*qd[5]*s234*(c6*p[0] - p[1]*s6) + qd[4]*s234*s5*(c6*p[1] + p[0]*s6) - s234*(c6*p[0] - p[1]*s6)*(qd[1] + qd[2] + qd[3]);
    H[18] = 0;
    H[19] = c1*qd[0];
    H[20] = c1*qd[0];
    H[21] = c1*qd[0];
    H[22] = c1*c234*(qd[1] + qd[2] + qd[3]) - qd[0]*s1*s234;
    H[23] = -c1*c234*c5*qd[4] + c1*c5*qd[0] + c1*s234*s5*(qd[1] + qd[2] + qd[3]) + c234*qd[0]*s1*s5 - qd[4]*s1*s5;
    H[24] = 0;
    H[25] = qd[0]*s1;
    H[26] = qd[0]*s1;
    H[27] = qd[0]*s1;
    H[28] = c1*qd[0]*s234 + c234*s1*(qd[1] + qd[2] + qd[3]);
    H[29] = -c1*c234*qd[0]*s5 + c1*qd[4]*s5 - c234*c5*qd[4]*s1 + c5*qd[0]*s1 + s1*s234*s5*(qd[1] + qd[2] + qd[3]);
    H[30] = 0;
    H[31] = 0;
    H[32] = 0;
    H[33] = 0;
    H[34] = s234*(qd[1] + qd[2] + qd[3]);
    H[35] = -c234*s5*(qd[1] + qd[2] + qd[3]) - c5*qd[4]*s234;
}

void UniversalRobotsAccelerator::gradient(const double* q, double* G) const {
    throw std::runtime_error("UniversalRobotsAccelerator::gradient not implemented");
}