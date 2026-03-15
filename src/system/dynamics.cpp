#include "constants.hpp"
#include "utils.hpp"
#include "system/dynamics.hpp"
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <stdexcept>

namespace uav_ugv_sim {

SystemModel::Dynamics::Dynamics(const double& l, const ControlInput& control_vec) : L(l), u(control_vec) {}

void SystemModel::Dynamics::operator()(const SystemState& x, SystemState& dxdt, double) const {
    dxdt(0) = u(0) * std::cos(wrapToPi(x(2)));
    dxdt(1) = u(0) * std::sin(wrapToPi(x(2)));
    dxdt(2) = (u(0) / L) * std::tan(u(1));
    dxdt(3) = u(2) * std::cos(wrapToPi(x(5)));
    dxdt(4) = u(2) * std::sin(wrapToPi(x(5)));
    dxdt(5) = u(3);
}

// Constructor Implementation for SystemModel
SystemModel::SystemModel(const SystemState& x0, const StateCov& Q, const MeasCov& R, const SystemParams& params)
    : x_(x0), Q_(Q), R_(R), params_(params), gen_(std::random_device{}()) {
        // Compute Discrete-Time Cholesky decomps of Q and R for process and meas noise generation
        StateCov Q_dt = Q * DT;
        MeasCov R_dt = R * DT;
        Eigen::LLT<StateCov> q_llt(Q_dt);
        Eigen::LLT<MeasCov> r_llt(R_dt);
        if (q_llt.info() != Eigen::Success) {
            throw std::runtime_error("Q_dt is not positive definite");
        }
        if (r_llt.info() != Eigen::Success) {
            throw std::runtime_error("R_dt is not positive definite");
        }
        Svx_ = q_llt.matrixL();
        Svy_ = r_llt.matrixL();
    }

// Propagate method implementation
void SystemModel::propagate(double t0, const ControlInput& u, bool add_noise) {
    Dynamics dyn(params_.L, u);

    // Suppress false positive uninitialized warnings from ODEINT/Eigen copies
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuninitialized"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
    boost::numeric::odeint::runge_kutta4<SystemState> stepper;
    boost::numeric::odeint::integrate_const(stepper, dyn, x_, t0, DT, DT / 10.0);
#pragma GCC diagnostic pop

    if (add_noise) {
        SystemState noise;
        std::normal_distribution<double> norm_dist(0.0, 1.0);
        for (int i = 0; i < noise.rows(); i++) {
            noise(i,0) = norm_dist(gen_);
        }

        x_ += Svx_ * noise;
    }

    // ensure UGV and UAV headings are wrapped to [-pi, pi]
    x_(2,0) = wrapToPi(x_(3,0));
    x_(5,0) = wrapToPi(x_(5,0));
}

void SystemModel::collectMeasurements() {
    y_(0,0) = wrapToPi(std::atan2(x_(4,0) - x_(1,0), x_(3,0) - x_(0,0)) - x_(2,0));
    y_(1,0) = std::sqrt(std::pow(x_(0,0) - x_(3,0), 2) + std::pow(x_(1,0) - x_(4,0), 2));
    y_(2,0) = wrapToPi(std::atan2(x_(1,0) - x_(4,0), x_(0,0) - x_(3,0)) - x_(5,0));
    y_(3,0) = x_(3,0);
    y_(4,0) = x_(4,0);

    ObservationState noise;
    std::normal_distribution<double> norm_dist(0.0, 1.0);
    for (int i = 0; i < noise.rows(); i++) {
        noise(i,0) = norm_dist(gen_);
    }

    y_ += Svy_ * noise;
}

// Getter implementations
const SystemState& SystemModel::getState() const { return x_; }
const ObservationState& SystemModel::getSensorMeasurement() const { return y_; };


}