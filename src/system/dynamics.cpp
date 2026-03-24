#include "constants.hpp"
#include "utils.hpp"
#include "system/dynamics.hpp"
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <stdexcept>

namespace uav_ugv_sim {

    SystemParams::SystemParams(
        const double l,
        const SystemState& x_initial,
        const ControlInput& u_initial) : L(l), x0(x_initial), u0(u_initial) {}

    // Constructor Implementation for SystemModel
    SystemModel::SystemModel(const SystemState& x0, const StateCov& Q, const MeasCov& R, const SystemParams& params)
        : x_(x0), Q_(Q), R_(R), params_(params), gen_(std::random_device{}()) {

        // Compute Discrete-Time Cholesky decomps of Q and R for process and meas noise generation
        auto Q_dt = Q * DT;
        auto R_dt = R * DT;

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

    // Propagate nonlinear system dynamics model
    void SystemModel::Propagate(double t0, const ControlInput& u, bool add_noise) {
        DynamicsModel dyn(u);

        // Suppress false positive uninitialized warnings from ODEINT/Eigen copies
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuninitialized"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
        boost::numeric::odeint::runge_kutta_dopri5<SystemState> stepper;
        boost::numeric::odeint::integrate_adaptive(
            boost::numeric::odeint::make_controlled(1e-6, 1e-6, stepper), dyn, x_, t0, DT, DT / 10.0);
#pragma GCC diagnostic pop

        if (add_noise) {
            SystemState noise;
            std::normal_distribution<double> norm_dist(0.0, 1.0);
            for (int i = 0; i < noise.rows(); i++) {
                noise(i, 0) = norm_dist(gen_);
            }

            x_ += Svx_ * noise;
        }

        // ensure UGV and UAV headings are wrapped to [-pi, pi]
        x_(2) = WrapToPi(x_(2));
        x_(5) = WrapToPi(x_(5));
    }

    void SystemModel::CollectMeasurements() {
        y_(0) = WrapToPi(std::atan2(x_(4) - x_(1), x_(3) - x_(0)) - x_(2));
        y_(1) = std::sqrt(std::pow(x_(0) - x_(3), 2) + std::pow(x_(1) - x_(4), 2));
        y_(2) = WrapToPi(std::atan2(x_(1) - x_(4), x_(0) - x_(3)) - x_(5));
        y_(3) = x_(3);
        y_(4) = x_(4);

        ObservationState noise;
        std::normal_distribution<double> norm_dist(0.0, 1.0);
        for (int i = 0; i < noise.rows(); i++) {
            noise(i, 0) = norm_dist(gen_);
        }

        y_ += Svy_ * noise;
    }

    // Getter implementations
    const SystemState& SystemModel::GetState() const { return x_; }
    const ObservationState& SystemModel::GetSensorMeasurement() const { return y_; };

}