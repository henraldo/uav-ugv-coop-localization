#include "constants.hpp"
#include "utils.hpp"
#include "system/dynamics.hpp"
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>
#include <stdexcept>

namespace uav_ugv_sim {

    SystemParams::SystemParams(const SystemState& x_initial, const ControlInput& u_initial)
        : x0(x_initial), u0(u_initial) {}

    // Constructor Implementation for SystemModel
    SystemModel::SystemModel(const SystemState& x0, const StateCov& Q, const MeasCov& R)
        : x_(x0), Q_(Q), R_(R), gen_(std::random_device{}()) {

        y_ = SensorModel(x_);

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

    // Sensor model h(x) -> y = [ heading_ugv, range, heading_uav, ξa, ηa ]^T
    // Generates measurements for fully nonlinear measurement model
    auto SystemModel::SensorModel(const SystemState& x) const -> ObservationState {
        ObservationState z;
        auto dx = x(3) - x(0);
        auto dy = x(4) - x(1);
        double heading_ugv = WrapToPi(std::atan2(dy, dx) - x(2));
        double heading_uav = WrapToPi(std::atan2(-dy, -dx) - x(5));
        double range = std::sqrt((dx * dx) + (dy * dy));

        z(0) = heading_ugv;
        z(1) = range;
        z(2) = heading_uav;
        z(3) = x(3);
        z(4) = x(4);

        return z;
    }

    std::tuple<std::vector<double>, Eigen::MatrixXd, Eigen::MatrixXd> SystemModel::GenerateGroundTruthData(double sim_time_seconds, ControlInput& u) {
        DynamicsModel dyn(u);

        std::vector<double> times;
        std::vector<SystemState> states;
        TrajectoryObserver obs(times, states);

        // Suppress false positive uninitialized warnings from ODEINT/Eigen copies
        #pragma GCC diagnostic push
        #pragma GCC diagnostic ignored "-Wuninitialized"
        #pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
        boost::numeric::odeint::integrate_const(
            boost::numeric::odeint::runge_kutta4<SystemState>{}, dyn, x_, 0.0, sim_time_seconds, DT, obs);
        #pragma GCC diagnostic pop

        const size_t n_steps = times.size();
        Eigen::MatrixXd x_truth(6, n_steps);
        Eigen::MatrixXd y_truth(5, n_steps);

        for (size_t i = 0; i < n_steps; i++) {
            SystemState proc_noise;
            ObservationState meas_noise;
            std::normal_distribution<double> norm_dist(0.0, 1.0);

            for (int j = 0; j < proc_noise.rows(); j++) {
                proc_noise(j, 0) = norm_dist(gen_);
            }
            for (int k = 0; k < meas_noise.rows(); k++) {
                meas_noise(k, 0) = norm_dist(gen_);
            }

            x_truth.col(i) = states[i] + (Svx_.transpose() * proc_noise);
            y_truth.col(i) = SensorModel(states[i]) + (Svy_.transpose() * meas_noise);

        }

        return {times, x_truth, y_truth};
    }

}