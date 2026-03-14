#include "constants.hpp"
#include "utils.hpp"
#include "system/dynamics.hpp"
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>
#include <Eigen/Dense>

namespace uav_ugv_sim {

SystemModel::SystemModel(const SystemState& x0, const StateCov& Q, const SystemParams& params)
    : x_(x0), Q_(Q), params_(params) {}

    void SystemModel::SystemModel::propagate(double t0, const ControlInput& u, bool add_noise = true) {
        Dynamics dyn(params_.L, u);
        boost::numeric::odeint::runge_kutta4<SystemState> stepper;
        boost::numeric::odeint::integrate_const(stepper, dyn, x_, t0, DT, DT);

        // Compute Discrete-Time normally distributed process noise via Cholesky decomp of Q * dt
        StateCov Q_dt = Q_ * DT;
    }


}