#include "constants.hpp"
#include "utils.hpp"
#include "filters/ekf.hpp"
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>
#include <cmath>

namespace uav_ugv_sim {

EKF::EKF(const SystemState& x0, const EkfParams& m_params) : xhat_(x0), params_(m_params) {
    P_ = params_.P0;
}

auto EKF::measurmentModel(const SystemState& xhat) const -> ObsSensativity {
    ObsSensativity H = ObsSensativity::Zero();
    H(0,0) = (xhat(4,0) - xhat(1,0)) / (std::pow(xhat(4,0) - xhat(1,0), 2) / std::pow(xhat(3,0) - xhat(0,0), 2) + 1) * std::pow(xhat(3,0) - xhat(0,0), 2);
    H(0,1) = -1 / (std::pow(xhat(4,0) - xhat(1,0), 2) / std::pow(xhat(3,0) - xhat(0,0), 2) + 1) * std::pow(xhat(3,0) - xhat(0,0), 2);
    H(0,2) = -1;
    H(0,3) = -H(0,0);
    H(0,4) = -H(0,1);
    H(1,0) = (xhat(0,0) - xhat(3,0)) / std::sqrt(std::pow(xhat(0,0) - xhat(3,0), 2) + std::pow(xhat(1,0) - xhat(4,0), 2));
    H(1,1) = (xhat(1,0) - xhat(4,0)) / std::sqrt(std::pow(xhat(0,0) - xhat(3,0), 2) + std::pow(xhat(1,0) - xhat(4,0), 2));
    H(1,3) = -H(1,0);
    H(1,4) = -H(1,1);
    H(2,0) = -(xhat(1,0) - xhat(4,0)) / (std::pow(xhat(1,0) - xhat(4,0), 2) / std::pow(xhat(0,0) - xhat(3,0), 2) + 1) * std::pow(xhat(0,0) - xhat(3,0), 2);
    H(2,1) = 1 / (std::pow(xhat(1,0) - xhat(4,0), 2) / std::pow(xhat(0,0) - xhat(3,0), 2) + 1) * std::pow(xhat(0,0) - xhat(3,0), 2);
    H(2,3) = -H(2,0);
    H(2,4) = -H(2,1);
    H(2,5) = -1;
    H(3,3) = 1;
    H(4,4) = 1;

    return H;
}

auto EKF::computeJacobianF(const SystemState& xhat, const ControlInput& u, double dt) const -> StateTransition {
    // Generates linearized discrete-time state transition matrix from plant dynamics for estimator
    StateTransition A = StateTransition::Zero();
    A(0,2) = -u(0) * std::sin(wrapToPi(xhat(2)));
    A(1,2) = u(0) * std::cos(wrapToPi(xhat(2)));
    A(3,5) = -u(2) * std::sin(wrapToPi(xhat(5)));
    A(4,5) = u(2) * std::cos(wrapToPi(xhat(5)));

    auto F = StateTransition::Identity() + dt*A;

    return F;
}

// Propagate nonlinear system dynamics model
void EKF::propagate(double t0, const ControlInput& u) {
    DynamicsModel dyn(u);

    // Suppress false positive uninitialized warnings from ODEINT/Eigen copies
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuninitialized"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
    boost::numeric::odeint::runge_kutta_dopri5<SystemState> stepper;
    boost::numeric::odeint::integrate_adaptive(
        boost::numeric::odeint::make_controlled(1e-6, 1e-6, stepper), dyn, xhat_, t0, DT, DT / 10.0
    );
#pragma GCC diagnostic pop

    // ensure UGV and UAV headings are wrapped to [-pi, pi]
    xhat_(2,0) = wrapToPi(xhat_(2,0));
    xhat_(5,0) = wrapToPi(xhat_(5,0));
}

void EKF::predict(double t0, const ControlInput& u) {
    EKF::propagate(t0, u);
    auto F = EKF::computeJacobianF(xhat_, u, DT);
    P_ = F * P_ * F.transpose() + (params_.Omega * params_.Q * params_.Omega.transpose());
}

void EKF::correct(const ObservationState& z) {
    auto H = EKF::measurmentModel(xhat_);
    yhat_ = z - (H * xhat_);

    yhat_(0) = wrapToPi(yhat_(0));
    yhat_(2) = wrapToPi(yhat_(2));

    auto S = H * P_ * H.transpose() + params_.R;
    auto K = P_ * H.transpose() * S.inverse();

    xhat_ += K * yhat_;
    xhat_(0) = wrapToPi(xhat_(0));
    xhat_(2) = wrapToPi(xhat_(2));

    P_ = (StateCov::Identity() - K * H) * P_;
}

// Getter implementations
const SystemState& EKF::getEstimatedState() const { return xhat_; }
const ObservationState& EKF::getMeasurementResiduals() const { return yhat_; }
const Eigen::Matrix<double, 6, 1> EKF::getCovarDiagonal() const { return P_.diagonal(); }

}