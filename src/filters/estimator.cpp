#include "constants.hpp"
#include "utils.hpp"
#include "filters/estimator.hpp"
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>
#include <cmath>

namespace uav_ugv_sim {

Estimator::~Estimator() = default;

Estimator::Estimator(const SystemState& x0, const FilterParams& filter_params)
: xhat_(x0), P_(filter_params.P0), params_(filter_params) {}

// Generates linearized measurement model updates for filter innovation calculation
auto Estimator::MeasurmentModel(const SystemState& xhat) const -> ObsSensativity {
    ObsSensativity H = ObsSensativity::Zero();

    H(0,0) = (xhat(4) - xhat(1)) / (std::pow(xhat(4) - xhat(1), 2) / std::pow(xhat(3) - xhat(0), 2) + 1) * std::pow(xhat(3) - xhat(0), 2);
    H(0,1) = -1 / (std::pow(xhat(4) - xhat(1), 2) / std::pow(xhat(3) - xhat(0), 2) + 1) * std::pow(xhat(3) - xhat(0), 2);
    H(0,2) = -1;
    H(0,3) = -H(0,0);
    H(0,4) = -H(0,1);
    H(1,0) = (xhat(0) - xhat(3)) / std::sqrt(std::pow(xhat(0) - xhat(3), 2) + std::pow(xhat(1) - xhat(4), 2));
    H(1,1) = (xhat(1) - xhat(4)) / std::sqrt(std::pow(xhat(0) - xhat(3), 2) + std::pow(xhat(1) - xhat(4), 2));
    H(1,3) = -H(1,0);
    H(1,4) = -H(1,1);
    H(2,0) = -(xhat(1) - xhat(4)) / (std::pow(xhat(1) - xhat(4), 2) / std::pow(xhat(0) - xhat(3), 2) + 1) * std::pow(xhat(0) - xhat(3), 2);
    H(2,1) = 1 / (std::pow(xhat(1) - xhat(4), 2) / std::pow(xhat(0) - xhat(3), 2) + 1) * std::pow(xhat(0) - xhat(3), 2);
    H(2,3) = -H(2,0);
    H(2,4) = -H(2,1);
    H(2,5) = -1;
    H(3,3) = 1;
    H(4,4) = 1;

    return H;
}

// Generates linearized discrete-time state transition matrix from plant dynamics for estimator
auto Estimator::ComputeJacobianF(const SystemState& xhat, const ControlInput& u, double dt) const -> StateTransition {
    StateTransition A = StateTransition::Zero();

    A(0,2) = -u(0) * std::sin(wrapToPi(xhat(2)));
    A(1,2) = u(0) * std::cos(wrapToPi(xhat(2)));
    A(3,5) = -u(2) * std::sin(wrapToPi(xhat(5)));
    A(4,5) = u(2) * std::cos(wrapToPi(xhat(5)));

    auto F = StateTransition::Identity() + dt * A;

    return F;
}

// Generates linearized discrete-time control input matrix from plant dynamics for estimator
auto Estimator::ComputeJacobianG(const SystemState& xhat, const ControlInput& u, double dt) const -> ControlMatrix {
    ControlMatrix B = ControlMatrix::Zero();

    B(0,0) = std::cos(wrapToPi(xhat(2)));
    B(1,0) = std::sin(wrapToPi(xhat(2)));
    B(2,0) = std::tan(u(1)) / UGV_L;
    B(2,1) = (u(0) / UGV_L) * std::pow(sec(u(1)), 2);
    B(3,2) = std::cos(wrapToPi(xhat(5)));
    B(4,2) = std::sin(wrapToPi(xhat(5)));
    B(5,3) = 1;

    auto G = dt * B;

    return G;
}

// Getter implementations
const SystemState& Estimator::GetEstimatedState() const { return xhat_; }
const ObservationState& Estimator::GetFilterResiduals() const { return ey_; }
const Eigen::Matrix<double, 6, 1> Estimator::GetCovarDiagonal() const { return P_.diagonal(); }

}