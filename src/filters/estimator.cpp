#include "constants.hpp"
#include "utils.hpp"
#include "filters/estimator.hpp"
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>
#include <cmath>

namespace uav_ugv_sim
{

    Estimator::~Estimator() = default;

    Estimator::Estimator(const SystemState& x0, const FilterParams& filter_params)
        : xhat_(x0), P_(filter_params.P0), params_(filter_params) {
            ey_ = ObservationState::Zero();
        }

    // Generates linearized measurement model updates for filter innovation calculation
    auto Estimator::ComputeJacobianH(const SystemState& xhat) const -> ObsSensativity {
        ObsSensativity H = ObsSensativity::Zero();
        double dx = xhat(3) - xhat(0);
        double dy = xhat(4) - xhat(1);
        double r2 = (dx * dx) + (dy * dy);

        // Prevent divide by zero error
        if (r2 < 1e-8) {
            return H;
        }
        double r = std::sqrt(r2);

        // row 0 = atan2(dy, dx) - x_wg
        H(0, 0) = dy / r2;
        H(0, 1) = -dx / r2;
        H(0, 2) = -1;
        H(0, 3) = -dy / r2;
        H(0, 4) = dx / r2;

        // row 1 = range
        H(1, 0) = -dx / r;
        H(1, 1) = -dy / r;
        H(1, 3) = dx / r;
        H(1, 4) = dy / r;

        // row 2 = atan2(-dy, -dx) - x_wa
        H(2, 0) = dy / r2;
        H(2, 1) = -dx / r2;
        H(2, 3) = -dy / r2;
        H(2, 4) = dx / r2;
        H(2, 5) = -1;

        // row 3 --> direct UAV position
        H(3, 3) = 1;
        H(4, 4) = 1;

        return H;
    }

    // Sensor model h(x) -> y = [ heading_ugv, range, heading_uav, ξa, ηa ]^T
    // Generates measurements for fully nonlinear measurement model
    auto Estimator::SensorModel(const SystemState& x) const -> ObservationState {
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

    // Generates linearized discrete-time state transition matrix from plant dynamics for estimator
    auto Estimator::ComputeJacobianF(const SystemState& xhat, const ControlInput& u, double dt) const -> StateTransition {
        StateTransition A = StateTransition::Zero();

        A(0, 2) = -u(0) * std::sin(xhat(2));
        A(1, 2) = u(0) * std::cos(xhat(2));
        A(3, 5) = -u(2) * std::sin(xhat(5));
        A(4, 5) = u(2) * std::cos(xhat(5));

        auto F = StateTransition::Identity() + (dt * A);

        return F;
    }

    // Generates linearized discrete-time control input matrix from plant dynamics for estimator
    auto Estimator::ComputeJacobianG(const SystemState& xhat, const ControlInput& u, double dt) const -> ControlMatrix {
        ControlMatrix B = ControlMatrix::Zero();

        B(0, 0) = std::cos(WrapToPi(xhat(2)));
        B(1, 0) = std::sin(WrapToPi(xhat(2)));
        B(2, 0) = std::tan(u(1)) / UGV_L;
        B(2, 1) = (u(0) / UGV_L) * std::pow(sec(u(1)), 2);
        B(3, 2) = std::cos(WrapToPi(xhat(5)));
        B(4, 2) = std::sin(WrapToPi(xhat(5)));
        B(5, 3) = 1;

        auto G = dt * B;

        return G;
    }

    // Getter implementations
    const SystemState& Estimator::GetEstimatedState() const { return xhat_; }
    const ObservationState& Estimator::GetFilterResiduals() const { return ey_; }
    const Eigen::Matrix<double, 6, 1> Estimator::GetCovarDiagonal() const { return P_.diagonal(); }

}