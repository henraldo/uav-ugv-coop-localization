#pragma once

#include "../constants.hpp"
#include "../system/dynamics.hpp"

namespace uav_ugv_sim {

struct EkfParams {
    StateCov P0;
    StateCov Q;
    StateCov Omega;
    MeasCov R;

    EkfParams(
        const StateCov& p_initial = Eigen::Matrix<double, 6, 1>(5.0, 5.0, 2.0, 20.0, 20.0, 2.0).asDiagonal(),
        const StateCov& q_initial = Eigen::Matrix<double, 6, 1>(0.001, 0.001, 3.25, 0.01, 0.01, 3.25).asDiagonal(),
        const MeasCov& r_initial = Eigen::Matrix<double, 5, 1>(0.001, 3.0, 0.01, 2.0, 2.0).asDiagonal()
    ) noexcept
        : P0(p_initial), Q(q_initial), R(r_initial) {
            Omega = StateCov::Identity() * DT;
        }
};

class EKF {
    private:
        SystemState xhat_;
        ObservationState yhat_;
        StateCov P_;
        EkfParams params_;

        auto computeJacobianF(const SystemState& xhat, const ControlInput& u, double dt) const -> StateTransition;

        auto measurmentModel(const SystemState& xhat) const -> ObsSensativity;

    public:
        EKF(const SystemState& x0, const EkfParams& m_params = EkfParams{});

        void propagate(double t0, const ControlInput& u);

        void predict(double t0, const ControlInput& u);

        void correct(const ObservationState& z);

        const SystemState& getEstimatedState() const;

        const ObservationState& getMeasurementResiduals() const;

        const Eigen::Matrix<double, 6, 1> getCovarDiagonal() const;
};

}