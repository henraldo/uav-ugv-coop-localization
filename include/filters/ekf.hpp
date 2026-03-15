#pragma once

#include "../constants.hpp"
#include "../system/dynamics.hpp"

namespace uav_ugv_sim {

struct EkfParams {
    StateCov P0;
    StateCov Q;
    MeasCov R;

    EkfParams(
        const StateCov& p_initial = Eigen::Matrix<double, 6, 1>(5.0, 5.0, 2.0, 20.0, 20.0, 2.0).asDiagonal(),
        const StateCov& q_initial = Eigen::Matrix<double, 6, 1>(0.001, 0.001, 3.25, 0.01, 0.01, 3.25).asDiagonal(),
        const MeasCov& r_initial = Eigen::Matrix<double, 5, 1>(0.001, 3.0, 0.01, 2.0, 2.0).asDiagonal()
    ) noexcept
        : P0(p_initial), Q(q_initial), R(r_initial) {}
};

class EKF {
    private:
        EkfParams params_;

        auto computeJacobianF(const SystemState& x, double dt, const SystemParams& sys_params) const -> StateTransition;

        auto measurmentModel(const SystemState& x) const -> ObsSensativity;

    public:
        EKF(const EkfParams& m_params = EkfParams{});

        void predict(SystemState& x_hat, StateCov& P, const SystemParams& sys_params);

        void correct(SystemState& x_hat, StateCov& P, ObservationState& z);

};

}