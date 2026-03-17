#pragma once

#include "../constants.hpp"

namespace uav_ugv_sim {

struct EkfParams {
    StateCov P0 = Eigen::Matrix<double, 6, 1>(5.0, 5.0, 2.0, 20.0, 20.0, 2.0).asDiagonal();
    StateCov Q = Q_TRUE;
    MeasCov R = R_TRUE;
    StateCov Omega = StateCov::Identity() * DT;
    double qTune = 1.0;
    double rTune = 1.0;

    EkfParams() = default;

    explicit EkfParams(const StateCov& p_initial, const double q_scale, const double r_scale);
};

class EKF {
    private:
        SystemState xhat_;
        ObservationState ey_;
        StateCov P_;
        EkfParams params_;

        auto computeJacobianF(const SystemState& xhat, const ControlInput& u, double dt) const -> StateTransition;

        auto measurmentModel(const SystemState& xhat) const -> ObsSensativity;

    public:
        EKF(const SystemState& x0, const EkfParams& m_params);

        void propagate(double t0, const ControlInput& u);

        void predict(double t0, const ControlInput& u);

        void correct(const ObservationState& z);

        const SystemState& getEstimatedState() const;

        const ObservationState& getFilterResiduals() const;

        const SystemState getCovarDiagonal() const;
};

}