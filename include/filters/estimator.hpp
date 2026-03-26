#pragma once

#include "../constants.hpp"
#include <string_view>

namespace uav_ugv_sim {

    // Available filter types for Cooperative Localization Simulations
    enum class EstimatorType : std::uint8_t { EKF, UKF };

    constexpr std::string_view ToString(EstimatorType estimator) {
        switch (estimator) {
            case EstimatorType::EKF: return "EKF";
            case EstimatorType::UKF: return "UKF";
        }
        return "Unknown";
    }

    struct FilterParams {
        StateCov P0 = Eigen::Matrix<double, 6, 1>(5.0, 5.0, 2.0, 20.0, 20.0, 2.0).asDiagonal();
        StateCov Q = Q_TRUE;
        MeasCov R = R_TRUE;
        StateCov Omega = StateCov::Identity() * DT;

        FilterParams() = default;

        explicit FilterParams(
            const StateCov& p_initial,
            const StateCov& q,
            const MeasCov& r
        ) noexcept : P0(p_initial), Q(q), R(r) {}

        void ScaleDefaultQR(double q_scale, double r_scale) {
            Q *= q_scale;
            R *= r_scale;
        }
    };

    class Estimator {
        protected:
            Estimator(const SystemState& x0, const FilterParams& filter_params);

            SystemState xhat_;
            ObservationState ey_;
            StateCov P_;
            EstimatorType estimator_type_;
            FilterParams params_;

            auto ComputeJacobianF(const SystemState& xhat, const ControlInput& u, double dt) const -> StateTransition;

            auto ComputeJacobianG(const SystemState& xhat, const ControlInput& u, double dt) const -> ControlMatrix;

            auto MeasurmentModel(const SystemState& xhat) const -> ObsSensativity;

        public:
            virtual ~Estimator();

            virtual void Propagate(double t0, const ControlInput& u) = 0;

            virtual void Predict(double t0, const ControlInput& u) = 0;

            virtual void Correct(const ObservationState& z) = 0;

            const SystemState& GetEstimatedState() const;

            const ObservationState& GetFilterResiduals() const;

            const SystemState GetCovarDiagonal() const;
    };

}