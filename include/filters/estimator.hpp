#pragma once

#include "../constants.hpp"
#include <string_view>

namespace uav_ugv_sim {

    // Available filter types for Cooperative Localization Simulations
    enum class EstimatorType : std::uint8_t { EKF, UKF };

    constexpr std::string_view ToString(EstimatorType estimator) {
        switch (estimator) {
            case EstimatorType::EKF: return "ekf";
            case EstimatorType::UKF: return "ukf";
        }
        return "Unknown";
    }

    struct FilterParams {
        StateCov P0 = StateCov::Identity();
        StateCov Q = Q_TRUE;
        MeasCov R = R_TRUE;
        StateCov Omega = StateCov::Identity() * DT;
        EstimatorType filter_type = EstimatorType::EKF;

        FilterParams() = default;

        explicit FilterParams(
            const StateCov& p_initial,
            const StateCov& q,
            const MeasCov& r,
            const EstimatorType& filt_type
        ) noexcept : P0(p_initial), Q(q), R(r), filter_type(filt_type) {}

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
            MeasCov S_;
            FilterParams params_;

            auto ComputeJacobianF(const SystemState& xhat, const ControlInput& u, double dt) const -> StateTransition;

            auto ComputeJacobianG(const SystemState& xhat, const ControlInput& u, double dt) const -> ControlMatrix;

            auto ComputeJacobianH(const SystemState& xhat) const -> ObsSensativity;

            auto SensorModel(const SystemState& xhat) const -> ObservationState;

        public:
            virtual ~Estimator();

            virtual void Propagate(double t0, const ControlInput& u) = 0;

            virtual void Predict(double t0, const ControlInput& u) = 0;

            virtual void Correct(const ObservationState& z) = 0;

            const SystemState& GetEstimatedState() const;

            const ObservationState& GetFilterResiduals() const;

            const SystemState GetCovarDiagonal() const;

            const StateCov& GetCovariance() const;

            const MeasCov& GetInnovCovariance() const;
    };

}