#pragma once

#include "../constants.hpp"
#include "../utils.hpp"
#include <random>
#include <vector>
#include <tuple>


namespace uav_ugv_sim {

    struct TrajectoryObserver {
        std::vector<double>& times;
        std::vector<SystemState>& states;

        TrajectoryObserver(
            std::vector<double>& t,
            std::vector<SystemState>& s
        ) : times(t), states(s) {}

        void operator()(const SystemState& x, double t) const {
            SystemState x_wrapped;
            x_wrapped = x;
            x_wrapped(2) = WrapToPi(x_wrapped(2));
            x_wrapped(5) = WrapToPi(x_wrapped(5));

            times.push_back(t);
            states.push_back(x_wrapped);
        }
    };

    struct SystemParams {
        SystemState x0 = SystemState(10.0, 0.0, PI / 2.0, -60.0, 0.0, -PI / 2.0);
        ControlInput u0 = ControlInput(2.0, -PI / 18, 12.0, PI / 25.0);

        SystemParams() = default;

        explicit SystemParams(const SystemState& x_initial, const ControlInput& u_initial);
    };

    class SystemModel {
        private:
            SystemState x_;
            ObservationState y_;
            StateCov Q_;
            StateCov Svx_;
            MeasCov R_;
            MeasCov Svy_;

            std::mt19937 gen_;

            auto SensorModel(const SystemState& x) const -> ObservationState;

        public:
            SystemModel(const SystemState& x0, const StateCov& Q, const MeasCov& R);

            std::tuple<std::vector<double>, Eigen::MatrixXd, Eigen::MatrixXd> GenerateGroundTruthData(double sim_time_seconds, ControlInput& u);

    };

}