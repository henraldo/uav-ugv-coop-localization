#pragma once

#include "../constants.hpp"
#include "../utils.hpp"
#include <random>

namespace uav_ugv_sim {

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

    public:
        SystemModel(const SystemState& x0, const StateCov& Q, const MeasCov& R);

        void Propagate(double t0, const ControlInput& u, bool add_noise = true);

        void CollectMeasurements();

        const SystemState& GetState() const;

        const ObservationState& GetSensorMeasurement() const;
};

}