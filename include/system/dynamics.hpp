#pragma once

#include "../constants.hpp"
#include "../utils.hpp"
#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include <cmath>
#include <vector>
#include <random>
#include <stdexcept>

namespace uav_ugv_sim {

struct SystemParams {
    double L;
    SystemState x0;
    ControlInput u0;

    SystemParams(
        const double l = 0.5,
        const SystemState& x_initial = SystemState(10.0, 0.0, PI / 2.0, -60.0, 0.0, -PI / 2.0),
        const ControlInput& u_initial = ControlInput(2.0, -PI / 18, 12.0, PI / 25.0)
    ) noexcept
        : L(l), x0(x_initial), u0(u_initial) {}
};

class SystemModel {
    private:
        SystemState x_;
        ObservationState y_;
        StateCov Q_;
        StateCov Svx_;
        MeasCov R_;
        MeasCov Svy_;
        SystemParams params_;

        std::mt19937 gen_;

        // ODE integration functor: defines dx/dt = f(x, u) for combined system
        struct Dynamics {
            const double& L;
            const ControlInput& u;

            Dynamics(const double& l, const ControlInput& control_vec);

            void operator()(const SystemState& x, SystemState& dxdt, double) const;
        };

    public:
        SystemModel(
            const SystemState& x0,
            const StateCov& Q,
            const MeasCov& R,
            const SystemParams& params
        );

        void propagate(double t0, const ControlInput& u, bool add_noise = true);

        void collectMeasurements();

        const SystemState& getState() const;

        const ObservationState& getSensorMeasurement() const;
};

}