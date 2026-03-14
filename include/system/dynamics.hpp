#pragma once

#include "../constants.hpp"
#include "../utils.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <random>

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
        StateCov Q_;
        SystemParams params_;

        std::mt19937 gen_;

        // ODE integration functor: defines dx/dt = f(x, u) for combined system
        struct Dynamics {
            const double& L;
            const ControlInput& u;

            Dynamics(const double& l, const ControlInput& control_vec) : L(l), u(control_vec) {}

            void operator()(const SystemState& x, SystemState& dxdt, double t) const {
                dxdt(0) = u(0) * std::cos(wrapToPi(x(2)));
                dxdt(1) = u(0) * std::sin(wrapToPi(x(2)));
                dxdt(2) = (u(0) / L) * std::tan(u(1));
                dxdt(3) = u(2) * std::cos(wrapToPi(x(5)));
                dxdt(4) = u(2) * std::sin(wrapToPi(x(5)));
                dxdt(5) = u(3);
            }
        };

        struct SensorObservations {
            std::vector<TrajectAndObsHist>& system_history;

            SensorObservations(std::vector<TrajectAndObsHist>& sys_hist) : system_history(sys_hist) {}

            void operator()(const )
        };

    public:
        SystemModel(const SystemState& x0, const StateCov& Q, const SystemParams& params)
            : x_(x0), Q_(Q), params_(params), gen_(std::random_device{}()) {}

        void propagate(double t0, const ControlInput& u, bool add_noise = true);

        const SystemState& getState() const { return x_; }
};

}