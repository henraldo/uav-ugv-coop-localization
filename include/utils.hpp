#pragma once

#include "constants.hpp"
#include <cmath>
#include <fstream>
#include <filesystem>
#include <string>
#include <vector>
#include <iostream>

namespace uav_ugv_sim {
// Definitions for global utility functions

inline double wrapToPi(double angle) {
    return std::fmod(angle + PI, 2 * PI) - PI;
};

// ODE integration functor: defines dx/dt = f(x, u) for combined system
struct DynamicsModel {
    const ControlInput& u;
    const double& L;

    DynamicsModel(const ControlInput& control_vec, const double& l = UGV_L) : u(control_vec), L(l) {}

    void operator()(const SystemState& x, SystemState& dxdt, double) const{
        dxdt(0) = u(0) * std::cos(wrapToPi(x(2)));
        dxdt(1) = u(0) * std::sin(wrapToPi(x(2)));
        dxdt(2) = (u(0) / L) * std::tan(u(1));
        dxdt(3) = u(2) * std::cos(wrapToPi(x(5)));
        dxdt(4) = u(2) * std::sin(wrapToPi(x(5)));
        dxdt(5) = u(3);
    };
};

}