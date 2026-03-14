#pragma once

#include "constants.hpp"
#include <cmath>

namespace uav_ugv_sim {

// Definitions for global utility functions

inline double wrapToPi(double angle) {
    return std::fmod(angle + PI, 2 * PI) - PI;
};

}