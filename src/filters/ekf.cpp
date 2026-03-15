#include "constants.hpp"
#include "utils.hpp"
#include "filters/ekf.hpp"
#include <cmath>

namespace uav_ugv_sim {

EKF::EKF(const EkfParams& m_params) : params_(m_params) {}

auto EKF::measurmentModel(const SystemState& x) const -> ObsSensativity {
    ObsSensativity H = ObsSensativity::Zero();
    H(0,0) = (x(4,0) - x(1,0)) / (std::pow(x(4,0) - x(1,0), 2) / std::pow(x(3,0) - x(0,0), 2) + 1) * std::pow(x(3,0) - x(0,0), 2);
    H(0,1) = -1 / (std::pow(x(4,0) - x(1,0), 2) / std::pow(x(3,0) - x(0,0), 2) + 1) * std::pow(x(3,0) - x(0,0), 2);
    H(0,2) = -1;
    H(0,3) = -H(0,0);
    H(0,4) = -H(0,1);
    H(1,0) = (x(0,0) - x(3,0)) / std::sqrt(std::pow(x(0,0) - x(3,0), 2) + std::pow(x(1,0) - x(4,0), 2));
    H(1,1) = (x(1,0) - x(4,0)) / std::sqrt(std::pow(x(0,0) - x(3,0), 2) + std::pow(x(1,0) - x(4,0), 2));
    H(1,3) = -H(1,0);
    H(1,4) = -H(1,1);
    H(2,0) = -(x(1,0) - x(4,0)) / (std::pow(x(1,0) - x(4,0), 2) / std::pow(x(0,0) - x(3,0), 2) + 1) * std::pow(x(0,0) - x(3,0), 2);
    H(2,1) = 1 / (std::pow(x(1,0) - x(4,0), 2) / std::pow(x(0,0) - x(3,0), 2) + 1) * std::pow(x(0,0) - x(3,0), 2);
    H(2,3) = -H(2,0);
    H(2,4) = -H(2,1);
    H(2,5) = -1;
    H(3,3) = 1;
    H(4,4) = 1;

    return H;
}


}