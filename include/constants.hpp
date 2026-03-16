#pragma once

#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace uav_ugv_sim {

// simulation timestep/sample-rate size
const double DT = 0.1;
constexpr double PI = EIGEN_PI;
const int TRAJECT_AND_OBS_SIZE = 12; // timestamps + 6 internal states + 5 observed states

// Filter types for Cooperative Localization Simulations
enum class FilterType {EKF, UKF};

// type alias for x, F, G, u
using SystemState = Eigen::Matrix<double, 6, 1>;
using StateTransition = Eigen::Matrix<double, 6, 6>;
using ControlMatrix = Eigen::Matrix<double, 6, 4>;
using ControlInput = Eigen::Matrix<double, 4, 1>;

// type alias for y, H
using ObsSensativity = Eigen::Matrix<double, 5, 6>;
using ObservationState = Eigen::Matrix<double, 5, 1>;

// type alias for Q/P, R
using StateCov = Eigen::Matrix<double, 6, 6>;
using MeasCov = Eigen::Matrix<double, 5, 5>;

using TrajectAndObsHist = Eigen::Vector<double, TRAJECT_AND_OBS_SIZE>;

struct TruthParams {
    StateCov QTrue = Eigen::Matrix<double, 6, 1>(0.001, 0.001, 0.01, 0.001, 0.001, 0.01).asDiagonal();
    MeasCov RTrue = Eigen::Matrix<double, 5, 1>(0.0225, 64.0, 0.04, 36.0, 36.0).asDiagonal();

    TruthParams() = default;
};

std::vector<std::string> SimDataHeaders = {
    "time", 
    "x_eg", 
    "x_en", 
    "x_ew", 
    "x_ag", 
    "x_an", 
    "x_aw", 
    "y_0", 
    "y_1", 
    "y_2", 
    "y_3", 
    "y_4",
    "xhat_eg", 
    "xhat_en", 
    "xhat_ew", 
    "xhat_ag", 
    "xhat_an", 
    "xhat_aw",
    "ey_0",
    "ey_1",
    "ey_2",
    "ey_3",
    "ey_4",
};
std::vector<std::string> CovarDiagRowLabels = {"R", "P", "Q"};

}