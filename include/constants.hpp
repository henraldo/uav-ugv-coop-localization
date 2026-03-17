#pragma once

#include <array>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>

namespace uav_ugv_sim {

// simulation timestep/sample-rate size
constexpr double DT = 0.1;
constexpr double PI = EIGEN_PI;

// Filter types for Cooperative Localization Simulations
enum class FilterType {EKF, UKF};

// UGV Wheelbase Length
constexpr double UGV_L = 0.5;

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

using SimHistory = Eigen::MatrixXd;

const StateCov Q_TRUE = Eigen::Matrix<double, 6, 1>(0.001, 0.001, 0.01, 0.001, 0.001, 0.01).asDiagonal();
const MeasCov R_TRUE = Eigen::Matrix<double, 5, 1>(0.0225, 64.0, 0.04, 36.0, 36.0).asDiagonal();

struct TruthParams {
    StateCov QTrue = Q_TRUE;
    MeasCov RTrue = R_TRUE;

    TruthParams() = default;
};

}