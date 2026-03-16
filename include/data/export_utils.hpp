#pragma once

#include "../constants.hpp"
#include <string>
#include <vector>
#include <array>

namespace uav_ugv_sim {

inline constexpr std::array<std::string_view, 29> SimDataHeaders = {
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
    "p_00",
    "p_11",
    "p_22",
    "p_33",
    "p_44",
    "p_55"
};
inline constexpr std::array<std::string_view, 3> CovarDiagRowLabels = {"R", "Q", "P_0"};

void exportSimDataToCSV(
    const std::string& dataset_name,
    const std::vector<std::vector<double>>& sim_history,
    const std::vector<std::vector<double>>& covar_diagonals,
    const FilterType& filter_type
);

}