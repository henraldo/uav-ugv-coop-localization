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

// void exportSimDataToCSV(
//     const std::string& dataset_name,
//     const std::vector<double>& time_history,
//     const std::vector<double>& covar_diagonals,
//     const FilterType& filter_type
// ) {
//     if (time_history.empty() || covar_diagonals.empty()) {
//         std::cerr << "Error: Empty vector received for time_history or covariance diagonals." << std::endl;
//         return;
//     }

//     // Build full path to output directory
//     std::string output_dir = "simulation_output";
//     std::filesystem::path current_path = std::filesystem::current_path();
//     std::filesystem::path base_directory = std::filesystem::path::parent_path(current_path);
//     std::filesystem::path output_path = base_directory / output_dir / dataset_name;
//     if (!std::filesystem::exists(output_path)) {
//         if (!std::filesystem::create_directories(output_path)) {
//             std::cerr << "Error: Could not create directory " << output_path << std::endl;
//             return;
//         }
//         std::cout << "Created output directory: " << output_path << std::endl;
//     }

//     std::string filter;
//     if (filter_type == FilterType::EKF) {
//         filter = "ekf_";
//     } else {
//         filter = "ukf_";
//     }

//     std::filesystem::path sim_data = output_path.append("simulation_data.csv");
//     std::filesystem::path covar_data = output_path.append(filter + "covar_diagonals.csv");

//     std::ofstream sim_csv_file(sim_data);
//     // std::ofstream covar_csv_file(covar_data);
//     if (!sim_csv_file.is_open()) {
//         std::cerr << "Error: Could not open " << sim_data << " for writing." << std::endl;
//         return;
//     }

//     // Write headers
//     for (int i = 0; i < SimDataHeaders.size(); i++) {
//         sim_csv_file << SimDataHeaders[i] << ", ";
//     }
//     sim_csv_file << std::endl;

//     // Write data to rows
//     // for (size_t step = 0; step < time_history[0].size(); step++) {
//     //     sim_csv_file << time_history[step];
//     //     for (int dimX = 0; dimX < time_history[step].size(); dimX++) {
//     //         sim_csv_file << "," << state_history[step](dimX);
//     //     }
//     //     sim_csv_file << std::endl;
//     // }

//     sim_csv_file.close();
//     std::cout << "Exported simulated trajectory and sensor data to " << sim_data << std::endl;
// };

}