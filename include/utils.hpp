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

// void exportTrajectoryDataToCSV(
//     const std::string& filename,
//     const std::vector<double>& time_history,
//     const std::vector<SystemState>& state_history,
//     const std::vector<ObservationState>& sensor_history,
//     const std::string& output_dir = "output_data"
// ) {
//     if (time_history.size() != state_history.size() || time_history.size() != sensor_history.size() || time_history.empty()) {
//         std::cerr << "Error: Empty or mismatched history sizes." << std::endl;
//         return;
//     }

//     // Build full path to output directory
//     std::filesystem::path current_path = std::filesystem::current_path();
//     std::filesystem::path output_directory = std::filesystem::path::parent_path(current_path) / output_dir;
//     std::filesystem::path output_path = output_directory.append(filename);
//     if (!std::filesystem::exists(output_directory)) {
//         if (!std::filesystem::create_directories(output_directory)) {
//             std::cerr << "Error: Could not create directory " << output_directory << std::endl;
//             return;
//         }
//         std::cout << "Created output directory: " << output_directory << std::endl;
//     }

//     std::ofstream csv_file(output_path);
//     if (!csv_file.is_open()) {
//         std::cerr << "Error: Could not open " << output_path << " for writing." << std::endl;
//         return;
//     }

//     // Write header
//     csv_file << "time";
//     for (int i = 0; i < state_history[0].size(); i++) {
//         csv_file << ", x_" << i;
//     }
//     for (int j = 0; j < sensor_history[0].size(); j++) {
//         csv_file << ", y_" << j;
//     }
//     csv_file << std::endl;

//     // Write data to rows
//     for (size_t step = 0; step < time_history.size(); step++) {
//         csv_file << time_history[step];
//         for (int dimX = 0; dimX < state_history[step].size(); dimX++) {
//             csv_file << "," << state_history[step](dimX);
//         }
//         for (int dimY = 0; dimY < sensor_history[step].size(); dimY++) {
//             csv_file << "," << sensor_history[step](dimY);
//         }
//         csv_file << std::endl;
//     }

//     csv_file.close();
//     std::cout << "Exported simulated trajectory and sensor data to " << output_path << std::endl;
// };

}