#include "constants.hpp"
#include "data/export_utils.hpp"
#include <fstream>
#include <filesystem>
#include <string>
#include <iostream>

namespace uav_ugv_sim {

void exportSimDataToCSV(
    const std::string& dataset_name,
    const std::vector<std::vector<double>>& sim_history,
    const std::vector<std::vector<double>>& covar_diagonals,
    const FilterType& filter_type
) {
    if (sim_history.empty() || covar_diagonals.empty()) {
        std::cerr << "Error: Empty vector received for time_history or covariance diagonals." << std::endl;
        return;
    }

    // Build full path to output directory
    std::string output_dir = "simulation_output";
    std::filesystem::path current_path = std::filesystem::current_path();
    std::filesystem::path base_directory = current_path.parent_path();
    std::filesystem::path output_path = base_directory / output_dir / dataset_name;
    if (!std::filesystem::exists(output_path)) {
        if (!std::filesystem::create_directories(output_path)) {
            std::cerr << "Error: Could not create directory " << output_path << std::endl;
            return;
        }
        std::cout << "Created output directory: " << output_path << std::endl;
    }

    std::string filter;
    if (filter_type == FilterType::EKF) {
        filter = "ekf_";
    } else {
        filter = "ukf_";
    }

    std::filesystem::path sim_data = output_path.append(filter + "simulation_data.csv");
    std::ofstream sim_csv_file(sim_data);
    if (!sim_csv_file.is_open()) {
        std::cerr << "Error: Could not open " << sim_data << " for writing." << std::endl;
        return;
    }

    // Write headers
    for (size_t i = 0; i < SimDataHeaders.size(); i++) {
        sim_csv_file << SimDataHeaders[i] << ", ";
    }
    sim_csv_file << std::endl;

    // Write data to rows
    for (size_t step = 0; step < sim_history[0].size(); step++) {
        for (size_t j = 0; j < sim_history.size(); j++) {
            sim_csv_file << "," << sim_history[step][j];
        }
        sim_csv_file << std::endl;
    }
    sim_csv_file.close();
    std::cout << "Exported simulation data to " << sim_data << std::endl;

    // Write out R, Q, and initial P covar matrix diagonals
    std::filesystem::path covar_data = output_path.append("covar_diagonals.csv");
    std::ofstream covar_csv_file(covar_data);
    if (!covar_csv_file.is_open()) {
        std::cerr << "Error: Could not open " << sim_data << " for writing." << std::endl;
        return;
    }

    // Write row headers and diagonals
    for (size_t i = 0; i < CovarDiagRowLabels.size(); i++) {
        covar_csv_file << CovarDiagRowLabels[i];

        for (size_t j = 0; j < covar_diagonals.size(); j++) {
            covar_csv_file << "," << covar_diagonals[i][j];
        }
        covar_csv_file << std::endl;
    }
    covar_csv_file.close();
    std::cout << "Exported covar matrix diagonal data to " << covar_data << std::endl;
};

}