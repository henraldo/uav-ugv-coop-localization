#include "constants.hpp"
#include "data/export_utils.hpp"
#include <fstream>
#include <filesystem>
#include <string>
#include <iostream>

namespace uav_ugv_sim {

void TimeHistoryCollector::Reserve(size_t max_steps) {

    time_col_ = kTimeHeader.size();
    state_col_ = kStateHeaders.size();
    meas_col_ = kMeasurementHeaders.size();
    est_col_ = kEstHeaders.size();
    err_col_ = kErrorHeaders.size();
    covar_col_ = kCovHeaders.size();
    total_cols_ = time_col_ + state_col_ + meas_col_ + est_col_ + err_col_ + covar_col_;

    data_.resize(max_steps, total_cols_);
    filter_settings_.resize(3, covar_col_);

    // Build vectors for column and row headers
    headers_.clear();
    headers_.insert(headers_.end(), kTimeHeader.begin(), kTimeHeader.end());
    headers_.insert(headers_.end(), kStateHeaders.begin(), kStateHeaders.end());
    headers_.insert(headers_.end(), kMeasurementHeaders.begin(), kMeasurementHeaders.end());
    headers_.insert(headers_.end(), kEstHeaders.begin(), kEstHeaders.end());
    headers_.insert(headers_.end(), kErrorHeaders.begin(), kErrorHeaders.end());
    headers_.insert(headers_.end(), kCovHeaders.begin(), kCovHeaders.end());

    filter_settings_headers_.clear();
    filter_settings_headers_ = kCovarDiagRowLabels;

    row_ = 0;
}

void TimeHistoryCollector::Record(
    double t,
    const SystemState& x_true,
    const ObservationState& y_true,
    const SystemState& xhat,
    const ObservationState& ey,
    const Eigen::Matrix<double, 6, 1> P_diag
) {
    if (row_ >= data_.rows()) {
        data_.conservativeResize(data_.rows() * 2, Eigen::NoChange);
    }

    auto current_row = data_.row(row_);

    current_row.segment(0, time_col_) = Eigen::VectorXd::Constant(time_col_, t);
    current_row.segment(time_col_, state_col_) = x_true.transpose();
    current_row.segment(time_col_ + state_col_, meas_col_) = y_true.transpose();
    current_row.segment(time_col_ + state_col_ + meas_col_, est_col_) = xhat.transpose();
    current_row.segment(time_col_ + state_col_ + meas_col_ + est_col_, err_col_) = ey.transpose();
    current_row.segment(time_col_ + state_col_ + meas_col_ + est_col_ + err_col_, covar_col_) = P_diag.transpose();
    ++row_;
}

void TimeHistoryCollector::Save(
    const std::string& dataset_name,
    const EstimatorType& filter_type
) const {
    if (row_ == 0) return;

    // Build full path to output directory
    std::string output_dir = "simulation_output";
    std::filesystem::path base_directory = std::filesystem::current_path();
    std::filesystem::path output_path = base_directory / output_dir / dataset_name;
    if (!std::filesystem::exists(output_path)) {
        if (!std::filesystem::create_directories(output_path)) {
            std::cout << "Error: Could not create directory " << output_path << std::endl;
            throw std::runtime_error("Failed to create directory");
        }
        std::cout << "Created output directory: " << output_path << std::endl;
    }

    std::string filter;
    if (filter_type == EstimatorType::EKF) {
        filter = "ekf_";
    } else {
        filter = "ukf_";
    }

    SimHistory final_data = data_.topRows(row_);
    std::filesystem::path sim_data = output_path.append(filter + "simulation_data.csv");
    std::ofstream sim_csv_file(sim_data);
    if (!sim_csv_file.is_open()) throw std::runtime_error("Cannot open simulation_data.csv");

    // Write headers
    for (size_t i = 0; i < headers_.size() - 1; i++) {
        sim_csv_file << headers_[i] << ",";
    }
    sim_csv_file << headers_.back() << std::endl;

    // Write data rows
    for (int i = 0; i < final_data.rows(); i++) {
        for (int j = 0; j < final_data.cols() - 1; j++) {
            sim_csv_file << final_data(i, j) << ",";
        }
        sim_csv_file << final_data(i, final_data.cols() - 1) << std::endl;
    }
    sim_csv_file.flush();
    sim_csv_file.close();
    std::cout << "Exported simulation data to " << sim_data << std::endl;
}

void TimeHistoryCollector::SaveFilterSettings(
    const std::string& dataset_name,
    const FilterParams& filter_params
) const {
    // Build full path to output directory
    std::string output_dir = "simulation_output";
    std::filesystem::path base_directory = std::filesystem::current_path();
    std::filesystem::path output_path = base_directory / output_dir / dataset_name;
    if (!std::filesystem::exists(output_path)) {
        if (!std::filesystem::create_directories(output_path)) {
            std::cout << "Error: Could not create directory " << output_path << std::endl;
            throw std::runtime_error("Failed to create directory");
        }
        std::cout << "Created output directory: " << output_path << std::endl;
    }

    // Write out R, Q, and initial P covar matrix diagonals
    std::filesystem::path covar_data = output_path.append("filter_settings.csv");
    std::ofstream covar_csv_file(covar_data);
    if (!covar_csv_file.is_open()) throw std::runtime_error("Cannot open filter_settings.csv");

    // Write row headers and diagonals
    std::vector<Eigen::VectorXd> filter_covar{};
    filter_covar.push_back(filter_params.R.diagonal());
    filter_covar.push_back(filter_params.Q.diagonal());
    filter_covar.push_back(filter_params.P0.diagonal());

    for (size_t i = 0; i < filter_settings_headers_.size(); i++) {
        covar_csv_file << filter_settings_headers_[i];

        for (int j = 0; j < filter_covar[i].rows(); j++) {
            covar_csv_file << "," << filter_covar[i](j);
        }
        covar_csv_file << std::endl;
    }
    covar_csv_file.flush();
    covar_csv_file.close();
    std::cout << "Exported covar matrix diagonal data to " << covar_data << std::endl;
}

}