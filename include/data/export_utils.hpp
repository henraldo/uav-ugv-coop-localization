#pragma once

#include "../constants.hpp"
#include "../filters/ekf.hpp"
#include <Eigen/Dense>
#include <string>
#include <vector>

namespace uav_ugv_sim {

class TimeHistoryCollector {
    private:
        SimHistory data_;
        SimHistory filter_settings_;
        std::vector<std::string> headers_;
        std::vector<std::string> filter_settings_headers_;
        size_t time_col_{0}, state_col_{0}, meas_col_{0}, est_col_{0}, err_col_{0}, covar_col_{0};
        size_t total_cols_{0};
        int row_{0};

    public:
        void Reserve(
            size_t max_steps,
            const std::vector<std::string>& state_headers,
            const std::vector<std::string>& meas_headers,
            const std::vector<std::string>& est_headers,
            const std::vector<std::string>& err_headers,
            const std::vector<std::string>& covar_headers,
            const std::vector<std::string>& settings_headers
        );

        void Record(
            double t,
            const SystemState& x_true,
            const ObservationState& y_true,
            const SystemState& xhat,
            const ObservationState& ey,
            const Eigen::Matrix<double, 6, 1> P_diag
        );

        void Save(
            const std::string& dataset_name,
            const FilterType& filter_type,
            const EkfParams& filter_params
        ) const;
};

}