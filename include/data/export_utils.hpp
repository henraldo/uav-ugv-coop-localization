#pragma once

#include "../constants.hpp"
#include "../filters/estimator.hpp"
#include <Eigen/Dense>
#include <string>
#include <vector>

namespace uav_ugv_sim {

inline const std::vector<std::string> kCovarDiagRowLabels = {"R", "Q", "P_0"};

inline const std::vector<std::string> kTimeHeader = {"time"};
inline const std::vector<std::string> kStateHeaders = {"x_eg", "x_en", "x_ew", "x_ag", "x_an","x_aw"};
inline const std::vector<std::string> kMeasurementHeaders = {"y_0", "y_1", "y_2", "y_3", "y_4"};
inline const std::vector<std::string> kEstHeaders = {"xhat_eg", "xhat_en", "xhat_ew", "xhat_ag", "xhat_an","xhat_aw"};
inline const std::vector<std::string> kErrorHeaders = {"ey_0", "ey_1", "ey_2", "ey_3", "ey_4"};
inline const std::vector<std::string> kCovHeaders = {"p_00", "p_11", "p_22", "p_33", "p_44", "p_55"};


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
        void Reserve(size_t max_steps);

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
            const EstimatorType& filter_type
        ) const;

        void SaveFilterSettings(
            const std::string& dataset_name,
            const FilterParams& filter_params
        ) const;
};

}