#pragma once

#include "../constants.hpp"
#include "../filters/estimator.hpp"
#include <Eigen/Dense>
#include <string>
#include <vector>

namespace uav_ugv_sim {

inline const std::vector<std::string> kCovarDiagRowLabels = {"R", "Q", "P_0"};

inline const std::vector<std::string> kTimeHeader = {"time"};
inline const std::vector<std::string> kStateHeaders = {"x_eg", "x_ng", "x_wg", "x_ea", "x_na","x_wa"};
inline const std::vector<std::string> kMeasurementHeaders = {"y_0", "y_1", "y_2", "y_3", "y_4"};
inline const std::vector<std::string> kEstHeaders = {"xhat_eg", "xhat_ng", "xhat_wg", "xhat_ea", "xhat_na","xhat_wa"};
inline const std::vector<std::string> kErrorHeaders = {"ey_0", "ey_1", "ey_2", "ey_3", "ey_4"};
inline const std::vector<std::string> kCovHeaders = {"p_00", "p_11", "p_22", "p_33", "p_44", "p_55"};
inline const std::vector<std::string> kMonteCarloRunIdx = {"run_idx"};
inline const std::vector<std::string> kNeesHeader = {"nees"};
inline const std::vector<std::string> kNisHeader = {"nis"};


enum class CollectorMode { Full, MCStatistics };


class TimeHistoryCollector {
    private:
        CollectorMode mode_ = CollectorMode::Full;

        SimHistory data_;
        SimHistory filter_settings_;
        std::vector<std::string> headers_;
        std::vector<std::string> filter_settings_headers_;
        size_t time_col_{0}, state_col_{0}, meas_col_{0}, est_col_{0}, err_col_{0}, covar_col_{0};
        size_t total_cols_{0};
        int row_{0};

        std::vector<double> mc_times_;
        std::vector<double> mc_nees_;
        std::vector<double> mc_nis_;

    public:
        void SetMode(CollectorMode mode) { mode_ = mode; }

        CollectorMode GetMode() { return mode_; }

        void Reserve(size_t max_steps);

        void Record(
            double t,
            const SystemState& x_true,
            const ObservationState& y_true,
            const SystemState& xhat,
            const ObservationState& ey,
            const Eigen::Matrix<double, 6, 1> P_diag,
            double nees,
            double nis
        );

        void Save(
            const std::string& dataset_name,
            const EstimatorType& filter_type
        ) const;

        void SaveMCStats(
            const std::vector<TimeHistoryCollector>& collectors,
            const EstimatorType& filter_type
        ) const;

        void SaveFilterSettings(
            const std::string& dataset_name,
            const FilterParams& filter_params
        ) const;
};

}