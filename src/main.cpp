#include "constants.hpp"
#include "system/dynamics.hpp"
#include "filters/estimator.hpp"
#include "filters/ekf.hpp"
#include "filters/ukf.hpp"
#include "data/export_utils.hpp"
#include <iostream>
#include <iomanip>
#include <memory>
#include <thread>

using namespace uav_ugv_sim;

TimeHistoryCollector RunSingleSimulation(
    int run_id,
    TruthParams truth_params,
    SystemParams sys_params,
    FilterParams filter_params,
    bool is_mc
) {
    // Setup Simulation
    const size_t max_steps = 5000;
    const double total_time = max_steps * DT;
    StateCov Q = truth_params.QTrue;
    MeasCov R = truth_params.RTrue;
    SystemModel model(sys_params.x0, Q, R);

    // Set estimator type to implement
    std::unique_ptr<Estimator> filter;
    std::string sim_name;
    if (filter_params.filter_type == EstimatorType::EKF) {
        filter = std::make_unique<EKF>(sys_params.x0, filter_params);
        sim_name = is_mc ? "ekf_mc_tmt_" + std::to_string(run_id) : "ekf_run";
    } else {
        filter = std::make_unique<UKF>(sys_params.x0, filter_params, 0.001, 2.0, 0); // <-- alpha/beta/kappa
        sim_name = is_mc ? "ukf_mc_tmt_" + std::to_string(run_id) : "ukf_run";
    }

    // Set up data recorder
    TimeHistoryCollector collector;
    collector.Reserve(max_steps);
    if (is_mc) collector.SetMode(CollectorMode::MCStatistics);

    double t = 0.0;
    auto [times, x_truth, y_truth] = model.GenerateGroundTruthData(total_time, sys_params.u0);

    // Record initial states
    collector.Record(
        t,
        x_truth.col(0),
        y_truth.col(0),
        filter->GetEstimatedState(),
        filter->GetFilterResiduals(),
        filter->GetCovarDiagonal(),
        0.0,
        0.0
    );

    for (size_t k = 0; k < max_steps - 1; k++) {

        // Make prediction
        filter->Predict(t, sys_params.u0);
        t += DT;

        // Correct prediction from latest measurement
        SystemState state = x_truth.col(k+1);
        ObservationState z = y_truth.col(k+1);
        filter->Correct(z);

        auto estimate = filter->GetEstimatedState();
        auto residuals = filter->GetFilterResiduals();
        auto covar_diag = filter->GetCovarDiagonal();
        auto covar = filter->GetCovariance();
        auto innov_covar = filter->GetInnovCovariance();

        SystemState est_err = state - estimate;
        est_err(2) = WrapToPi(est_err(2));
        est_err(5) = WrapToPi(est_err(5));
        double nees = est_err.transpose() * covar.inverse() * est_err;
        double nis = residuals.transpose() * innov_covar.inverse() * residuals;

        collector.Record(t, state, z, estimate, residuals, covar_diag, nees, nis);
    }

    if (!is_mc) {
        collector.Save(sim_name, filter_params.filter_type);
        collector.SaveFilterSettings(sim_name, filter_params);
    }
    std::cout << "[INFO] " << sim_name << " completed" << std::endl;

    return collector;
}

int main(int argc, char* argv[]) {

    size_t num_mc_runs = 1;
    EstimatorType filter_type = EstimatorType::EKF;
    std::string filter_name = "ekf";

    if (argc >= 2) {
        std::string arg1 = argv[1];
        if (arg1 == "ukf" || arg1 == "UKF") {
            filter_type = EstimatorType::UKF;
            filter_name = "ukf";
            if (argc >= 3) num_mc_runs = std::stoi(argv[2]);
        } else if (arg1 == "ekf" || arg1 == "EKF") {
            filter_type = EstimatorType::EKF;
            if (argc >= 3) num_mc_runs = std::stoi(argv[2]);
        } else {
            // Assume first arg is number of MC runs (EKF default)
            num_mc_runs = std::stoi(arg1);
        }
    }

    // Define initial states and covariance parameters for Ground Truth generation
    TruthParams truth_params{};
    SystemParams sys_params{};

    // Define covariance traces for estimator (sample values here - update as desired)
    double p_00 = 100.0, p_11 = 100.0, p_22 = 0.8;
    double p_33 = 150.0, p_44 = 150.0, p_55 = 1.2;
    StateCov P_0 = Eigen::Matrix<double, 6, 1>(p_00, p_11, p_22, p_33, p_44, p_55).asDiagonal();

    double q_00 = 0.001, q_11 = 0.001, q_22 = 0.01;
    double q_33 = 0.001, q_44 = 0.001, q_55 = 0.01;
    StateCov Q_est = Eigen::Matrix<double, 6, 1>(q_00, q_11, q_22, q_33, q_44, q_55).asDiagonal();

    double r_00 = 0.0225, r_11 = 64.0, r_22 = 0.04, r_33 = 36.0, r_44 = 36.0;
    MeasCov R_est = Eigen::Matrix<double, 5, 1>(r_00, r_11, r_22, r_33, r_44).asDiagonal();

    FilterParams filter_params(P_0, Q_est, R_est, filter_type);

    if (num_mc_runs == 1) {
        std::cout << "[INFO] Beginning single simulation..." << std::endl;
        RunSingleSimulation(0, truth_params, sys_params, filter_params, false);
    } else {
        std::cout << "[INFO] Beginning " << num_mc_runs << " Monte-Carlo  Truth Model Testing runs..." << std::endl;

        TimeHistoryCollector mc_collector;
        mc_collector.SetMode(CollectorMode::MCStatistics);

        std::vector<std::thread> threads;
        std::vector<TimeHistoryCollector> collectors(num_mc_runs);
        threads.reserve(num_mc_runs);

        for (size_t i = 0; i < num_mc_runs; i++) {
            int run_id = i + 1;
            threads.emplace_back(
                [&collectors, i, run_id, truth_params, sys_params, filter_params]() {
                    collectors[i] = RunSingleSimulation(run_id, truth_params, sys_params, filter_params, true);
            });
        }

        // Wait for threads to finish
        for (auto& sim_thread : threads) {
            if (sim_thread.joinable()) sim_thread.join();
        }

        mc_collector.SaveMCStats(collectors, filter_type);
        std::cout << "[INFO] All " << num_mc_runs << " Monte-Carlo runs completed & stats consolidated for postprocessing" << std::endl;

        std::string dataset_name = filter_name + "_mc_tmt_settings";
        mc_collector.SaveFilterSettings(dataset_name, filter_params);
    }

    return 0;
};
