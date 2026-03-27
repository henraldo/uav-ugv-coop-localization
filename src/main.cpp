#include "constants.hpp"
#include "system/dynamics.hpp"
#include "filters/estimator.hpp"
#include "filters/ekf.hpp"
#include "filters/ukf.hpp"
#include "data/export_utils.hpp"
#include <iostream>
#include <iomanip>
#include <memory>

using namespace uav_ugv_sim;

int main() {

    const size_t max_steps = 1000;

    TimeHistoryCollector collector;
    collector.Reserve(max_steps);

    TruthParams truth_data{};
    StateCov Q = truth_data.QTrue;
    MeasCov R = truth_data.RTrue;

    // Setup Simulation
    SystemParams sys_params{};
    SystemModel model(sys_params.x0, Q, R);

    // Set estimator type to implement
    EstimatorType filter_type = EstimatorType::EKF;

    // Define covariance traces for estimator (sample values here - update as desired)
    StateCov P_0 = Eigen::Matrix<double, 6, 1>(5.0, 5.0, 20.0, 5.0, 5.0, 20.0).asDiagonal();
    StateCov Q_est = Q;
    MeasCov R_est = R;

    FilterParams filter_params(P_0, Q_est, R_est);
    std::string sim_name;
    std::unique_ptr<Estimator> filter;

    if (filter_type == EstimatorType::EKF) {
        filter = std::make_unique<EKF>(sys_params.x0, filter_params);
        sim_name = "ekf_run";
        collector.SaveFilterSettings(sim_name, filter_params);
    } else {
        filter = std::make_unique<UKF>(sys_params.x0, filter_params, 0.01, 2.0, 0); // <-- update alpha, beta, kappa to your desired values if UKF selected
        sim_name = "ukf_run";
        collector.SaveFilterSettings(sim_name, filter_params);
    }

    std::cout << "[INFO] Beginning simulation..." << std::endl;

    double t = 0.0;
    for (size_t k = 0; k < max_steps; k++) {
        // Simulate system model dynamics
        model.Propagate(t, sys_params.u0);
        model.CollectMeasurements();
        auto state = model.GetState();
        auto z = model.GetSensorMeasurement();

        // Filter update
        filter->Predict(t, sys_params.u0);
        filter->Correct(z);

        auto estimate = filter->GetEstimatedState();
        auto residuals = filter->GetFilterResiduals();
        auto covar_diag = filter->GetCovarDiagonal();
        collector.Record(t, state, z, estimate, residuals, covar_diag);

        t += DT;
    }
    std::cout << "[INFO] Simulation complete --> writing out results..." << std::endl;
    collector.Save(sim_name, filter_type);
    std::cout << "[INFO] Simulation results successfully written to disk" << std::endl;

    return 0;
};
