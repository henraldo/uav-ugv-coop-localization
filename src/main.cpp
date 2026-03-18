#include "constants.hpp"
#include "system/dynamics.hpp"
#include "filters/estimator.hpp"
#include "filters/ekf.hpp"
#include "data/export_utils.hpp"
#include <iostream>
#include <iomanip>

using namespace uav_ugv_sim;

int main() {

    const size_t max_steps = 100;

    TimeHistoryCollector collector;
    collector.Reserve(max_steps);

    TruthParams truth_data{};
    StateCov Q = truth_data.QTrue;
    MeasCov R = truth_data.RTrue;

    SystemParams sys_params{};
    SystemModel model(sys_params.x0, Q, R, sys_params);

    FilterParams ekf_params{};
    EKF filter(sys_params.x0, ekf_params);
    collector.SaveFilterSettings("test_run", ekf_params);

    double t = 0.0;
    for (size_t k = 0; k < max_steps; k++) {
        model.Propagate(t, sys_params.u0);
        model.CollectMeasurements();

        auto state = model.GetState();
        auto z = model.GetSensorMeasurement();

        filter.Predict(t, sys_params.u0);
        filter.Correct(z);

        auto estimate = filter.GetEstimatedState();
        auto residuals = filter.GetFilterResiduals();
        auto covar_diag = filter.GetCovarDiagonal();

        collector.Record(t, state, z, estimate, residuals, covar_diag);
        t += DT;
    }

    collector.Save("test_run", EstimatorType::EKF);

    return 0;
};
