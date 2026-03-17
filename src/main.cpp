#include "constants.hpp"
#include "system/dynamics.hpp"
#include "filters/ekf.hpp"
#include "data/export_utils.hpp"
#include <iostream>
#include <iomanip>

using namespace uav_ugv_sim;

std::vector<std::string> kCovarDiagRowLabels = {"R", "Q", "P_0"};
std::vector<std::string> kStateHeaders = {"x_eg", "x_en", "x_ew", "x_ag", "x_an","x_aw"};
std::vector<std::string> kMeasurementHeaders = {"y_0", "y_1", "y_2", "y_3", "y_4"};
std::vector<std::string> kEstHeaders = {"xhat_eg", "xhat_en", "xhat_ew", "xhat_ag", "xhat_an","xhat_aw"};
std::vector<std::string> kErrorHeaders = {"ey_0", "ey_1", "ey_2", "ey_3", "ey_4"};
std::vector<std::string> kCovHeaders = {"p_00", "p_11", "p_22", "p_33", "p_44", "p_55"};

int main() {

    const size_t max_steps = 100;

    TimeHistoryCollector collector;
    collector.Reserve(
        max_steps,
        kStateHeaders,
        kMeasurementHeaders,
        kEstHeaders,
        kErrorHeaders,
        kCovHeaders,
        kCovarDiagRowLabels
    );

    TruthParams truth_data{};
    StateCov Q = truth_data.QTrue;
    MeasCov R = truth_data.RTrue;

    SystemParams sys_params{};
    SystemModel model(sys_params.x0, Q, R, sys_params);

    EkfParams ekf_params{};
    EKF filter(sys_params.x0, ekf_params);

    double t = 0.0;
    for (size_t k = 0; k < max_steps; k++) {
        model.propagate(t, sys_params.u0);
        model.collectMeasurements();

        auto state = model.getState();
        auto z = model.getSensorMeasurement();

        filter.predict(t, sys_params.u0);
        filter.correct(z);

        auto estimate = filter.getEstimatedState();
        auto residuals = filter.getFilterResiduals();
        auto covar_diag = filter.getCovarDiagonal();

        collector.Record(t, state, z, estimate, residuals, covar_diag);
        t += DT;
    }

    collector.Save("test_run", FilterType::EKF, ekf_params);


    return 0;
};
