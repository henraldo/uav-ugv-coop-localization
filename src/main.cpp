#include "constants.hpp"
#include "system/dynamics.hpp"
#include "filters/ekf.hpp"
#include "data/export_utils.hpp"
#include <iostream>
#include <iomanip>

using namespace uav_ugv_sim;

int main() {

    TruthParams truth_data{};
    StateCov Q = truth_data.QTrue;
    MeasCov R = truth_data.RTrue;

    SystemParams sys_params{};
    SystemModel model(sys_params.x0, Q, R, sys_params);

    EkfParams ekf_params{};
    EKF filter(sys_params.x0, ekf_params);

    double t0 = 0.0;
    model.propagate(t0, sys_params.u0);
    model.collectMeasurements();

    auto state = model.getState();
    auto z = model.getSensorMeasurement();

    filter.predict(t0, sys_params.u0);
    filter.correct(z);

    auto estimate = filter.getEstimatedState();
    auto residuals = filter.getFilterResiduals();
    auto covar_diag = filter.getCovarDiagonal();

    std::cout << "X: [" << std::endl;
    for (int i = 0; i < state.rows(); i++) {
        std::cout << "  " << state(i,0) << "," << std::endl;
    }
    std::cout << "]\n" << std::endl;

    std::cout << "Z: [" << std::endl;
    for (int j = 0; j < z.rows(); j++) {
        std::cout << "  " << z(j,0) << "," << std::endl;
    }
    std::cout << "]" << std::endl;

    std::cout << "X_HAT: [" << std::endl;
    for (int j = 0; j < estimate.rows(); j++) {
        std::cout << "  " << estimate(j,0) << "," << std::endl;
    }
    std::cout << "]" << std::endl;

    std::cout << "Y_HAT: [" << std::endl;
    for (int j = 0; j < residuals.rows(); j++) {
        std::cout << "  " << residuals(j,0) << "," << std::endl;
    }
    std::cout << "]" << std::endl;

    std::cout << "P: [" << std::endl;
    for (int j = 0; j < covar_diag.rows(); j++) {
        std::cout << "  " << covar_diag(j,0) << "," << std::endl;
    }
    std::cout << "]" << std::endl;

    return 0;
};
