#include "constants.hpp"
#include "system/dynamics.hpp"
#include <iostream>
#include <iomanip>

int main() {
    using namespace uav_ugv_sim;

    TruthParams truth_data {};
    StateCov Q = truth_data.QTrue;
    MeasCov R = truth_data.RTrue;

    SystemParams sys_params{};

    SystemModel model(sys_params.x0, Q, R, sys_params);

    // std::cout << "Q: [" << std::endl;
    // for (int i = 0; i < Q.rows(); i++) {
    //     std::cout << Q(i,0) << ", " << Q(i,1) << ", " << Q(i,2) << ", " << Q(i,3) << ", " << Q(i,4) << ", " << Q(i,5) << std::endl;
    // }
    // std::cout << "]" << std::endl;

    double t0 = 0.0;
    model.propagate(t0, sys_params.u0);
    model.collectMeasurements();

    SystemState state = model.getState();
    ObservationState z = model.getSensorMeasurement();

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


    return 0;
};
