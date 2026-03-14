#include <iostream>
#include "constants.hpp"
#include <cmath>
#include <iomanip>

int main() {
    using namespace uav_ugv_sim;

    TruthParams truth_data {};
    StateCov Q = truth_data.QTrue;

    std::cout << "Q: [" << std::endl;
    for (int i = 0; i < Q.rows(); i++) {
        std::cout << Q(i,0) << ", " << Q(i,1) << ", " << Q(i,2) << ", " << Q(i,3) << ", " << Q(i,4) << ", " << Q(i,5) << std::endl;
    }
    std::cout << "]" << std::endl;

    return 0;
};
