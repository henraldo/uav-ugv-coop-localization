# uav-ugv-coop-localization
Simulation of cooperative localization between a UAV and UGV using/comparing an Extended Kalman Filter (EKF) and Unscented Kalman Filter (UKF)

# Project Setup

## Building

```bash
cmake -B build -S . -G Ninja -DCMAKE_BUILD_TYPE=Release
```
or
```bash
cmake -B build -S . -G Ninja -DCMAKE_CXX_COMPILER=$(which g++-15) -DCMAKE_C_COMPILER=$(which gcc-15) -DCMAKE_BUILD_TYPE=Release
```
if GCC is not your default compiler.

```bash
cmake --build build
```
