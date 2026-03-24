# UAV-UGV Cooperative Localization
This project generates a user-configurable simulation of cooperative localization between a UAV and UGV, using either an Extended Kalman Filter (EKF), or an Unscented Kalman Filter (UKF). The project
can also be used as a testbed for EKF or UKF performance and stability tuning via user-selectable
Truth Model Testing (TMT) - where Monte Carlo trials are run to collect NEES and NIS statistics for assessing dynamic performance of the implemented filter.

Time history data generated from the simulation (along with filter implementation settings) are written to CSV files and placed in a user-defined subdirectory under `/project-root/simulation_output/` for post-processing and analysis.

# Simulation Overview
<img width="652" height="239" alt="system_diagram" src="https://github.com/user-attachments/assets/5b0029d4-67ff-4e23-aa43-b61e219730c7" />

## State-Space System Definition
The combined nonlinear state-space system has the form:

$$\boldsymbol{\dot{x}} (t) = \boldsymbol{f}(x, u) + \boldsymbol{\Gamma} \boldsymbol{\tilde{w}} (t)$$
$$\boldsymbol{y} (t) = \boldsymbol{h}(x) + \boldsymbol{\tilde{v}} (t)$$

### UGV Model
The equations of motion used to define the UGV dynamics model are as follows:

$$\dot{\xi}_g = v_g cos\theta_g + \tilde{w}_{x,g}$$

$$\dot{\eta}_g = v_g sin\theta_g + \tilde{w}_{y,g}$$

$$\dot{\theta}_g = \frac{v_g}{L}tan\phi_g + \tilde{w}_{\omega,g}$$

with control inputs

$$\boldsymbol{u}_g (t) = \begin{bmatrix}
    v_g\\
    \phi_g\\
\end{bmatrix}
$$

and AWG process noise

$$\boldsymbol{\tilde{w}}_g (t) = \begin{bmatrix}
    \tilde{w}_{x,g}\\
    \tilde{w}_{y,g}\\
    \tilde{w}_{\omega,g}\\
\end{bmatrix}
$$

Here $L$ is the wheel base length of the UGV, and velocity and steering angle control inputs are limited to

$$v_{g, max} = 3 \ m/s$$

and

$$\phi_g \ \epsilon \ [-\frac{5\pi}{12},\frac{5\pi}{12}]\ rad$$

### UAV Model
The equations of motion used to define the UAV dynamics model are as follows:

$$\dot{\xi}_a = v_a cos\theta_a + \tilde{w}_{x,a}$$

$$\dot{\eta}_a = v_a sin\theta_a + \tilde{w}_{y,a}$$

$$\dot{\theta}_a = \omega_a + \tilde{w}_{\omega,a}$$

with control inputs

$$\boldsymbol{u}_a (t) = \begin{bmatrix}
    v_a\\
    \omega_a\\
\end{bmatrix}
$$

and AWG process noise

$$\boldsymbol{\tilde{w}}_a (t) = \begin{bmatrix}
    \tilde{w}_{x,a}\\
    \tilde{w}_{y,a}\\
    \tilde{w}_{\omega,a}\\
\end{bmatrix}
$$

The UAV velocity and steering rate inputs are limited to

$$v_a \ \epsilon \ [10, 20] \ m/s$$

and

$$\omega_a \  \epsilon \ [-\frac{\pi}{6},\frac{\pi}{6}] \ rad/s$$

### Combined System Model
The combined system state, control input, and disturbance input vectors are defined as

$$\boldsymbol{x} (t) = \begin{bmatrix}
    \xi_g\\
    \eta_g\\
    \theta_g\\
    \xi_a\\
    \eta_a\\
    \theta_a\\
\end{bmatrix}
$$

$$\boldsymbol{u} (t) = \begin{bmatrix}
    \boldsymbol{u}_g\\
    \boldsymbol{u}_a\\
\end{bmatrix}
$$

$$\boldsymbol{\tilde{w}} (t) = \begin{bmatrix}
    \boldsymbol{\tilde{w}}_g\\
    \boldsymbol{\tilde{w}}_a\\
\end{bmatrix}
$$

and the combined nonlinear state-space system is defined as

$$\boldsymbol{\dot{x}} (t) = \begin{bmatrix}
    u_1 cos x_3 \\
    u_1 sin x_3 \\
    \frac{u_1}{L} tan u_2 \\
    u_3 cos x_6 \\
    u_3 sin x_6 \\
    u_4 \\
\end{bmatrix} + \boldsymbol{\Gamma} \boldsymbol{\tilde{w}} (t)
$$

Lastly, the combined system's sensing model is defined as

$$\boldsymbol{y} (t) = \begin{bmatrix}
    tan^{-1} \frac{x_5-x_2}{x_4-x_1} - x_3\\
    \sqrt{(x_1 - x_4)^{2} + (x_2 - x_5)^{2}}\\
    tan^{-1} \frac{x_2-x_5}{x_1-x_4} - x_6\\
    x_4\\
    x_5\\
\end{bmatrix} + \boldsymbol{\tilde{v}} (t)
$$

## Project Build
This project was developed and tested using GCC-15/G++15 for compiling, therefore it is recommended
to use GCC and G++ for generating build files and compiling. If you prefer to use a different compiler,
you may need to make adjustments to `CMakeLists.txt` to build the project.

### Building
Clone down the project from Github and navigate to the project root in your terminal of choice. Next,
run the following command (ommit `-G Ninja` if desired):
```bash
cmake -B build -S . -G Ninja -DCMAKE_BUILD_TYPE=Release
```

If GCC is not your default compiler:
```bash
cmake -B build -S . -G Ninja -DCMAKE_CXX_COMPILER=$(which g++-15) -DCMAKE_C_COMPILER=$(which gcc-15) -DCMAKE_BUILD_TYPE=Release
```

Once generation of the build files has completed, run the following command to generate the executable:
```bash
cmake --build build
```

### Running
Upon successful completion of building the build process, the program is run by executing the following
command:
```bash
./build/uav_ugv_coop_localization
```
or for Windows users
```bash
./build/uav_ugv_coop_localization.exe
```
