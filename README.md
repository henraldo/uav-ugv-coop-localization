# UAV-UGV Cooperative Localization
This project generates a user-configurable simulation of cooperative localization between a UAV and UGV, using either an Extended Kalman Filter (EKF), or an Unscented Kalman Filter (UKF).

Future releases will also provide the means to tune EKF or UKF performance and stability via Monte Carlo
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

## Extended Kalman Filter Overview
The online nonlinear trajectory for the filter is updated at each time step, as part of the prediction
stage and the nonlinear dynamics model is propagated forward using BOOST's `odeint::runge_kutta_dopri5`
ODE solver. A new state estimate $\boldsymbol{\hat{x}}^{-}_{k+1} = \boldsymbol{f}(\boldsymbol{\hat{x}}^{+}_k , \boldsymbol{u}_k , \boldsymbol{w}_k = 0)$ is predicted by propagating the dynamics
model and the new filter covariance $\boldsymbol{{P}}^{-}_{k+1}$ is predicted as

$$\boldsymbol{P}^{-}_{k+1} = \boldsymbol{\tilde{F}}_k \boldsymbol{P}^{+}_k \boldsymbol{\tilde{F}}^{T}_k + \boldsymbol{\tilde{\Omega}}_k \boldsymbol{Q} \boldsymbol{\tilde{\Omega}}^{T}_k$$

where $\boldsymbol{\tilde{F}}_k$ is the discrete-time mapping of the Jacobian of the state transition
matrix at time $t_k$. This is updated prior to the new covariance prediction as

$$\boldsymbol{\tilde{F}}_k |_{\hat{x}^{+}_k, u_k, w_k = 0} = \boldsymbol{I} + \Delta\boldsymbol{T} \cdot \boldsymbol{\tilde{A}} |_{(\hat{x}^{+}_k, u(t_k),w(t_k)=0)}$$

where the state transition matrix Jacobian $\boldsymbol{\tilde{A}}$ is

$$\boldsymbol{\tilde{A}} = \begin{bmatrix}
    0 & 0 & -u_1 sin x_3 & 0 & 0 & 0 \\
    0 & 0 & u_1 cos x_3 & 0 & 0 & 0 \\
    0 & 0 & 0 & 0 & 0 & 0 \\
    0 & 0 & 0 & 0 & 0 & -u_3 sin x_6 \\
    0 & 0 & 0 & 0 & 0 & u_3 cos x_6 \\
    0 & 0 & 0 & 0 & 0 & 0 \\
\end{bmatrix}_{(x_{k},u_{k})}$$

Next, after a new set of observations are received at measurement $k+1$, the filter's correction
stage corrects the predicted state estimate and covariance to render a final new state estimate
$\boldsymbol{\hat{x}}^{+}_{k+1}$ . The correction stage begins by calculating an updated discrete-time
observation matrix Jacobian
$\boldsymbol{\tilde{H}}_{k+1} = \frac{\partial\boldsymbol{h}}{\partial\boldsymbol{x}} |_{\hat{x}^{-}_{k+1}}$
which is computed to be

$$\boldsymbol{\tilde{H}}_{k+1} = \begin{bmatrix}
    \frac{x_5 - x_2}{(\frac{(x_5 - x_2)^2}{(x_4-x_1)^2}+1) (x_4 - x_1)^2} & -\frac{1}{(x_4-x_1)(\frac{(x_5-x_2)^2}{(x_4-x_1)^2} + 1)} & -1 & -\frac{x_5-x_2}{(x_4-x_1)^2(\frac{(x_5-x_2)^2}{(x_4-x_1)^2} + 1)} & \frac{1}{(x_4-x_1)(\frac{(x_5-x_2)^2}{(x_4-x_1)^2} + 1)} & 0 \\
    \frac{x_1 - x_4}{\sqrt{(x_1-x_4)^2 + (x_2-x_5)^2}} & \frac{x_2 - x_5}{\sqrt{(x_1-x_4)^2 + (x_2-x_5)^2}} & 0 & \frac{x_4 - x_1}{\sqrt{(x_1-x_4)^2 + (x_2-x_5)^2}} & \frac{x_5 - x_2}{\sqrt{(x_1-x_4)^2 + (x_2-x_5)^2}} & 0 \\
    -\frac{x_2-x_5}{(x_1-x_4)^2(\frac{(x_2-x_5)^2}{(x_1-x_4)^2} + 1)} & \frac{1}{(x_1-x_4)(\frac{(x_2-x_5)^2}{(x_1-x_4)^2} + 1)} & 0 & \frac{x_2 - x_5}{(\frac{(x_2 - x_5)^2}{(x_1-x_4)^2}+1) (x_1 - x_4)^2} & -\frac{1}{(x_1-x_4)(\frac{(x_2-x_5)^2}{(x_1-x_4)^2} + 1)} & -1 \\
    0 & 0 & 0 & 1 & 0 & 0 \\
    0 & 0 & 0 & 0 & 1 & 0 \\
\end{bmatrix}_{\hat{x}^{-}_{k+1}}$$

and the observation estimate for measurement $k+1$ is

$$\boldsymbol{\hat{y}}_{k+1} = \boldsymbol{\tilde{H}}_{k+1} \boldsymbol{\hat{x}}^{-}_{k+1}$$

The measurement and covariance innovations are then calculated as

$$\boldsymbol{\tilde{e}_y}_{k+1} = \boldsymbol{y}_{k+1} - \boldsymbol{\hat{y}}_{k+1}$$

$$\boldsymbol{S}_{k+1} = \boldsymbol{\tilde{H}}_{k+1} \boldsymbol{P}^{-}_{k+1} \boldsymbol{\tilde{H}}^{T}_{k+1} + \boldsymbol{R}_{k+1}$$

Finally, the Kalman Gain is updated, and the state estimate and filter covariance for measurement step $k+1$ are corrected as follows:

$$\boldsymbol{K}_{k+1} = \boldsymbol{P}^{-}_{k+1} \boldsymbol{\tilde{H}}_{k+1}^{T} \boldsymbol{S}_{k+1}^{-1}$$

$$\boldsymbol{\hat{x}}^{+}_{k+1} = \boldsymbol{\hat{x}}^{-}_{k+1} + \boldsymbol{K}_{k+1} \boldsymbol{\tilde{e}_y}_{k+1}$$

$$\boldsymbol{P}^{+}_{k+1} = (\boldsymbol{I} - \boldsymbol{K}_{k+1}\boldsymbol{\tilde{H}}_{k+1})\boldsymbol{P}^{-}_{k+1}$$


## Unscented Kalman Filter Overview
The UKF is a fully nonlinear filter; formulated by applying an Unscented Transform to the model dynamics,
since it is easier to directly approximate mean and covariance between pdf's than through the nonlinear state
transition functions. Instead of using a Taylor Series expansion to linearize the model dynamics, sigma points
are generated from the current state pdf $p(X_k \: | \: Y_{1:k})$ and propagated to the next time step through
the nonlinear dynamics to approximate the first two moments of the pdf at the future time step. Where
$2n \: + \: 1$ sigma points are required to accurately sketch out $\boldsymbol{E}[\boldsymbol{x}_k] = \boldsymbol{\mu}_x$ and $cov(\boldsymbol{x}_k) = \boldsymbol{P}_{xx}$
from $\boldsymbol{x}_k \: \sim \: p(x_k | y_{1:k})$. Given that $\boldsymbol{P}_{xx} = \boldsymbol{S}^T \boldsymbol{S}$
and $\boldsymbol{S} = chol(\boldsymbol{P}_{xx})$, sigma points are computed as:

$$\boldsymbol{\chi}^0 = \boldsymbol{\mu}_x$$

$$\boldsymbol{\chi}^i = \left\{
    \begin{array}{ll}
        \boldsymbol{\mu}_x + (\sqrt{n + \lambda}) \cdot \boldsymbol{S}^{j,T}, & \; for \: i = 1,...,n, \: and \: j = 1,...,n \\
        \boldsymbol{\mu}_x - (\sqrt{n + \lambda}) \cdot \boldsymbol{S}^{j,T}, & \; for \: i = n+1,...,2n, \: and \: j = 1,...,n
\end{array}
\right.$$

Where $n$ is the number of states in $\boldsymbol{x}_k$ and $\lambda$ is the scaling parameter defined by:

$$\lambda = \alpha^2 \cdot (n + k) - n$$

The propagated sigma points can then be recombined and used to estimate the new mean and covariance.
The variable $\alpha$ controls the spread of the sigma points and as its magnitude is reduced, the estimate
performance approaches that of the EKF.

The UKF algorithm also incorporates a prediction step and measurement update/correction step.
However, a nonlinear transformation will be performed at both steps - approximating the statistics using
the generated sigma points. The prediction step is computed as follows, given initial values for $\boldsymbol{\hat{x}}^{+}_k$, $\boldsymbol{P}^{+}_k$, and $\boldsymbol{S}_k$:

$$\boldsymbol{\chi}^0_k = \boldsymbol{\hat{x}}^{+}_k$$

$$\boldsymbol{\chi}^i = \left\{
    \begin{array}{ll}
        \boldsymbol{\hat{x}}^{+}_k + (\sqrt{n + \lambda}) \cdot \boldsymbol{S}^{j,T}_k, & \; for \: i = 1,...,n, \: and \: j = 1,...,n \\
        \boldsymbol{\hat{x}}^{+}_k - (\sqrt{n + \lambda}) \cdot \boldsymbol{S}^{j,T}_k, & \; for \: i = n+1,...,2n, \: and \: j = 1,...,n \\
\end{array}\right.
$$

Then each of the sigma points at $t = t_k$ is propagated through the nonlinear dynamics function $\boldsymbol{f}$
(again using the BOOST `odeint::runge_kutta_dopri5` solver) to generate predicted points $\boldsymbol{\chi}^{-0}_{k+1} \; and \; \boldsymbol{\chi}^{-i}_{k+1}$ at $t = t_{k+1}$.
The prediction step is concluded by recombining the resultant points to compute the predicted mean and covariance by:

$$\boldsymbol{\hat{x}}^-_{k+1} \approx \sum_{i=0}^{2n} \boldsymbol{\omega}^{i}_m \cdot \boldsymbol{\chi}^{-i}_{k+1}$$

$$\boldsymbol{P}^-_{k+1} \approx \sum_{i=0}^{2n} \boldsymbol{\omega}^{i}_m \cdot (\boldsymbol{\chi}^{-i}_{k+1} - \boldsymbol{\hat{x}}^-_{k+1})(\boldsymbol{\chi}^{-i}_{k+1} - \boldsymbol{\hat{x}}^-_{k+1})^T + \boldsymbol{Q}$$

where

$$\omega^0_m = \frac{\lambda}{n + \lambda}$$

$$\omega^0_c = \frac{\lambda}{n + \lambda} + 1 - \alpha^2 + \beta$$

$$\omega^i_m = \frac{1}{2(n + \lambda)} = \omega^i_c$$

The filter correction step has a form similar to the prediction step. Given $\boldsymbol{\hat{x}}^-_{k+1}$, $\boldsymbol{P}^-_{k+1}$, $\boldsymbol{\bar{S}}_{k+1}$, and observation $\boldsymbol{y}_{k+1}$, the correction
step begins by generating another set of sigma points at measurement $k+1$ for the span of measurement estimates
$(\boldsymbol{\gamma}^{0}_{k+1} \; and \; \boldsymbol{\gamma}^{i}_{k+1})$ by passing the state sigma points
predicted at the previous step through the nonlinear sensor dynamics function $\boldsymbol{h}$ and applying
the mean weights. The updated measurement mean and covariance are then computed by:

$$\boldsymbol{\hat{y}}_{k+1} \approx \sum_{i=0}^{2n} \boldsymbol{\omega}^i_m \cdot \boldsymbol{\gamma}^{i}_{k+1}$$

$$\boldsymbol{P_{yy}}_{k+1} \approx \sum_{i=0}^{2n} \boldsymbol{\omega}^{i}_m \cdot (\boldsymbol{\gamma}^{i}_{k+1} - \boldsymbol{\hat{y}}_{k+1})(\boldsymbol{\gamma}^{i}_{k+1} - \boldsymbol{\hat{y}}_{k+1})^T + \boldsymbol{R}$$

Next, the state-measurement cross-covariance and Kalman gains are calculated in order to compute a corrected
state estimate from the observation $\boldsymbol{y}_{k+1}$.

$$\boldsymbol{P_{xy}}_{k+1} \approx \sum_{i=0}^{2n} \boldsymbol{\omega}^i_c \cdot (\boldsymbol{\chi}^{-i}_{k+1} - \boldsymbol{\hat{x}}^-_{k+1})(\boldsymbol{\gamma}^{i}_{k+1} - \boldsymbol{\hat{y}}_{k+1})^T$$

$$\boldsymbol{K}_{k+1} \approx \boldsymbol{P_{xy}}_{k+1} \cdot [\boldsymbol{P_{yy}}_{k+1}]^{-1}$$

$$\boldsymbol{\hat{x}}^+_{k+1} = \boldsymbol{\hat{x}}^-_{k+1} + \boldsymbol{K}_{k+1} \boldsymbol{P_{yy}}_{k+1} \boldsymbol{K}^T_{k+1} = \boldsymbol{P}^-_{k+1} - \boldsymbol{P_{xy}}_{k+1} [\boldsymbol{P_{yy}}_{k+1}]^{-1} \boldsymbol{P_{xy}}^{T}_{k+1}$$


# Project Build
This project was developed and tested using GCC-15/G++15 for compiling, therefore it is recommended
to use GCC and G++ for generating build files and compiling. If you prefer to use a different compiler,
you may need to make adjustments to `CMakeLists.txt` to build the project.

## Building
Clone down the project from Github and navigate to the project root in your terminal of choice. Next,
run the following command (ommit `-G Ninja` if desired):
```bash
cmake -B build -S . -G Ninja -DCMAKE_BUILD_TYPE=Release
```

If GCC is not your default compiler:
```bash
cmake -B build -S . -G Ninja -DCMAKE_CXX_COMPILER=$(which g++-15) -DCMAKE_C_COMPILER=$(which gcc-15) -DCMAKE_BUILD_TYPE=Release
```

Once generation of the build files has completed, update the `main()` function in `/src/main.cpp`
to configure the filter type you'd like to implement as well as your initial filter, process noise,
and measurement noise covariance diagonals (see `EstimatorType` and `FilterParams`). Then run the
following command to generate the executable:
```bash
cmake --build build
```

## Running
Upon successful completion of building the build process, the program is run by executing the following
command:
```bash
./build/uav_ugv_coop_localization
```
or for Windows users
```bash
./build/uav_ugv_coop_localization.exe
```
