#pragma once

#include "constants.hpp"
#include "utils.hpp"
#include "estimator.hpp"
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>
#include <cmath>

namespace uav_ugv_sim {

    class EKF : public Estimator {

    public:
        EKF(const SystemState& x0, FilterParams& filter_params) : Estimator(x0, filter_params) {
            estimator_type_ = EstimatorType::EKF;
        }

        // Propagate nonlinear system dynamics model
        void Propagate(double t0, const ControlInput& u) override {
            DynamicsModel dyn(u);

            // Suppress false positive uninitialized warnings from ODEINT/Eigen copies
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wuninitialized"
    #pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
            boost::numeric::odeint::runge_kutta_dopri5<SystemState> stepper;
            boost::numeric::odeint::integrate_adaptive(
                boost::numeric::odeint::make_controlled(1e-6, 1e-6, stepper), dyn, xhat_, t0, DT, DT / 10.0);
    #pragma GCC diagnostic pop

            // ensure UGV and UAV headings are wrapped to [-pi, pi]
            xhat_(2) = WrapToPi(xhat_(2));
            xhat_(5) = WrapToPi(xhat_(5));
        }

        // Performs state and covariance predictions
        void Predict(double t0, const ControlInput& u) override {
            EKF::Propagate(t0, u);
            auto F = EKF::ComputeJacobianF(xhat_, u, DT);
            P_ = F * P_ * F.transpose() + (params_.Omega * params_.Q * params_.Omega.transpose());
        }

        // Corrects state and covariance predictions from latest observations
        void Correct(const ObservationState& z) override {
            auto H = EKF::MeasurmentModel(xhat_);
            ey_ = z - (EKF::SensorModel(xhat_));

            ey_(0) = WrapToPi(ey_(0));
            ey_(2) = WrapToPi(ey_(2));

            auto S = H * P_ * H.transpose() + params_.R;
            auto K = P_ * H.transpose() * S.inverse();

            xhat_ += K * ey_;
            xhat_(2) = WrapToPi(xhat_(2));
            xhat_(5) = WrapToPi(xhat_(5));

            P_ = (StateCov::Identity() - K * H) * P_;
        }
    };

}