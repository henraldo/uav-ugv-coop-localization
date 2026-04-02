#pragma once

#include "../constants.hpp"
#include "utils.hpp"
#include "estimator.hpp"
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>
#include <cmath>

namespace uav_ugv_sim {

    using SigmaPoints = Eigen::MatrixXd;
    using StatePredictions = Eigen::MatrixXd;
    using MeasPredictions = Eigen::MatrixXd;

    class UKF : public Estimator {

    private:
        int n_, L_;
        double alpha_, beta_, kappa_, lambda_;
        Eigen::VectorXd weights_mean_, weights_covar_;
        SigmaPoints sp_x_;
        ObservationState yhat_;

        // Updates the sets of weights for next state and covariance predictions
        void UpdateWeights() {
            lambda_ = alpha_ * alpha_ * (n_ + kappa_) - n_;

            weights_mean_.resize(L_);
            weights_covar_.resize(L_);

            weights_mean_(0) = lambda_ / (n_ + lambda_);
            weights_covar_(0) = weights_mean_(0) + (1 - (alpha_ * alpha_) + beta_);
            double w = 1 / (2.0 * (n_ + lambda_));

            for (int i = 1; i < L_; i++) {
                weights_mean_(i) = weights_covar_(i) = w;
            }
        }

        // Generate sigma points from last estimated state for next filter prediction step
        void ComputeSigmaPoints() {
            double gamma = std::sqrt(n_ + lambda_);

            Eigen::LLT<StateCov> p_llt(P_);
            Eigen::MatrixXd Svp = p_llt.matrixL();

            sp_x_.col(0) = xhat_;
            for (int i = 0; i < n_; i++) {
                sp_x_.col(i + 1) = xhat_ + (gamma * Svp.col(i));
                sp_x_(2, i + 1) = WrapToPi(sp_x_(2, i + 1));
                sp_x_(5, i + 1) = WrapToPi(sp_x_(5, i + 1));

                sp_x_.col(i + 1 + n_) = xhat_ - (gamma * Svp.col(i));
                sp_x_(2, i + 1 + n_) = WrapToPi(sp_x_(2, i + 1 + n_));
                sp_x_(5, i + 1 + n_) = WrapToPi(sp_x_(5, i + 1 + n_));
            }
        }

    public:
        UKF(
            const SystemState& x0,
            const FilterParams& filter_params,
            const double alpha,
            const double beta,
            const double kappa
        ) : Estimator(x0, filter_params), alpha_(alpha), beta_(beta), kappa_(kappa) {
            estimator_type_ = EstimatorType::UKF;
            n_ = x0.rows();
            L_ = 2 * n_ + 1;

            sp_x_.resize(n_, L_);

            UpdateWeights();
        };

        // Propagate nonlinear system dynamics model
        void Propagate(double t0, const ControlInput& u) override {
            DynamicsModel dyn(u);

    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wuninitialized"
    #pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
            for (int i = 0; i < L_; i++) {
                SystemState x = sp_x_.col(i);

                boost::numeric::odeint::integrate_const(
                    boost::numeric::odeint::runge_kutta4<SystemState>{}, dyn, x, t0, t0 + DT, DT
                );

                x(2) = WrapToPi(x(2));
                x(5) = WrapToPi(x(5));
                sp_x_.col(i) = x;
            }
    #pragma GCC diagnostic pop
        }

        // Performs state and covariance predictions
        void Predict(double t0, const ControlInput& u) override {
            ComputeSigmaPoints();

            UKF::Propagate(t0, u);
            xhat_ = sp_x_ * weights_mean_;
            xhat_(2) = WrapToPi(xhat_(2));
            xhat_(5) = WrapToPi(xhat_(5));

            P_.setZero();
            for (int i = 0; i < L_; i++) {
                SystemState d_xx = sp_x_.col(i) - xhat_;
                d_xx(2) = WrapToPi(d_xx(2));
                d_xx(5) = WrapToPi(d_xx(5));
                P_ += weights_covar_(i) * (d_xx * d_xx.transpose());
            }
            P_ += params_.Q;
        }

        // Corrects state and covariance predictions from latest observations
        void Correct(const ObservationState& z) override {
            int m = z.rows();
            SigmaPoints sp_z(m, L_);

            for (int i = 0; i < L_; i++) {
                sp_z.col(i) = UKF::SensorModel(sp_x_.col(i));
            }

            ObservationState yhat_ = sp_z * weights_mean_;
            yhat_(0) = WrapToPi(yhat_(0));
            yhat_(2) = WrapToPi(yhat_(2));

            ey_ = z - yhat_;

            ey_(0) = WrapToPi(ey_(0));
            ey_(2) = WrapToPi(ey_(2));

            Eigen::MatrixXd Pyy = Eigen::MatrixXd::Zero(m, m);
            for (int i = 0; i < L_; i++) {
                ObservationState d_yy = sp_z.col(i) - yhat_;
                d_yy(0) = WrapToPi(d_yy(0));
                d_yy(2) = WrapToPi(d_yy(2));
                Pyy += weights_covar_(i) * (d_yy * d_yy.transpose());
            }
            Pyy += params_.R;

            Eigen::MatrixXd Pxy(n_, m);
            Pxy.setZero();
            for (int i = 0; i < L_; i++) {
                SystemState dx = sp_x_.col(i) - xhat_;
                dx(2) = WrapToPi(dx(2));
                dx(5) = WrapToPi(dx(5));

                ObservationState dy = sp_z.col(i) - yhat_;
                dy(0) = WrapToPi(dy(0));
                dy(2) = WrapToPi(dy(2));
                Pxy += weights_covar_(i) * (dx * dy.transpose());
            }

            auto K = Pxy * Pyy.inverse();
            xhat_ += K * ey_;
            P_ -= K * Pyy * K.transpose();

            xhat_(2) = WrapToPi(xhat_(2));
            xhat_(5) = WrapToPi(xhat_(5));
        }
    };

}