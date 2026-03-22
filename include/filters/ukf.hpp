#pragma once

#include "../constants.hpp"
#include "utils.hpp"
#include "estimator.hpp"
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>
#include <cmath>

namespace uav_ugv_sim
{

    using SigmaPoints = Eigen::MatrixXd;
    using StatePredictions = Eigen::MatrixXd;
    using MeasPredictions = Eigen::MatrixXd;

    class UKF : public Estimator
    {

    private:
        int n_, L_;
        double alpha_, beta_, kappa_, lambda_;
        Eigen::VectorXd weights_mean_, weights_covar_;
        SigmaPoints sp_x_;
        ObservationState yhat_;

        void UpdateWeights()
        {
            lambda_ = alpha_ * alpha_ * (n_ + kappa_) - n_;

            weights_mean_.resize(L_);
            weights_covar_.resize(L_);

            weights_mean_(0) = lambda_ / (n_ + lambda_);
            weights_covar_(0) = weights_mean_(0) + (1 - (alpha_ * alpha_) + beta_);
            double w = 1 / (2.0 * (n_ + lambda_));

            for (int i = 1; i < L_; i++)
            {
                weights_mean_(i) = weights_covar_(i) = w;
            }
        }

        void ComputeSigmaPoints()
        {
            double gamma = std::sqrt(n_ + lambda_);

            Eigen::LLT<StateCov> p_llt(P_);
            Eigen::MatrixXd Svp = p_llt.matrixL();

            sp_x_.col(0) = xhat_;
            for (int i = 0; i < L_; i++)
            {
                sp_x_.col(i + 1) = xhat_ + (gamma * Svp);
                sp_x_.col(i + 1 + n_) = xhat_ - (gamma * Svp);
            }
        }

        auto SensorModel(const SystemState& x) const -> ObservationState
        {
            ObservationState z;
            z(0) = wrapToPi(std::atan2(x(4) - x(1), x(3) - x(0)) - x(2));
            z(1) = std::sqrt(std::pow(x(0) - x(3), 2) + std::pow(x(1) - x(4), 2));
            z(2) = wrapToPi(std::atan2(x(1) - x(4), x(0) - x(3)) - x(5));
            z(3) = x(3);
            z(4) = x(4);

            return z;
        }

    public:
        UKF(
            const SystemState& x0,
            const FilterParams& filter_params,
            const double alpha,
            const double beta,
            const double kappa) : Estimator(x0, filter_params), alpha_(alpha), beta_(beta), kappa_(kappa)
        {
            estimator_type_ = EstimatorType::UKF;
            n_ = x0.rows();
            L_ = 2 * n_ + 1;

            sp_x_.resize(n_, L_);

            UpdateWeights();
            ComputeSigmaPoints();
        };

        void Propagate(double t0, const ControlInput& u) override
        {
            DynamicsModel dyn(u);

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuninitialized"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
            for (int i = 0; i < L_; i++)
            {
                SystemState x = sp_x_.col(i);

                boost::numeric::odeint::runge_kutta_dopri5<SystemState> stepper;
                boost::numeric::odeint::integrate_adaptive(
                    boost::numeric::odeint::make_controlled(1e-6, 1e-6, stepper), dyn, x, t0, DT, DT / 10.0);

                x(2) = wrapToPi(x(2));
                x(5) = wrapToPi(x(5));
                sp_x_.col(i) = x;
            }
#pragma GCC diagnostic pop
        }

        void Predict(double t0, const ControlInput& u) override
        {
            UKF::Propagate(t0, u);
            xhat_ = sp_x_ * weights_mean_;

            for (int i = 0; i < L_; i++)
            {
                SystemState d_xx = sp_x_.col(i) - xhat_;
                P_ += weights_covar_(i) * (d_xx * d_xx.transpose());
            }
            P_ += params_.Q;
        }

        void Correct(const ObservationState& z) override
        {
            int m = z.rows();
            SigmaPoints sp_z(m, L_);

            for (int i = 0; i < L_; i++)
            {
                sp_z.col(i) = UKF::SensorModel(sp_x_.col(i));
            }

            ObservationState yhat_ = sp_z * weights_covar_;
            ey_ = z - yhat_;

            ey_(0) = wrapToPi(ey_(0));
            ey_(2) = wrapToPi(ey_(2));

            Eigen::MatrixXd Pyy = Eigen::MatrixXd::Zero(m, m);
            for (int i = 0; i < L_; i++)
            {
                ObservationState dyy = sp_z.col(i) - yhat_;
                Pyy += weights_covar_(i) * (dyy * dyy.transpose());
            }
            Pyy += params_.R;

            Eigen::MatrixXd Pxy(n_, m);
            Pxy.setZero();
            for (int i = 0; i < L_; i++)
            {
                SystemState dx = sp_x_.col(i) - xhat_;
                ObservationState dy = sp_z.col(i) - yhat_;
                Pxy += weights_covar_(i) * (dx * dy.transpose());
            }

            auto K = Pxy * Pyy.inverse();
            xhat_ += K * ey_;
            P_ -= K * Pyy * K.transpose();

            xhat_(2) = wrapToPi(xhat_(2));
            xhat_(5) = wrapToPi(xhat_(5));
        }
    };

}