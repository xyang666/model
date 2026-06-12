#pragma once

#include "FilterBase.h"
#include <Eigen/Dense>

namespace TargetMeasurement
{
    namespace Filter
    {

        /**
         * KalmanFilterND
         * ----------------
         * 纯数学 Kalman 滤波器，无状态结构假设。
         *
         * 通过 setStateModel / setMeasModel 设置回调：
         *   f(x, dt)、F(x, dt)—状态转移及 Jacobian
         *   h(x)、H(x)       —观测预测及 Jacobian
         *
         * Q 和 R 通过 setQ/setR 设置。
         */
        class KalmanFilterND : public FilterBase
        {
        public:
            KalmanFilterND(int stateDim, int measureDim)
                : stateDim_(stateDim), measureDim_(measureDim), x_(Eigen::VectorXd::Zero(stateDim)), P_(Eigen::MatrixXd::Identity(stateDim, stateDim)), Q_(Eigen::MatrixXd::Identity(stateDim, stateDim)), R_(Eigen::MatrixXd::Identity(measureDim, measureDim)), initialized_(false)
            {
            }

            // ---- FilterBase 接口 ----

            void reset() override
            {
                x_.setZero();
                P_.setIdentity();
                initialized_ = false;
            }

            bool isInitialized() const override { return initialized_; }
            Eigen::VectorXd state() const override { return x_; }
            Eigen::MatrixXd covariance() const override { return P_; }

            void predict(double dt) override
            {
                if (!initialized_ || dt <= 0.0 || !f_ || !F_)
                    return;

                Eigen::MatrixXd F_mat = F_(x_, dt);
                x_ = f_(x_, dt);
                P_ = F_mat * P_ * F_mat.transpose() + Q_;
            }

            void update(const Eigen::VectorXd &z) override
            {
                if (z.size() != measureDim_)
                    return;

                if (!initialized_)
                {
                    int n = std::min(measureDim_, stateDim_);
                    Eigen::VectorXd init = Eigen::VectorXd::Zero(stateDim_);
                    init.head(n) = z.head(n);
                    x_ = init;
                    initialized_ = true;

                    Eigen::MatrixXd H_mat = H_(x_);
                    innovation_ = z;
                    innovationCovariance_ = H_mat * P_ * H_mat.transpose() + R_;
                    return;
                }

                Eigen::MatrixXd H_mat = H_(x_);
                Eigen::VectorXd zPred = h_(x_);
                innovation_ = z - zPred;
                innovationCovariance_ = H_mat * P_ * H_mat.transpose() + R_;

                Eigen::MatrixXd K = P_ * H_mat.transpose() * innovationCovariance_.inverse();
                x_ += K * innovation_;
                P_ = (Eigen::MatrixXd::Identity(stateDim_, stateDim_) - K * H_mat) * P_;
            }

            const Eigen::VectorXd &innovation() const override { return innovation_; }
            const Eigen::MatrixXd &innovationCovariance() const override { return innovationCovariance_; }

            void setStateFunc(const StateFunc &f) override { f_ = f; }
            void setStateJacobian(const StateJacobianFunc &F) override { F_ = F; }
            void setMeasureFunc(const MeasureFunc &h) override { h_ = h; }
            void setMeasureJacobian(const MeasureJacobianFunc &H) override { H_ = H; }

            // ---- 噪声矩阵设置 ----

            void setQ(const Eigen::MatrixXd &Q) override { Q_ = Q; }
            void setR(const Eigen::MatrixXd &R) override { R_ = R; }
            const Eigen::MatrixXd &getQ() const { return Q_; }
            const Eigen::MatrixXd &getR() const { return R_; }

            /// 直接设置状态和协方差
            void setStateAndCovariance(const Eigen::VectorXd &x,
                                       const Eigen::MatrixXd &P)
            {
                if (x.size() != stateDim_ ||
                    P.rows() != stateDim_ || P.cols() != stateDim_)
                    return;

                x_ = x;
                P_ = P;
                initialized_ = true;
            }

        private:
            int stateDim_;
            int measureDim_;
            Eigen::VectorXd x_;
            Eigen::MatrixXd P_;
            Eigen::MatrixXd Q_;
            Eigen::MatrixXd R_;
            Eigen::VectorXd innovation_;
            Eigen::MatrixXd innovationCovariance_;
            bool initialized_;

            StateFunc f_;
            StateJacobianFunc F_;
            MeasureFunc h_;
            MeasureJacobianFunc H_;
        };

    } // namespace Filter
} // namespace TargetMeasurement
