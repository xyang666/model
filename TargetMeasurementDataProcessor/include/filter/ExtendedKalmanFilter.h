#pragma once

#include "FilterBase.h"
#include <Eigen/Dense>
#include <functional>

namespace TargetMeasurement
{
    namespace Filter
    {

        /**
         * ExtendedKalmanFilter
         * ---------------------
         * 扩展 Kalman 滤波器，通过 Jacobian 矩阵对非线性系统进行线性化。
         *
         * Q（过程噪声协方差）和 R（测量噪声协方差）由 setQ/setR 传入完整矩阵。
         */
        class ExtendedKalmanFilter : public FilterBase
        {
        public:
            using StateFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &, double)>;
            using StateJacobianFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &, double)>;
            using MeasureFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;
            using MeasureJacobianFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;

            ExtendedKalmanFilter(int stateDim, int measureDim)
                : stateDim_(stateDim), measureDim_(measureDim), state_(Eigen::VectorXd::Zero(stateDim)), covariance_(Eigen::MatrixXd::Identity(stateDim, stateDim)), Q_(Eigen::MatrixXd::Identity(stateDim, stateDim)), R_(Eigen::MatrixXd::Identity(measureDim, measureDim)), initialized_(false)
            {
            }

            // ---- FilterBase 接口 ----

            void reset() override
            {
                state_.setZero();
                covariance_.setIdentity();
                initialized_ = false;
            }

            bool isInitialized() const override { return initialized_; }
            Eigen::VectorXd state() const override { return state_; }
            Eigen::MatrixXd covariance() const override { return covariance_; }

            void predict(double dt) override
            {
                if (!initialized_ || dt <= 0.0 || !f_ || !F_)
                    return;

                Eigen::VectorXd predictedState = f_(state_, dt);
                if (predictedState.size() != stateDim_)
                    return;

                Eigen::MatrixXd jacobian = F_(state_, dt);
                if (jacobian.rows() != stateDim_ || jacobian.cols() != stateDim_)
                    return;

                state_ = predictedState;
                covariance_ = jacobian * covariance_ * jacobian.transpose() + Q_;
            }

            void update(const Eigen::VectorXd &z) override
            {
                if (!initialized_ || z.size() != measureDim_ || !h_ || !H_)
                    return;

                Eigen::VectorXd predictedMeasurement = h_(state_);
                if (predictedMeasurement.size() != measureDim_)
                    return;

                Eigen::MatrixXd jacobian = H_(state_);
                if (jacobian.rows() != measureDim_ || jacobian.cols() != stateDim_)
                    return;

                innovation_ = z - predictedMeasurement;
                innovationCovariance_ = jacobian * covariance_ * jacobian.transpose() + R_;
                Eigen::MatrixXd K = covariance_ * jacobian.transpose() * innovationCovariance_.inverse();

                state_ += K * innovation_;
                covariance_ = (Eigen::MatrixXd::Identity(stateDim_, stateDim_) - K * jacobian) * covariance_;
            }

            // ---- 噪声矩阵设置 ----

            void setQ(const Eigen::MatrixXd &Q) { Q_ = Q; }
            void setR(const Eigen::MatrixXd &R) { R_ = R; }
            const Eigen::MatrixXd &getQ() const { return Q_; }
            const Eigen::MatrixXd &getR() const { return R_; }

            // ---- 新息查询 ----

            const Eigen::VectorXd &innovation() const override { return innovation_; }
            const Eigen::MatrixXd &innovationCovariance() const override { return innovationCovariance_; }

            // ---- 回调设置 ----

            void setStateFunc(const StateFunc &f) override { f_ = f; }
            void setStateJacobian(const StateJacobianFunc &F) override { F_ = F; }
            void setMeasureFunc(const MeasureFunc &h) override { h_ = h; }
            void setMeasureJacobian(const MeasureJacobianFunc &H) override { H_ = H; }

            // ---- 初始化 ----

            void Initialize(const Eigen::VectorXd &state, const Eigen::MatrixXd &covariance)
            {
                if (state.size() != stateDim_ || covariance.rows() != stateDim_ || covariance.cols() != stateDim_)
                    return;
                state_ = state;
                covariance_ = covariance;
                initialized_ = true;
            }

        private:
            int stateDim_;
            int measureDim_;
            Eigen::VectorXd state_;
            Eigen::MatrixXd covariance_;
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
