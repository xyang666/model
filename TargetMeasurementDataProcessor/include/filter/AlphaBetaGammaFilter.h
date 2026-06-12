#pragma once

#include "FilterBase.h"

namespace TargetMeasurement
{
namespace Filter
{

/**
 * AlphaBetaGammaFilter
 * ---------------------
 * 二阶 alpha-beta-gamma 滤波器，适用于匀加速目标跟踪。
 *
 * 算法原理：
 *   预测：x̂⁻ = x̂ + v̂·Δt + ½·â·Δt²   （匀加速外推）
 *         v̂⁻ = v̂ + â·Δt
 *   更新：x̂  = x̂⁻ + α·(z - x̂⁻)         （位置校正）
 *         v̂  = v̂⁻ + β·(z - x̂⁻)/Δt     （速度校正）
 *         â  = â  + γ·(z - x̂⁻)/(½Δt²) （加速度校正）
 *
 * 状态向量：3 维 [位置; 速度; 加速度]
 */
class AlphaBetaGammaFilter : public FilterBase
{
public:
    AlphaBetaGammaFilter(double alpha = 0.85, double beta = 0.005, double gamma = 0.0001)
        : alpha_(alpha)
        , beta_(beta)
        , gamma_(gamma)
        , dt_(1.0)
    {
        state_.setZero();
        cov_.setIdentity();
    }

    // ---- FilterBase 接口 ----

    void reset() override
    {
        state_.setZero();
        cov_.setIdentity();
        initialized_ = false;
    }

    void predict(double dt) override
    {
        dt_ = dt;
        if (!initialized_ || dt <= 0.0)
            return;

        // 状态外推：x = x + v·dt + ½·a·dt²
        state_(0) += state_(1) * dt + 0.5 * state_(2) * dt * dt;
        state_(1) += state_(2) * dt;

        // 协方差外推（简化）
        cov_(0, 0) += cov_(1, 1) * dt * dt + cov_(2, 2) * dt * dt * dt * dt * 0.25;
        cov_(0, 1) += cov_(1, 1) * dt;
        cov_(1, 0) = cov_(0, 1);
    }

    void update(const Eigen::VectorXd& z) override
    {
        if (z.size() < 1)
            return;

        if (!initialized_)
        {
            state_(0) = z(0);
            state_(1) = 0.0;
            state_(2) = 0.0;
            initialized_ = true;
            return;
        }

        double residual = z(0) - state_(0);
        innovation_(0) = residual;
        innovationCovariance_(0, 0) = cov_(0, 0);  // simplified
        state_(0) += alpha_ * residual;
        state_(1) += beta_ * residual / dt_;
        state_(2) += gamma_ * residual / (0.5 * dt_ * dt_);
    }

    bool isInitialized() const override { return initialized_; }
    Eigen::VectorXd state() const override { return state_; }
    Eigen::MatrixXd covariance() const override { return cov_; }

    // ---- 便捷方法 ----

    void Initialize(double position, double velocity, double acceleration = 0.0)
    {
        state_(0) = position;
        state_(1) = velocity;
        state_(2) = acceleration;
        initialized_ = true;
    }

    double GetPosition() const { return state_(0); }
    double GetVelocity() const { return state_(1); }
    double GetAcceleration() const { return state_(2); }

    const Eigen::VectorXd& innovation() const override { return innovation_; }
    const Eigen::MatrixXd& innovationCovariance() const override { return innovationCovariance_; }

private:
    double alpha_;
    double beta_;
    double gamma_;
    double dt_;
    Eigen::Vector3d state_{Eigen::Vector3d::Zero()};
    Eigen::Matrix3d cov_{Eigen::Matrix3d::Identity()};
    Eigen::VectorXd innovation_{Eigen::VectorXd::Zero(1)};
    Eigen::MatrixXd innovationCovariance_{Eigen::MatrixXd::Identity(1, 1)};
    bool initialized_ = false;
};

} // namespace Filter
} // namespace TargetMeasurement
