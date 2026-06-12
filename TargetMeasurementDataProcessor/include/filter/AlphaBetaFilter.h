#pragma once

#include "FilterBase.h"
#include "QualityEstimator.h"

namespace TargetMeasurement
{
namespace Filter
{

/**
 * AlphaBetaFilter
 * ----------------
 * 一阶 alpha-beta 滤波器，适用于匀速目标跟踪。
 *
 * 算法原理：
 *   预测：x̂⁻ = x̂ + v̂·Δt          （匀速外推）
 *   更新：x̂  = x̂⁻ + α·(z - x̂⁻)   （位置校正）
 *         v̂  = v̂  + β·(z - x̂⁻)/Δt （速度校正）
 * 其中 α 控制位置平滑度，β 控制速度响应速度。
 *
 * 状态向量：2 维 [位置; 速度]
 */
class AlphaBetaFilter : public FilterBase
{
public:
    AlphaBetaFilter(double alpha = 0.85, double beta = 0.005)
        : alpha_(alpha)
        , beta_(beta)
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

        // 状态外推：x = x + v·dt
        state_(0) += state_(1) * dt;

        // 协方差外推（简化为对角增长）
        cov_(0, 0) += cov_(1, 1) * dt * dt;
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
            initialized_ = true;
            return;
        }

        double residual = z(0) - state_(0);
        innovation_(0) = residual;
        innovationCovariance_(0, 0) = cov_(0, 0);  // simplified
        state_(0) += alpha_ * residual;
        state_(1) += beta_ * residual / dt_;
    }

    bool isInitialized() const override { return initialized_; }
    Eigen::VectorXd state() const override { return state_; }
    Eigen::MatrixXd covariance() const override { return cov_; }

    // ---- 便捷方法（保留兼容） ----

    void Initialize(double position, double velocity)
    {
        state_(0) = position;
        state_(1) = velocity;
        initialized_ = true;
    }

    double GetPosition() const { return state_(0); }
    double GetVelocity() const { return state_(1); }

    const Eigen::VectorXd& innovation() const override { return innovation_; }
    const Eigen::MatrixXd& innovationCovariance() const override { return innovationCovariance_; }

private:
    double alpha_;
    double beta_;
    double dt_;
    Eigen::Vector2d state_{Eigen::Vector2d::Zero()};
    Eigen::Matrix2d cov_{Eigen::Matrix2d::Identity()};
    Eigen::VectorXd innovation_{Eigen::VectorXd::Zero(1)};
    Eigen::MatrixXd innovationCovariance_{Eigen::MatrixXd::Identity(1, 1)};
    bool initialized_ = false;
};

} // namespace Filter
} // namespace TargetMeasurement
