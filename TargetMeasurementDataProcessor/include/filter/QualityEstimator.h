#pragma once

#include "FilterTypes.h"
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

namespace TargetMeasurement
{
namespace Filter
{

/**
 * QualityEstimator
 * -----------------
 * 独立误差评估组件，与具体滤波器解耦。
 *
 * 使用方式（数据推入模式）：
 *   1. 每次滤波更新后调用 recordUpdate(z, zPred, S)
 *   2. 每次无探测外推后调用 recordPredict(dt)
 *   3. 需要 DRMS / 特征值时先 updateCovariance(P)，再取值
 */
class QualityEstimator
{
public:
    QualityEstimator()
        : filterState_(TM_STATE_UNINITIALIZED)
        , passCount_(0)
        , sumNIS_(0.0)
        , nisCount_(0)
        , nisWindowCount_(0)
        , lastNIS_(0.0)
    {
        for (int i = 0; i < 5; ++i)
            nisWindow_[i] = 0.0;
        covariance_.setZero(6, 6);
    }

    // ---- 推入当前协方差（供 DRMS / 特征值计算） ----

    void updateCovariance(const Eigen::MatrixXd& P)
    {
        covariance_ = P;
    }

    // ---- 滤波器生命周期状态 ----

    int getFilterState() const { return filterState_; }
    int getPass() const { return passCount_; }

    // ---- 每次测量更新后调用 ----

    void recordUpdate(const Eigen::VectorXd& innovation,
                      const Eigen::MatrixXd& innovationCovariance)
    {
        passCount_++;

        // 归一化新息平方 NIS = yᵀ·S⁻¹·y
        double nis = innovation.transpose() * innovationCovariance.inverse() * innovation;
        lastNIS_ = nis;

        // 滑动窗口
        nisWindow_[nisWindowCount_ % 5] = nis;
        nisWindowCount_ = std::min(nisWindowCount_ + 1, 5);

        // 历史累计
        sumNIS_ += nis;
        nisCount_++;

        // 状态推进
        if (filterState_ == TM_STATE_UNINITIALIZED)
            filterState_ = TM_STATE_INITIALIZING;
        else if (passCount_ >= 3)
            filterState_ = TM_STATE_TRACKING;
    }

    // ---- 无探测外推时调用 ----

    void recordPredict(double dt)
    {
        (void)dt;
        if (filterState_ == TM_STATE_TRACKING)
            filterState_ = TM_STATE_COASTING;
    }

    // ---- 重置 ----

    void reset()
    {
        filterState_ = TM_STATE_UNINITIALIZED;
        passCount_ = 0;
        sumNIS_ = 0.0;
        nisCount_ = 0;
        nisWindowCount_ = 0;
        lastNIS_ = 0.0;
        for (int i = 0; i < 5; ++i)
            nisWindow_[i] = 0.0;
        covariance_.setZero();
    }

    // ---- 位置 / 速度 RMS 误差 ----

    double getPositionRmsError() const
    {
        int stateDim = static_cast<int>(covariance_.rows());
        int posDim = stateDim / 2;
        if (posDim < 1)
            return 0.0;

        double sum = 0.0;
        for (int i = 0; i < posDim; ++i)
            sum += covariance_(i, i);
        return std::sqrt(std::max(0.0, sum));
    }

    double getVelocityRmsError() const
    {
        int stateDim = static_cast<int>(covariance_.rows());
        int posDim = stateDim / 2;
        if (posDim < 1)
            return 0.0;

        double sum = 0.0;
        for (int i = posDim; i < stateDim; ++i)
            sum += covariance_(i, i);
        return std::sqrt(std::max(0.0, sum));
    }

    // ---- 卡方统计量 ----

    double getChiSquared()    const { return lastNIS_; }
    double getAverageNIS()    const { return nisCount_ == 0 ? 0.0 : sumNIS_ / nisCount_; }

    double getAvgChiSquared() const
    {
        if (nisWindowCount_ == 0)
            return 0.0;
        double sum = 0.0;
        int n = std::min(nisWindowCount_, 5);
        for (int i = 0; i < n; ++i)
            sum += nisWindow_[i];
        return sum / n;
    }

    // ---- 跟踪质量 [0, 1] ----

    double getTrackQuality() const
    {
        if (nisCount_ < 2)
            return 0.0;
        double expectedNIS = 3.0;
        double avgNIS = getAverageNIS();
        double deviation = std::abs(avgNIS - expectedNIS) / expectedNIS;
        return std::exp(-deviation);
    }

    // ---- 误差椭球轴长（位置协方差特征值的平方根） ----

    void getPositionSigmas(double sigmas[3]) const
    {
        sigmas[0] = sigmas[1] = sigmas[2] = 0.0;

        int stateDim = static_cast<int>(covariance_.rows());
        int posDim = stateDim / 2;
        if (posDim < 1)
            return;

        // 提取位置子矩阵（补齐至 3×3）
        Eigen::Matrix3d Ppos = Eigen::Matrix3d::Zero();
        int n = std::min(posDim, 3);
        Ppos.topLeftCorner(n, n) = covariance_.topLeftCorner(n, n);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(Ppos);
        Eigen::Vector3d ev = solver.eigenvalues();
        // σA ≥ σB ≥ σC（降序）
        sigmas[0] = std::sqrt(std::max(0.0, ev(2)));
        sigmas[1] = std::sqrt(std::max(0.0, ev(1)));
        sigmas[2] = std::sqrt(std::max(0.0, ev(0)));
    }

    // ---- NIS 持久化 ----

    double getLastNIS() const { return lastNIS_; }
    double getSumNIS()  const { return sumNIS_; }
    int    getNisCount() const { return nisCount_; }

    void setNisState(double sumNIS, int nisCount, double lastNIS)
    {
        sumNIS_ = sumNIS;
        nisCount_ = nisCount;
        lastNIS_ = lastNIS;
    }

private:
    int    filterState_;
    int    passCount_;
    double sumNIS_;
    int    nisCount_;
    double nisWindow_[5];
    int    nisWindowCount_;
    double lastNIS_;
    Eigen::MatrixXd covariance_;
};

} // namespace Filter
} // namespace TargetMeasurement
