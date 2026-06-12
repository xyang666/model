#pragma once

#include "FilterBase.h"
#include <Eigen/Dense>
#include <deque>
#include <utility>

namespace TargetMeasurement
{
namespace Filter
{

/**
 * SlidingWindowFilter
 * --------------------
 * 无模型滑动窗口多项式拟合滤波器。
 *
 * 不假设任何目标运动模型（无匀速/匀加速等先验），
 * 纯粹将最近 N 帧测量值通过最小二乘拟合为多项式，
 * 以多项式在查询时刻的值作为状态估计。
 *
 * 参数：
 *   windowSize：滑动窗口大小（帧数），建议 6~20
 *   polyDegree：多项式阶数
 *     0 → 仅估计位置
 *     1 → 估计位置 + 速度
 *     2 → 估计位置 + 速度 + 加速度
 *
 * 状态向量：[pos(3); vel(3); ...]（3 维测量 × polyDegree+1 项）
 */
class SlidingWindowFilter : public FilterBase
{
public:
    SlidingWindowFilter(int measureDim = 3,
                        int windowSize = 10,
                        int polyDegree = 1)
        : measureDim_(measureDim)
        , windowSize_(windowSize)
        , polyDegree_(polyDegree)
        , stateDim_(measureDim * (polyDegree + 1))
        , currentTime_(0.0)
        , initialized_(false)
    {
        state_.setZero(stateDim_);
        cov_.setIdentity(stateDim_, stateDim_);
    }

    // ---- FilterBase 接口 ----

    void reset() override
    {
        window_.clear();
        state_.setZero();
        cov_.setIdentity();
        initialized_ = false;
    }

    bool isInitialized() const override { return initialized_; }
    Eigen::VectorXd state() const override { return state_; }
    Eigen::MatrixXd covariance() const override { return cov_; }

    void predict(double dt) override
    {
        if (!initialized_ || window_.empty())
            return;

        double targetTime = currentTime_ + dt;

        if (window_.size() < polyDegree_ + 2)
        {
            // 样本不足，无法拟合，退化至恒定外推
            extrapolateConstant(dt);
            return;
        }

        // 对每个测量维度独立拟合多项式并外推
        Eigen::VectorXd coeffs = fitPolynomial();
        int coeffPerDim = polyDegree_ + 1;
        for (int dim = 0; dim < measureDim_; ++dim)
        {
            for (int deg = 0; deg <= polyDegree_; ++deg)
            {
                int idx = dim * coeffPerDim + deg;
                double value = (deg == 0) ? coeffs(idx) : coeffs(idx);
                if (deg == 0)
                {
                    // 位置 = 多项式在 targetTime 处的值
                    state_(dim) = evaluatePoly(coeffs, dim, targetTime);
                }
                else
                {
                    // 速度 = 一阶导数，加速度 = 二阶导数
                    state_(measureDim_ + dim + (deg - 1) * measureDim_) =
                        evaluatePolyDerivative(coeffs, dim, deg, targetTime);
                }
            }
        }

        currentTime_ = targetTime;

        // 协方差：使用拟合残差的统计量
        updateCovarianceFromResiduals(coeffs);
    }

    void update(const Eigen::VectorXd& z) override
    {
        if (z.size() < measureDim_)
            return;

        // 首次测量：初始化窗口和状态
        if (!initialized_ || window_.empty())
        {
            window_.push_back({currentTime_, z.head(measureDim_)});
            state_.head(measureDim_) = z.head(measureDim_);
            state_.tail(stateDim_ - measureDim_).setZero();
            initialized_ = true;
            return;
        }

        // 添加新测量，维持窗口大小
        window_.push_back({currentTime_, z.head(measureDim_)});
        while (static_cast<int>(window_.size()) > windowSize_)
            window_.pop_front();

        // 重新拟合
        if (static_cast<int>(window_.size()) >= polyDegree_ + 2)
        {
            Eigen::VectorXd coeffs = fitPolynomial();
            int coeffPerDim = polyDegree_ + 1;
            for (int dim = 0; dim < measureDim_; ++dim)
            {
                state_(dim) = evaluatePoly(coeffs, dim, currentTime_);
                for (int deg = 1; deg <= polyDegree_; ++deg)
                {
                    state_(measureDim_ + dim + (deg - 1) * measureDim_) =
                        evaluatePolyDerivative(coeffs, dim, deg, currentTime_);
                }
            }

            updateCovarianceFromResiduals(coeffs);
        }
        else
        {
            // 样本不足，直接用最新测量值作为位置
            state_.head(measureDim_) = z.head(measureDim_);
        }

        // 存储新息供 QualityEstimator 使用
        innovation_ = z.head(measureDim_) - state_.head(measureDim_);
    }

    // ---- QualityEstimator 支持 ----

    const Eigen::VectorXd& innovation() const override { return innovation_; }
    const Eigen::MatrixXd& innovationCovariance() const override { return innovationCovariance_; }

    // ---- 配置 ----

    void setWindowSize(int n) { windowSize_ = n; }
    void setPolyDegree(int d)
    {
        polyDegree_ = d;
        stateDim_ = measureDim_ * (d + 1);
        state_.setZero(stateDim_);
        cov_.setIdentity(stateDim_, stateDim_);
    }
    void setCurrentTime(double t) { currentTime_ = t; }

private:
    // 构建时间-测量矩阵，对每个维度分别做最小二乘拟合
    // 返回系数向量 [coeffs_dim0; coeffs_dim1; ...]
    // 每维 coeffs = [a0, a1, ..., a_degree]（低次到高次）
    Eigen::VectorXd fitPolynomial() const
    {
        int n = static_cast<int>(window_.size());
        int coeffPerDim = polyDegree_ + 1;

        // 构建设计矩阵 V（范德蒙德矩阵）
        Eigen::MatrixXd V(n, coeffPerDim);
        double t0 = window_.front().first;
        for (int i = 0; i < n; ++i)
        {
            double tRel = window_[i].first - t0;
            double tk = 1.0;
            for (int k = 0; k < coeffPerDim; ++k)
            {
                V(i, k) = tk;
                tk *= tRel;
            }
        }

        // 预计算 (VᵀV)⁻¹Vᵀ
        Eigen::MatrixXd VtV = V.transpose() * V;
        Eigen::MatrixXd pseudoInv = VtV.ldlt().solve(V.transpose());

        // 对每个测量维度独立求解
        Eigen::VectorXd coeffs(measureDim_ * coeffPerDim);
        for (int dim = 0; dim < measureDim_; ++dim)
        {
            Eigen::VectorXd y(n);
            int idx = 0;
            for (const auto& p : window_)
                y(idx++) = p.second(dim);

            coeffs.segment(dim * coeffPerDim, coeffPerDim) = pseudoInv * y;
        }

        return coeffs;
    }

    // 在时刻 t 处评估第 dim 维的多项式值
    double evaluatePoly(const Eigen::VectorXd& coeffs, int dim, double t) const
    {
        double t0 = window_.front().first;
        double tRel = t - t0;
        int coeffPerDim = polyDegree_ + 1;
        int base = dim * coeffPerDim;
        double result = 0.0;
        double tk = 1.0;
        for (int k = 0; k < coeffPerDim; ++k)
        {
            result += coeffs(base + k) * tk;
            tk *= tRel;
        }
        return result;
    }

    // 评估多项式第 deg 阶导数在时刻 t 的值
    double evaluatePolyDerivative(const Eigen::VectorXd& coeffs, int dim, int deg, double t) const
    {
        double t0 = window_.front().first;
        double tRel = t - t0;
        int coeffPerDim = polyDegree_ + 1;
        int base = dim * coeffPerDim;
        double result = 0.0;

        double factorial = 1.0;
        for (int k = 1; k <= deg; ++k)
            factorial *= k;

        double tk = (deg == 0) ? 1.0 : factorial;
        for (int k = deg; k < coeffPerDim; ++k)
        {
            result += coeffs(base + k) * tk;
            tk *= tRel / (k - deg + 1) * (k + 1);
        }

        // 简化：对每项求导
        result = 0.0;
        for (int k = deg; k < coeffPerDim; ++k)
        {
            double coeff = 1.0;
            for (int j = 0; j < deg; ++j)
                coeff *= (k - j);
            double power = (k > deg) ? std::pow(tRel, k - deg) : 1.0;
            result += coeffs(base + k) * coeff * power;
        }

        return result;
    }

    // 仅使用最新测量外推（退化模式）
    void extrapolateConstant(double dt)
    {
        (void)dt;
        // 位置保持不变，速度保持原值
        currentTime_ += dt;
    }

    // 从拟合残差更新协方差
    void updateCovarianceFromResiduals(const Eigen::VectorXd& coeffs)
    {
        int n = static_cast<int>(window_.size());
        if (n < 2)
            return;

        int coeffPerDim = polyDegree_ + 1;

        // 计算每个维度的残差方差
        Eigen::MatrixXd P = Eigen::MatrixXd::Zero(stateDim_, stateDim_);
        for (int dim = 0; dim < measureDim_; ++dim)
        {
            double sumSq = 0.0;
            int idx = 0;
            for (const auto& p : window_)
            {
                double fitted = evaluatePoly(coeffs, dim, p.first);
                double resid = p.second(dim) - fitted;
                sumSq += resid * resid;
                idx++;
            }
            double var = sumSq / (n - coeffPerDim);

            // 位置方差
            P(dim, dim) = var;
            // 速度方差（由位置方差和时间跨度推导）
            double timeSpan = window_.back().first - window_.front().first;
            if (timeSpan > 0.0 && polyDegree_ >= 1)
            {
                P(measureDim_ + dim, measureDim_ + dim) = var / (timeSpan * timeSpan);
            }
        }

        cov_ = 0.5 * cov_ + 0.5 * P; // 指数平滑协方差
    }

    int measureDim_;
    int windowSize_;
    int polyDegree_;
    int stateDim_;
    double currentTime_;

    // 滑动窗口：(时间, 测量向量)
    std::deque<std::pair<double, Eigen::VectorXd>> window_;

    Eigen::VectorXd state_;
    Eigen::MatrixXd cov_;
    Eigen::VectorXd innovation_;
    Eigen::MatrixXd innovationCovariance_;
    bool initialized_;
};

} // namespace Filter
} // namespace TargetMeasurement
