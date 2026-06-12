#pragma once

#include "FilterBase.h"
#include <Eigen/Dense>
#include <functional>
#include <vector>

namespace TargetMeasurement
{
namespace Filter
{

/**
 * UnscentedKalmanFilter
 * ---------------------
 * 无迹 Kalman 滤波器，使用 sigma 点通过非线性函数传播概率分布。
 *
 * Q（过程噪声协方差）和 R（测量噪声协方差）由 setQ/setR 传入完整矩阵。
 */
class UnscentedKalmanFilter : public FilterBase
{
public:
    using StateFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd&, double)>;
    using MeasureFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;

    UnscentedKalmanFilter(int stateDim, int measureDim,
                          double alpha = 1e-3,
                          double beta = 2.0,
                          double kappa = 0.0)
        : stateDim_(stateDim)
        , measureDim_(measureDim)
        , alpha_(alpha)
        , beta_(beta)
        , kappa_(kappa)
        , lambda_(alpha * alpha * (stateDim + kappa) - stateDim)
        , state_(Eigen::VectorXd::Zero(stateDim))
        , covariance_(Eigen::MatrixXd::Identity(stateDim, stateDim))
        , Q_(Eigen::MatrixXd::Identity(stateDim, stateDim))
        , R_(Eigen::MatrixXd::Identity(measureDim, measureDim))
        , initialized_(false)
    {
        sigmaCount_ = 2 * stateDim_ + 1;
        weightsMean_.resize(sigmaCount_);
        weightsCovariance_.resize(sigmaCount_);
        weightsMean_[0] = lambda_ / (stateDim_ + lambda_);
        weightsCovariance_[0] = weightsMean_[0] + (1.0 - alpha_ * alpha_ + beta_);
        for (int i = 1; i < sigmaCount_; ++i)
        {
            weightsMean_[i] = 1.0 / (2.0 * (stateDim_ + lambda_));
            weightsCovariance_[i] = weightsMean_[i];
        }
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
        if (!initialized_ || dt <= 0.0 || !f_)
            return;

        Eigen::MatrixXd sigmaPoints = generateSigmaPoints();
        Eigen::MatrixXd sigmaState(stateDim_, sigmaCount_);

        for (int i = 0; i < sigmaCount_; ++i)
            sigmaState.col(i) = f_(sigmaPoints.col(i), dt);

        state_.setZero();
        for (int i = 0; i < sigmaCount_; ++i)
            state_ += weightsMean_[i] * sigmaState.col(i);

        covariance_.setZero();
        for (int i = 0; i < sigmaCount_; ++i)
        {
            Eigen::VectorXd diff = sigmaState.col(i) - state_;
            covariance_ += weightsCovariance_[i] * diff * diff.transpose();
        }
        covariance_ += Q_;
    }

    void update(const Eigen::VectorXd& z) override
    {
        if (!initialized_ || z.size() != measureDim_ || !h_)
            return;

        Eigen::MatrixXd sigmaPoints = generateSigmaPoints();
        Eigen::MatrixXd sigmaMeasurement(measureDim_, sigmaCount_);

        for (int i = 0; i < sigmaCount_; ++i)
            sigmaMeasurement.col(i) = h_(sigmaPoints.col(i));

        zPred_ = Eigen::VectorXd::Zero(measureDim_);
        for (int i = 0; i < sigmaCount_; ++i)
            zPred_ += weightsMean_[i] * sigmaMeasurement.col(i);

        innovation_ = z - zPred_;

        Eigen::MatrixXd S = Eigen::MatrixXd::Zero(measureDim_, measureDim_);
        Eigen::MatrixXd crossCov = Eigen::MatrixXd::Zero(stateDim_, measureDim_);
        for (int i = 0; i < sigmaCount_; ++i)
        {
            Eigen::VectorXd zDiff = sigmaMeasurement.col(i) - zPred_;
            Eigen::VectorXd xDiff = sigmaPoints.col(i) - state_;
            S += weightsCovariance_[i] * zDiff * zDiff.transpose();
            crossCov += weightsCovariance_[i] * xDiff * zDiff.transpose();
        }
        S += R_;

        innovationCovariance_ = S;

        Eigen::MatrixXd K = crossCov * S.inverse();
        state_ += K * innovation_;
        covariance_ -= K * S * K.transpose();
    }

    // ---- 噪声矩阵设置 ----

    void setQ(const Eigen::MatrixXd& Q) { Q_ = Q; }
    void setR(const Eigen::MatrixXd& R) { R_ = R; }
    const Eigen::MatrixXd& getQ() const { return Q_; }
    const Eigen::MatrixXd& getR() const { return R_; }

    // ---- 新息查询 ----

    const Eigen::VectorXd& innovation() const override { return innovation_; }
    const Eigen::MatrixXd& innovationCovariance() const override { return innovationCovariance_; }

    // ---- 回调设置 ----

    void setStateFunc(const StateFunc& f) { f_ = f; }
    void setMeasureFunc(const MeasureFunc& h) { h_ = h; }

    // ---- 初始化 ----

    void Initialize(const Eigen::VectorXd& state, const Eigen::MatrixXd& covariance)
    {
        if (state.size() != stateDim_ || covariance.rows() != stateDim_ || covariance.cols() != stateDim_)
            return;
        state_ = state;
        covariance_ = covariance;
        initialized_ = true;
    }

private:
    Eigen::MatrixXd generateSigmaPoints() const
    {
        Eigen::MatrixXd sigmaPoints(stateDim_, sigmaCount_);
        Eigen::MatrixXd sqrtMatrix = ((stateDim_ + lambda_) * covariance_).llt().matrixL();
        sigmaPoints.col(0) = state_;
        for (int i = 0; i < stateDim_; ++i)
        {
            sigmaPoints.col(i + 1) = state_ + sqrtMatrix.col(i);
            sigmaPoints.col(i + 1 + stateDim_) = state_ - sqrtMatrix.col(i);
        }
        return sigmaPoints;
    }

    int stateDim_;
    int measureDim_;
    double alpha_;
    double beta_;
    double kappa_;
    double lambda_;
    int sigmaCount_;
    Eigen::VectorXd state_;
    Eigen::MatrixXd covariance_;
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;
    Eigen::VectorXd innovation_;
    Eigen::MatrixXd innovationCovariance_;
    Eigen::VectorXd zPred_;
    std::vector<double> weightsMean_;
    std::vector<double> weightsCovariance_;
    bool initialized_;

    StateFunc f_;
    MeasureFunc h_;
};

} // namespace Filter
} // namespace TargetMeasurement
