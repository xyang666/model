#pragma once

#include <Eigen/Dense>
#include <functional>

namespace TargetMeasurement
{
    namespace Filter
    {

        using StateFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &, double)>;
        using StateJacobianFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &, double)>;
        using MeasureFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;
        using MeasureJacobianFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;

        class FilterBase
        {
        public:
            virtual ~FilterBase() = default;

            virtual void reset() = 0;
            virtual void predict(double dt) = 0;
            virtual void update(const Eigen::VectorXd &z) = 0;

            virtual bool isInitialized() const = 0;
            virtual Eigen::VectorXd state() const = 0;
            virtual Eigen::MatrixXd covariance() const = 0;
            virtual const Eigen::VectorXd &innovation() const = 0;
            virtual const Eigen::MatrixXd &innovationCovariance() const = 0;

            virtual void setStateFunc(const StateFunc &f) { (void)f; }
            virtual void setStateJacobian(const StateJacobianFunc &F) { (void)F; }
            virtual void setMeasureFunc(const MeasureFunc &h) { (void)h; }
            virtual void setMeasureJacobian(const MeasureJacobianFunc &H) { (void)H; }

            virtual void setQ(const Eigen::MatrixXd &Q) { (void)Q; }
            virtual void setR(const Eigen::MatrixXd &R) { (void)R; }
        };

    } // namespace Filter
} // namespace TargetMeasurement
