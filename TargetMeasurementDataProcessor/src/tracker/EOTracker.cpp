#include "tracker/EOTracker.h"
#include "utils/CoordinateTransform.h"
#include <cmath>

using namespace TargetMeasurement::Coord;

namespace TargetMeasurement
{

EOTracker::EOTracker(double focalLenPx, double cx, double cy)
    : fx_(focalLenPx), fy_(focalLenPx), cx_(cx), cy_(cy)
{
    setSensorFunc(TM_SENSOR_EO,
        [this](const void* m, const Eigen::VectorXd& x, const Eigen::MatrixXd& P) {
            return this->computeObservation(m, x, P);
        });
}

Tracker::ObservationData EOTracker::computeObservation(
    const void* measurement,
    const Eigen::VectorXd& predictedState,
    const Eigen::MatrixXd& predictedCov) const
{
    (void)predictedCov;

    auto* m = static_cast<const TM_EOMeasurement*>(measurement);
    const auto& input = m->input;

    // ---- 像素 → 本体系 bearing 角 ----
    double az = std::atan2(m->pixelU - cx_, fx_);
    double el = std::atan2(m->pixelV - cy_, fy_);
    double azErr = (m->pixelErrorU > 0) ? m->pixelErrorU : 1.0;
    double elErr = (m->pixelErrorV > 0) ? m->pixelErrorV : 1.0;

    // 像素误差 → 角度误差（通过 intrinsic Jacobian）
    double J_az_u = fx_ / ((m->pixelU - cx_) * (m->pixelU - cx_) + fx_ * fx_);
    double J_el_v = fy_ / ((m->pixelV - cy_) * (m->pixelV - cy_) + fy_ * fy_);
    double sigmaAz = J_az_u * azErr;
    double sigmaEl = J_el_v * elErr;

    // ---- 预测距离（从滤波器状态） ----
    double sensorEcef[3] = {input.position[0], input.position[1], input.position[2]};
    if (input.referenceFrame == TM_FRAME_LLA)
        llaToEcef(input.position[0], input.position[1], input.position[2], sensorEcef);

    double dx = predictedState(0) - sensorEcef[0];
    double dy = predictedState(1) - sensorEcef[1];
    double dz = predictedState(2) - sensorEcef[2];
    double range = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (range < 1.0) range = 50000.0;

    // ---- 用 range 做伪 RBE → ECEF 变换 ----
    double body[3];
    rbeToBody(range, az, el, body);

    double J[3][3];
    rbeJacobian(range, az, el, J);

    // 协方差：bearing-only 用 30% 距离不确定性
    double rangeErr = range * 0.3;

    Eigen::Matrix3d covBody;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            covBody(i, j) = J[i][0] * J[j][0] * rangeErr * rangeErr
                          + J[i][1] * J[j][1] * sigmaAz * sigmaAz
                          + J[i][2] * J[j][2] * sigmaEl * sigmaEl;

    // ---- 链式传播：Body → NED → 世界系 ----
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R_bn(input.attitude);

    Eigen::Vector3d bodyVec(body[0], body[1], body[2]);
    Eigen::Vector3d nedVec = R_bn * bodyVec;
    Eigen::Matrix3d covNED = R_bn * covBody * R_bn.transpose();

    CoordTransform xf(TM_FRAME_NED, TM_FRAME_ECEF, sensorEcef);

    double nedArr[3] = {nedVec(0), nedVec(1), nedVec(2)};
    double ecefArr[3];
    xf.applyPos(nedArr, ecefArr);
    Eigen::Vector3d ecefPos(ecefArr[0], ecefArr[1], ecefArr[2]);

    Eigen::Matrix3d ecefCov = xf.getR() * covNED * xf.getR().transpose();

    // ---- 返回 ----
    ObservationData obs;
    obs.measurement = ecefPos;
    obs.R = ecefCov;
    return obs;
}


} // namespace TargetMeasurement
