#include "tracker/RadarTracker.h"
#include "utils/CoordinateTransform.h"

using namespace TargetMeasurement::Coord;

namespace TargetMeasurement
{

RadarTracker::RadarTracker()
{
    setSensorFunc(TM_SENSOR_RADAR,
        [this](const void* m, const Eigen::VectorXd& x, const Eigen::MatrixXd& P) {
            return this->computeObservation(m, x, P);
        });
}

Tracker::ObservationData RadarTracker::computeObservation(
    const void* measurement,
    const Eigen::VectorXd& predictedState,
    const Eigen::MatrixXd& predictedCov) const
{
    (void)predictedState; (void)predictedCov;

    auto* m = static_cast<const TM_RadarMeasurement*>(measurement);
    const auto& input = m->input;

    // ---- 填充缺省值 ----
    double azErr = (m->azimuthError > 0.0) ? m->azimuthError : 0.1 * 3.14159265358979323846 / 180.0;
    double elErr = (m->elevationError > 0.0) ? m->elevationError : 0.1 * 3.14159265358979323846 / 180.0;
    double range = m->range;
    double rangeErr = m->rangeError;
    if (range <= 0.0 || !(m->validFlags & TM_MEAS_RANGE))
    {
        range = 50000.0;
        rangeErr = 50.0;
    }
    else if (rangeErr <= 0.0)
        rangeErr = 50.0;

    // ---- 传感器位置 LLA → 世界系 ----
    double sensorEcef[3] = {input.position[0], input.position[1], input.position[2]};
    if (input.referenceFrame == TM_FRAME_LLA)
        llaToEcef(input.position[0], input.position[1], input.position[2], sensorEcef);

    // ================================================================
    // 链式协方差传播：cov_world = R_ned2world * (R_body2ned * cov_body * R_body2ned^T) * R_ned2world^T
    //
    // 第一步：RBE → Body（Jacobian 传播）
    //    cov_body = J * diag(sigma²) * Jᵀ
    //
    // 第二步：Body → NED（姿态矩阵旋转）
    //    cov_ned = R_bn * cov_body * R_bnᵀ
    //
    // 第三步：NED → 世界系（参考系变换）
    //    cov_world = R_ne * cov_ned * R_neᵀ
    //
    // 位置变换与协方差使用同样的旋转链：
    //    p_body = RBE_to_Body(r, az, el)
    //    p_ned  = R_bn * p_body
    //    p_world = R_ne * p_ned + origin
    // ================================================================

    // ---- 第一步：RBE → Body（Jacobian 传播） ----
    double body[3];
    rbeToBody(range, m->azimuth, m->elevation, body);

    double J[3][3];
    rbeJacobian(range, m->azimuth, m->elevation, J);

    using RM33 = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
    Eigen::Matrix3d covBody = Eigen::Vector3d(rangeErr * rangeErr,
                                              azErr * azErr,
                                              elErr * elErr).asDiagonal();
    covBody = Eigen::Map<const RM33>(&J[0][0]) * covBody
            * Eigen::Map<const RM33>(&J[0][0]).transpose();

    // ---- 第二步：Body → NED（姿态矩阵） ----
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R_bn(input.attitude);

    Eigen::Vector3d bodyVec(body[0], body[1], body[2]);
    Eigen::Vector3d nedVec = R_bn * bodyVec;
    Eigen::Matrix3d covNED = R_bn * covBody * R_bn.transpose();

    // ---- 第三步：NED → 世界系 ----
    CoordTransform xf(TM_FRAME_NED, TM_FRAME_ECEF, sensorEcef);

    double nedArr[3] = {nedVec(0), nedVec(1), nedVec(2)};
    double ecefArr[3];
    xf.applyPos(nedArr, ecefArr);
    Eigen::Vector3d ecefPos(ecefArr[0], ecefArr[1], ecefArr[2]);

    // 协方差旋转：R_ne * covNED * R_ne^T
    Eigen::Matrix3d R_ne = xf.getR();
    Eigen::Matrix3d ecefCov = R_ne * covNED * R_ne.transpose();

    // ---- 返回观测数据 ----
    ObservationData obs;
    obs.measurement = ecefPos;
    obs.R = ecefCov;
    return obs;
}

} // namespace TargetMeasurement
