#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace TargetMeasurement
{
namespace Attitude
{

// ======================================================================
// 欧拉角 → 旋转矩阵
// ======================================================================
// axes 为 3 字符转序字符串，R = R_axes[2](a3) · R_axes[1](a2) · R_axes[0](a1)
//
// 常用转序：
//   "ZYX" → R = Rx(a3) · Ry(a2) · Rz(a1)  — aerospace 体→NED
//   "XYZ" → R = Rz(a3) · Ry(a2) · Rx(a1)  — robotics
//   "ZXZ" → R = Rz(a3) · Rx(a2) · Rz(a1)  — 经典欧拉角
// ======================================================================

/// @param axes  3 字符转序字符串，如 "ZYX"、"XYZ"
/// @param a1    第一旋转角（弧度）
/// @param a2    第二旋转角（弧度）
/// @param a3    第三旋转角（弧度）
/// @param R     输出 3×3 旋转矩阵
inline void eulerToMatrix(const char axes[3], double a1, double a2, double a3,
                           double R[3][3])
{
    using RM33 = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;

    auto mulLeft = [](double R[3][3], char axis, double angle)
    {
        double c = std::cos(angle), s = std::sin(angle);
        double Rk[3][3];
        if (axis == 'X')
            Rk[0][0]=1,Rk[0][1]=0,Rk[0][2]=0, Rk[1][0]=0,Rk[1][1]=c,Rk[1][2]=-s, Rk[2][0]=0,Rk[2][1]=s,Rk[2][2]=c;
        else if (axis == 'Y')
            Rk[0][0]=c,Rk[0][1]=0,Rk[0][2]=s, Rk[1][0]=0,Rk[1][1]=1,Rk[1][2]=0, Rk[2][0]=-s,Rk[2][1]=0,Rk[2][2]=c;
        else
            Rk[0][0]=c,Rk[0][1]=-s,Rk[0][2]=0, Rk[1][0]=s,Rk[1][1]=c,Rk[1][2]=0, Rk[2][0]=0,Rk[2][1]=0,Rk[2][2]=1;
        Eigen::Map<const RM33> A(&Rk[0][0]);
        Eigen::Map<RM33> Rm(&R[0][0]);
        Rm = A * Rm;
    };

    Eigen::Map<RM33>(&R[0][0]).setIdentity();
    mulLeft(R, axes[0], a1);
    mulLeft(R, axes[1], a2);
    mulLeft(R, axes[2], a3);
}

} // namespace Attitude
} // namespace TargetMeasurement
