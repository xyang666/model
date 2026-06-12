#include "Antenna.hpp"

#include <cmath>

static constexpr double kPi = 3.14159265358979323846;

// ---------------------------------------------------------------------------
// (az, el) → 单位向量（ENU 约定）：
//   x = 东 = cos(el)*sin(az)
//   y = 北 = cos(el)*cos(az)
//   z = 天 = sin(el)
// ---------------------------------------------------------------------------
static Eigen::Vector3d unit_vec(double az_rad, double el_rad)
{
    return Eigen::Vector3d(
        std::cos(el_rad) * std::sin(az_rad),
        std::cos(el_rad) * std::cos(az_rad),
        std::sin(el_rad));
}

double Antenna::off_boresight_angle_rad(double source_az_rad, double source_el_rad) const
{
    Eigen::Vector3d bore = unit_vec(boresight_az_rad, boresight_el_rad);
    Eigen::Vector3d src = unit_vec(source_az_rad, source_el_rad);

    // 将点积钳位到 [-1, 1] 防止浮点舍入误差
    double dot = std::max(-1.0, std::min(1.0, bore.dot(src)));
    return std::acos(dot);
}

void Antenna::compute_offsets(double source_az_rad,
                              double source_el_rad,
                              double &off_az_rad,
                              double &off_el_rad) const
{
    // 构建与波束轴对齐的正交坐标系：
    //   bx = 波束轴单位向量
    //   by = 水平垂直轴（方位轴）
    //   bz = 仰角轴
    Eigen::Vector3d bx = unit_vec(boresight_az_rad, boresight_el_rad);

    // 方位垂直方向：将波束轴旋转 90° 后投影到水平面
    Eigen::Vector3d by = unit_vec(boresight_az_rad + kPi / 2.0, 0.0);

    // 仰角轴：叉积（右手定则，指向波束"上方"）
    Eigen::Vector3d bz = bx.cross(by);
    if (bz.norm() < 1e-12)
    {
        // 波束轴接近垂直；回退到世界 Up 叉积波束轴
        Eigen::Vector3d up(0.0, 0.0, 1.0);
        bz = up.cross(bx);
        if (bz.norm() < 1e-12)
        {
            off_az_rad = 0.0;
            off_el_rad = 0.0;
            return;
        }
        bz.normalize();
        by = bz.cross(bx);
    }
    else
    {
        bz.normalize();
    }

    Eigen::Vector3d src = unit_vec(source_az_rad, source_el_rad);

    // 将信号来向投影到波束切平面
    double dot_bx = bx.dot(src);
    dot_bx = std::max(-1.0, std::min(1.0, dot_bx));
    double theta = std::acos(dot_bx); // 总偏置角

    // 投影到切平面的 by（方位）和 bz（仰角）轴
    // 小角度时切平面投影精度高；
    // 大角度时仍提供有用的有符号分解。
    double proj_by = by.dot(src);
    double proj_bz = bz.dot(src);
    double proj_mag = std::sqrt(proj_by * proj_by + proj_bz * proj_bz);

    if (proj_mag < 1e-12)
    {
        off_az_rad = 0.0;
        off_el_rad = 0.0;
    }
    else
    {
        off_az_rad = theta * (proj_by / proj_mag);
        off_el_rad = theta * (proj_bz / proj_mag);
    }
}

double Antenna::gain_dBi(double source_az_rad, double source_el_rad,
                         double frequency_hz) const
{
    double off_az = 0.0;
    double off_el = 0.0;
    compute_offsets(source_az_rad, source_el_rad, off_az, off_el);
    return pattern.gain_dBi(off_az, off_el, frequency_hz);
}

// FOV检查：信号来向和距离是否在视场角约束内。
bool Antenna::within_fov(double az_rad, double el_rad, double range_m) const
{
    if (az_rad < fov_min_az_rad || az_rad > fov_max_az_rad)
        return false;
    if (el_rad < fov_min_el_rad || el_rad > fov_max_el_rad)
        return false;
    if (range_m < min_range_m || range_m > max_range_m)
        return false;
    return true;
}
