#ifndef ANTENNA_HPP
#define ANTENNA_HPP

#include "AntennaPattern.hpp"
#include <Eigen/Core>
#include <Eigen/Dense>

// 带波束指向的定向天线，将信号到达方向投影到波束坐标系后查询方向图增益。
//
// 坐标约定（ENU 平地）：
//   azimuth  — 水平面内的角度，从 +Y（北）起顺时针为正 (rad)
//   elevation — 相对水平面的角度，向上为正 (rad)
//
// boresight 定义天线"看向"的方向。
// gain_dBi(source_az, source_el) 将信号来向相对波束轴的偏置角
// 分解为 (off_az, off_el) 分量后查询方向图。

class Antenna
{
public:
    // 波束扫描模式。
    enum class ScanMode
    {
        FIXED,    // 波束固定指向 boresight
        AZ_SCAN,  // 仅在方位方向扫描
        EL_SCAN,  // 仅在仰角方向扫描
        AZ_EL_SCAN // 方位和仰角均可扫描
    };

    AntennaPattern pattern;

    double boresight_az_rad{0.0}; // 波束指向方位角 (rad)
    double boresight_el_rad{0.0}; // 波束指向仰角 (rad)

    // --- 扫描参数 ---
    ScanMode scan_mode{ScanMode::FIXED};
    double scan_min_az_rad{ -kPi};
    double scan_max_az_rad{  kPi};
    double scan_min_el_rad{-kPi_2};
    double scan_max_el_rad{ kPi_2};

    // --- 视场角约束 (FOV) ---
    double fov_min_az_rad{ -kPi};
    double fov_max_az_rad{  kPi};
    double fov_min_el_rad{-kPi_2};
    double fov_max_el_rad{ kPi_2};
    double min_range_m{0.0};
    double max_range_m{1.0e9};

    // 计算来自 (source_az_rad, source_el_rad) 方向的信号增益 (dBi)。
    // frequency_hz > 0 时传递到方向图的频率修正表。
    double gain_dBi(double source_az_rad, double source_el_rad,
                    double frequency_hz = 0.0) const;

    // 计算波束轴与信号来向之间的总偏置角 (rad)。
    double off_boresight_angle_rad(double source_az_rad, double source_el_rad) const;

    // 检查信号来向 (az_rad, el_rad) 和距离 (range_m) 是否在FOV内。
    bool within_fov(double az_rad, double el_rad, double range_m) const;

    static constexpr double kPi   = 3.14159265358979323846;
    static constexpr double kPi_2 = 1.57079632679489661923;

private:
    // 将 (source_az, source_el) 分解为相对波束轴的 (off_az, off_el)。
    // 使用小角度安全的球面投影到波束切平面。
    void compute_offsets(double  source_az_rad,
                         double  source_el_rad,
                         double& off_az_rad,
                         double& off_el_rad) const;
};

#endif // ANTENNA_HPP
