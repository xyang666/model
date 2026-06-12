#ifndef COORD_TRANSFORM_H
#define COORD_TRANSFORM_H

#include "erfa.h"
#include <vector>
#include <string>

// EOP数据结构
struct EOPData {
    double mjd;           // Modified Julian Date
    double x_pole;        // 极移x (arcsec)
    double y_pole;        // 极移y (arcsec)
    double ut1_utc;       // UT1-UTC (seconds)
    double lod;           // Length of day (ms)
    double dx;            // dX章动修正 (mas)
    double dy;            // dY章动修正 (mas)
};

// 闰秒数据结构
struct LeapSecond {
    double mjd;           // Modified Julian Date
    double tai_utc;       // TAI-UTC (seconds)
};

// 坐标系类型
enum CoordSystem {
    GCRS,    // Geocentric Celestial Reference System (地心天球参考系)
    ITRS,    // International Terrestrial Reference System (国际地球参考系)
    CIRS,    // Celestial Intermediate Reference System (天球中间参考系)
    TIRS,    // Terrestrial Intermediate Reference System (地球中间参考系)
    ICRS     // International Celestial Reference System (国际天球参考系)
};

// 坐标转换类
class CoordTransform {
private:
    std::vector<EOPData> eop_data;
    std::vector<LeapSecond> leap_seconds;

    // 读取finals2000A.all文件
    bool loadEOPData(const std::string& filename);

    // 读取leap-seconds文件
    bool loadLeapSeconds(const std::string& filename);

    // 插值获取指定MJD的EOP参数
    bool interpolateEOP(double mjd, EOPData& eop);

    // 获取指定MJD的TAI-UTC
    double getTAI_UTC(double mjd);

public:
    CoordTransform();
    ~CoordTransform();

    // 初始化，加载EOP数据
    bool initialize(const std::string& eop_file);

    // 初始化，加载EOP数据和闰秒数据
    bool initialize(const std::string& eop_file, const std::string& leap_file);

    // 坐标转换主函数
    bool transform(
        const double pos_in[3],      // 输入位置 (m)
        CoordSystem from_sys,         // 源坐标系
        CoordSystem to_sys,           // 目标坐标系
        double utc_mjd,               // UTC时间(MJD)
        double pos_out[3]             // 输出位置 (m)
    );

    // 位置和速度转换主函数
    bool transformPV(
        const double pv_in[2][3],    // 输入位置和速度 [pos, vel] (m, m/s)
        CoordSystem from_sys,         // 源坐标系
        CoordSystem to_sys,           // 目标坐标系
        double utc_mjd,               // UTC时间(MJD)
        double pv_out[2][3]           // 输出位置和速度 (m, m/s)
    );

    // GCRS -> ITRS (位置和速度)
    bool gcrs2itrsPV(const double pv_gcrs[2][3], double utc_mjd, double pv_itrs[2][3]);

    // ITRS -> GCRS (位置和速度)
    bool itrs2gcrsPV(const double pv_itrs[2][3], double utc_mjd, double pv_gcrs[2][3]);

    // GCRS -> ITRS
    bool gcrs2itrs(const double gcrs[3], double utc_mjd, double itrs[3]);

    // ITRS -> GCRS
    bool itrs2gcrs(const double itrs[3], double utc_mjd, double gcrs[3]);

    // GCRS -> CIRS
    bool gcrs2cirs(const double gcrs[3], double tt_mjd, double cirs[3]);

    // CIRS -> GCRS
    bool cirs2gcrs(const double cirs[3], double tt_mjd, double gcrs[3]);

    // CIRS -> TIRS
    bool cirs2tirs(const double cirs[3], double ut1_mjd, double tirs[3]);

    // TIRS -> CIRS
    bool tirs2cirs(const double tirs[3], double ut1_mjd, double cirs[3]);

    // TIRS -> ITRS
    bool tirs2itrs(const double tirs[3], double utc_mjd, double itrs[3]);

    // ITRS -> TIRS
    bool itrs2tirs(const double itrs[3], double utc_mjd, double tirs[3]);

    // 辅助函数：UTC转TT
    void utc2tt(double utc_mjd, double* tt_mjd1, double* tt_mjd2);

    // 辅助函数：UTC转UT1
    void utc2ut1(double utc_mjd, double dut1, double* ut1_mjd1, double* ut1_mjd2);
};

#endif // COORD_TRANSFORM_H
