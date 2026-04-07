#include <iostream>
#include "SGP4.h"
#include "utils.h"
#include <Eigen/Dense>
#include <cmath>

int main()
{
    using namespace SGP4Funcs;
    elsetrec satrec;

    // 示例TLE数据（国际空间站的示例TLE）
    char tle1[] = "1 25544U 98067A   23166.00000000  .00000000  00000-0  00000-0 0  9999";
    char tle2[] = "2 25544  51.6400  10.0000 0002000  90.0000 270.0000 15.50000000    01";

    double startmfe = 0.0;   // 从epoch时间开始计算
    double stopmfe = 1440.0; // 计算一天内的轨道位置
    double deltamin = 10.0;   // 每分钟计算一次位置
    // 初始化卫星记录
    twoline2rv(tle1, tle2, 'v', 'd', 'i', wgs84, startmfe, stopmfe, deltamin, satrec);

    if (satrec.error != 0)
    {
        std::cout << "Error initializing satellite: " << satrec.error << std::endl;
        return 1;
    }

    // 计算卫星在epoch后的位置和速度（tsince = 0 表示epoch时间）
    double r[3], v[3];
    double tsince = 0.0; // 分钟

    sgp4(satrec, tsince, r, v);

    // 输出结果
    // 使用 deltamin 作为步长，从 0 到 stopmfe 递增 tsince
    for (tsince = 0.0; tsince <= stopmfe; tsince += deltamin)
    {
        sgp4(satrec, tsince, r, v);
    }
    std::cout << "Time: " << tsince << " min, Position: " << r[0] << ", " << r[1] << ", " << r[2] << std::endl;
    // std::cout << "position (km):" << std::endl;
    // std::cout << "X: " << r[0] << std::endl;
    // std::cout << "Y: " << r[1] << std::endl;
    // std::cout << "Z: " << r[2] << std::endl;

    // std::cout << "velocity (km/s):" << std::endl;
    // std::cout << "VX: " << v[0] << std::endl;
    // std::cout << "VY: " << v[1] << std::endl;
    // std::cout << "VZ: " << v[2] << std::endl;

    return 0;
}