#include <iostream>
#include <cmath>
#include <string>
#include "SGP_SDP.hpp"

using namespace SGPSDP;

// 解析TLE
bool ParseTLE(const std::string& line1, const std::string& line2, TLE& tle) {
    if (line1.length() < 69 || line2.length() < 69) return false;

    // Line 1
    tle.epoch = 0.0; // 简化处理
    tle.xndt2o = std::stod(line1.substr(33, 10)) * 2.0 * PI / (XMNPDA * XMNPDA);
    tle.xndd6o = std::stod(line1.substr(44, 8)) * 6.0 * PI / (XMNPDA * XMNPDA * XMNPDA);
    tle.bstar = std::stod(line1.substr(53, 8)) / 100000.0;

    // Line 2
    tle.xincl = std::stod(line2.substr(8, 8)) * PI / 180.0;
    tle.xnodeo = std::stod(line2.substr(17, 8)) * PI / 180.0;
    tle.eo = std::stod("0." + line2.substr(26, 7));
    tle.omegao = std::stod(line2.substr(34, 8)) * PI / 180.0;
    tle.xmo = std::stod(line2.substr(43, 8)) * PI / 180.0;
    tle.xno = std::stod(line2.substr(52, 11)) * 2.0 * PI / XMNPDA;

    return true;
}

int main() {
    // ISS TLE
    std::string line1 = "1 25544U 98067A   08264.51782528 -.00002182  00000-0 -11606-4 0  2927";
    std::string line2 = "2 25544  51.6416 247.4627 0006703 130.5360 325.0288 15.72125391563537";

    TLE tle;
    if (!ParseTLE(line1, line2, tle)) {
        std::cerr << "Failed to parse TLE" << std::endl;
        return 1;
    }

    // 传播时间 (分钟)
    double tsince = 60.0;

    // 位置和速度 (km, km/min)
    double pos[3], vel[3];

    Propagate(tle, tsince, pos, vel);

    std::cout << "Time: " << tsince << " min" << std::endl;
    std::cout << "Position (km): " << pos[0] << ", " << pos[1] << ", " << pos[2] << std::endl;
    std::cout << "Velocity (km/min): " << vel[0] << ", " << vel[1] << ", " << vel[2] << std::endl;

    // 转换为 km/s
    std::cout << "Velocity (km/s): " << vel[0]/60.0 << ", " << vel[1]/60.0 << ", " << vel[2]/60.0 << std::endl;

    return 0;
}
