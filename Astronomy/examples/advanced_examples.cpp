// 天文坐标转换系统 - 高级使用示例

#include "coord_transform.h"
#include <iostream>
#include <iomanip>
#include <cmath>

// 示例1: 地面站坐标转换
void example_ground_station() {
    std::cout << "\n=== 示例1: 地面站坐标转换 ===" << std::endl;

    CoordTransform transformer;
    transformer.initialize("finals2000A.all");

    // 北京站ITRS坐标 (WGS84)
    // 经度: 116.2°E, 纬度: 40.2°N, 高度: 50m
    double beijing_itrs[3] = {-2148744.0, 4426641.0, 4044655.0};

    // 转换到GCRS (用于卫星轨道计算)
    double beijing_gcrs[3];
    double mjd = 59000.0;  // 某个观测时刻

    if (transformer.itrs2gcrs(beijing_itrs, mjd, beijing_gcrs)) {
        std::cout << "北京站ITRS: [" << beijing_itrs[0] << ", "
                  << beijing_itrs[1] << ", " << beijing_itrs[2] << "]" << std::endl;
        std::cout << "北京站GCRS: [" << beijing_gcrs[0] << ", "
                  << beijing_gcrs[1] << ", " << beijing_gcrs[2] << "]" << std::endl;
    }
}

// 示例2: 卫星位置转换
void example_satellite_position() {
    std::cout << "\n=== 示例2: 卫星位置转换 ===" << std::endl;

    CoordTransform transformer;
    transformer.initialize("finals2000A.all");

    // 卫星在GCRS中的位置 (例如GPS卫星)
    double sat_gcrs[3] = {20000000.0, 15000000.0, 10000000.0};

    // 转换到ITRS (用于地面观测)
    double sat_itrs[3];
    double mjd = 59000.5;

    if (transformer.gcrs2itrs(sat_gcrs, mjd, sat_itrs)) {
        std::cout << "卫星GCRS: [" << sat_gcrs[0] << ", "
                  << sat_gcrs[1] << ", " << sat_gcrs[2] << "]" << std::endl;
        std::cout << "卫星ITRS: [" << sat_itrs[0] << ", "
                  << sat_itrs[1] << ", " << sat_itrs[2] << "]" << std::endl;

        // 计算距离
        double distance = sqrt(sat_itrs[0]*sat_itrs[0] +
                              sat_itrs[1]*sat_itrs[1] +
                              sat_itrs[2]*sat_itrs[2]);
        std::cout << "卫星距地心距离: " << distance/1000.0 << " km" << std::endl;
    }
}

// 示例3: 时间序列转换
void example_time_series() {
    std::cout << "\n=== 示例3: 时间序列转换 ===" << std::endl;

    CoordTransform transformer;
    transformer.initialize("finals2000A.all");

    // 固定的ITRS位置
    double pos_itrs[3] = {6378137.0, 0.0, 0.0};  // 赤道上的点

    std::cout << "赤道点在不同时刻的GCRS坐标:" << std::endl;
    std::cout << std::fixed << std::setprecision(1);

    // 一天内每6小时转换一次
    for (int i = 0; i < 4; i++) {
        double mjd = 59000.0 + i * 0.25;  // 每6小时
        double pos_gcrs[3];

        if (transformer.itrs2gcrs(pos_itrs, mjd, pos_gcrs)) {
            std::cout << "MJD " << mjd << ": ["
                     << pos_gcrs[0] << ", "
                     << pos_gcrs[1] << ", "
                     << pos_gcrs[2] << "]" << std::endl;
        }
    }
}

// 示例4: 中间坐标系使用
void example_intermediate_systems() {
    std::cout << "\n=== 示例4: 中间坐标系转换 ===" << std::endl;

    CoordTransform transformer;
    transformer.initialize("finals2000A.all");

    double pos_itrs[3] = {6378137.0, 0.0, 0.0};
    double mjd = 59000.0;

    // 逐步转换: ITRS -> TIRS -> CIRS -> GCRS
    double pos_tirs[3], pos_cirs[3], pos_gcrs[3];

    transformer.transform(pos_itrs, ITRS, TIRS, mjd, pos_tirs);
    transformer.transform(pos_tirs, TIRS, CIRS, mjd, pos_cirs);
    transformer.transform(pos_cirs, CIRS, GCRS, mjd, pos_gcrs);

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "ITRS: [" << pos_itrs[0] << ", " << pos_itrs[1] << ", " << pos_itrs[2] << "]" << std::endl;
    std::cout << "TIRS: [" << pos_tirs[0] << ", " << pos_tirs[1] << ", " << pos_tirs[2] << "]" << std::endl;
    std::cout << "CIRS: [" << pos_cirs[0] << ", " << pos_cirs[1] << ", " << pos_cirs[2] << "]" << std::endl;
    std::cout << "GCRS: [" << pos_gcrs[0] << ", " << pos_gcrs[1] << ", " << pos_gcrs[2] << "]" << std::endl;
}

// 示例5: 精度验证
void example_accuracy_test() {
    std::cout << "\n=== 示例5: 往返转换精度验证 ===" << std::endl;

    CoordTransform transformer;
    transformer.initialize("finals2000A.all");

    double original[3] = {-2148744.0, 4426641.0, 4044655.0};
    double temp[3], result[3];
    double mjd = 59000.0;

    // ITRS -> GCRS -> ITRS
    transformer.itrs2gcrs(original, mjd, temp);
    transformer.gcrs2itrs(temp, mjd, result);

    double error = sqrt(
        pow(result[0] - original[0], 2) +
        pow(result[1] - original[1], 2) +
        pow(result[2] - original[2], 2)
    );

    std::cout << std::scientific << std::setprecision(6);
    std::cout << "原始坐标: [" << original[0] << ", " << original[1] << ", " << original[2] << "]" << std::endl;
    std::cout << "往返结果: [" << result[0] << ", " << result[1] << ", " << result[2] << "]" << std::endl;
    std::cout << "误差: " << error << " 米" << std::endl;
    std::cout << "相对误差: " << error/sqrt(original[0]*original[0] +
                                           original[1]*original[1] +
                                           original[2]*original[2]) << std::endl;
}

// 示例6: 批量转换
void example_batch_conversion() {
    std::cout << "\n=== 示例6: 批量坐标转换 ===" << std::endl;

    CoordTransform transformer;
    transformer.initialize("finals2000A.all");

    // 多个地面站
    struct Station {
        std::string name;
        double itrs[3];
    };

    Station stations[] = {
        {"北京", {-2148744.0, 4426641.0, 4044655.0}},
        {"上海", {-2831687.0, 4675733.0, 3275327.0}},
        {"广州", {-2418845.0, 5386140.0, 2405070.0}}
    };

    double mjd = 59000.0;

    std::cout << std::fixed << std::setprecision(1);
    for (const auto& station : stations) {
        double gcrs[3];
        if (transformer.itrs2gcrs(station.itrs, mjd, gcrs)) {
            std::cout << station.name << " GCRS: ["
                     << gcrs[0] << ", "
                     << gcrs[1] << ", "
                     << gcrs[2] << "]" << std::endl;
        }
    }
}

int main() {
    std::cout << "天文坐标转换系统 - 高级示例" << std::endl;
    std::cout << "================================" << std::endl;

    example_ground_station();
    example_satellite_position();
    example_time_series();
    example_intermediate_systems();
    example_accuracy_test();
    example_batch_conversion();

    std::cout << "\n所有示例运行完成!" << std::endl;

    return 0;
}
