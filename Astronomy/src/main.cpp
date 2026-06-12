#include "coord_transform.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <windows.h>

void printVector(const std::string& name, const double vec[3]) {
    std::cout << std::fixed << std::setprecision(6);
    std::cout << name << ": ["
              << vec[0] << ", "
              << vec[1] << ", "
              << vec[2] << "]" << std::endl;
}

int main() {
    // Set console to UTF-8 encoding
    SetConsoleOutputCP(CP_UTF8);

    std::cout << "========================================" << std::endl;
    std::cout << "天文坐标系转换系统" << std::endl;
    std::cout << "基于ERFA库和finals2000A.all数据" << std::endl;
    std::cout << "========================================\n" << std::endl;

    // 初始化坐标转换器
    CoordTransform transformer;

    std::cout << "正在加载EOP数据和闰秒数据..." << std::endl;
    if (!transformer.initialize("finals2000A.all", "leap-seconds.list")) {
        std::cerr << "错误: 无法加载数据文件" << std::endl;
        std::cerr << "尝试仅加载EOP数据..." << std::endl;
        if (!transformer.initialize("finals2000A.all")) {
            std::cerr << "错误: 无法加载EOP数据文件" << std::endl;
            return 1;
        }
    }
    std::cout << std::endl;

    // 测试时间: 2020年1月1日 00:00:00 UTC
    // MJD = 58849.0
    double utc_mjd = 58849.0;

    std::cout << "测试时间: MJD " << utc_mjd
              << " (2020-01-01 00:00:00 UTC)" << std::endl;
    std::cout << std::endl;

    // 测试位置: 北京站的ITRS坐标 (单位: 米)
    // 经度: 116.2°E, 纬度: 40.2°N, 高度: 50m
    double itrs_pos[3] = {-2148744.0, 4426641.0, 4044655.0};

    std::cout << "========================================" << std::endl;
    std::cout << "示例1: ITRS -> GCRS 转换" << std::endl;
    std::cout << "========================================" << std::endl;
    printVector("输入 (ITRS)", itrs_pos);

    double gcrs_pos[3];
    if (transformer.itrs2gcrs(itrs_pos, utc_mjd, gcrs_pos)) {
        printVector("输出 (GCRS)", gcrs_pos);
    } else {
        std::cerr << "转换失败!" << std::endl;
    }
    std::cout << std::endl;

    // 反向转换验证
    std::cout << "========================================" << std::endl;
    std::cout << "示例2: GCRS -> ITRS 转换 (验证)" << std::endl;
    std::cout << "========================================" << std::endl;
    printVector("输入 (GCRS)", gcrs_pos);

    double itrs_back[3];
    if (transformer.gcrs2itrs(gcrs_pos, utc_mjd, itrs_back)) {
        printVector("输出 (ITRS)", itrs_back);

        // 计算误差
        double error = sqrt(
            pow(itrs_back[0] - itrs_pos[0], 2) +
            pow(itrs_back[1] - itrs_pos[1], 2) +
            pow(itrs_back[2] - itrs_pos[2], 2)
        );
        std::cout << "往返转换误差: " << std::scientific
                  << error << " 米" << std::endl;
    } else {
        std::cerr << "转换失败!" << std::endl;
    }
    std::cout << std::endl;

    // 测试中间坐标系
    std::cout << "========================================" << std::endl;
    std::cout << "示例3: 通过中间坐标系转换" << std::endl;
    std::cout << "========================================" << std::endl;

    double cirs_pos[3];
    if (transformer.gcrs2cirs(gcrs_pos, utc_mjd, cirs_pos)) {
        printVector("GCRS -> CIRS", cirs_pos);
    }

    double tirs_pos[3];
    if (transformer.transform(itrs_pos, ITRS, TIRS, utc_mjd, tirs_pos)) {
        printVector("ITRS -> TIRS", tirs_pos);
    }
    std::cout << std::endl;

    // 通用转换接口测试
    std::cout << "========================================" << std::endl;
    std::cout << "示例4: 使用通用转换接口" << std::endl;
    std::cout << "========================================" << std::endl;

    CoordSystem systems[] = {ITRS, GCRS, CIRS, TIRS};
    std::string names[] = {"ITRS", "GCRS", "CIRS", "TIRS"};

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            if (i == j) continue;

            double result[3];
            if (transformer.transform(itrs_pos, systems[i], systems[j],
                                     utc_mjd, result)) {
                std::cout << names[i] << " -> " << names[j] << ": ";
                std::cout << "[" << std::fixed << std::setprecision(3)
                         << result[0] << ", "
                         << result[1] << ", "
                         << result[2] << "]" << std::endl;
            }
        }
    }
    std::cout << std::endl;

    // 测试不同时刻的转换
    std::cout << "========================================" << std::endl;
    std::cout << "示例5: 不同时刻的ITRS->GCRS转换" << std::endl;
    std::cout << "========================================" << std::endl;

    double test_mjds[] = {58849.0, 58849.5, 58850.0, 58850.5};
    for (int i = 0; i < 4; i++) {
        double result[3];
        if (transformer.itrs2gcrs(itrs_pos, test_mjds[i], result)) {
            std::cout << "MJD " << test_mjds[i] << ": "
                     << "[" << std::fixed << std::setprecision(3)
                     << result[0] << ", "
                     << result[1] << ", "
                     << result[2] << "]" << std::endl;
        }
    }
    std::cout << std::endl;

    std::cout << "========================================" << std::endl;
    std::cout << "测试完成!" << std::endl;
    std::cout << "========================================" << std::endl;

    // 测试速度转换
    std::cout << "\n========================================" << std::endl;
    std::cout << "示例6: 位置和速度转换 (ITRS -> GCRS)" << std::endl;
    std::cout << "========================================" << std::endl;

    // 地面站位置和速度 (速度为0，因为固定在地球上)
    double pv_itrs[2][3] = {
        {-2148744.0, 4426641.0, 4044655.0},  // 位置 (m)
        {0.0, 0.0, 0.0}                       // 速度 (m/s)
    };

    double pv_gcrs[2][3];
    if (transformer.itrs2gcrsPV(pv_itrs, utc_mjd, pv_gcrs)) {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "输入 ITRS 位置: ["
                  << pv_itrs[0][0] << ", "
                  << pv_itrs[0][1] << ", "
                  << pv_itrs[0][2] << "]" << std::endl;
        std::cout << "输入 ITRS 速度: ["
                  << pv_itrs[1][0] << ", "
                  << pv_itrs[1][1] << ", "
                  << pv_itrs[1][2] << "]" << std::endl;
        std::cout << "\n输出 GCRS 位置: ["
                  << pv_gcrs[0][0] << ", "
                  << pv_gcrs[0][1] << ", "
                  << pv_gcrs[0][2] << "]" << std::endl;
        std::cout << "输出 GCRS 速度: ["
                  << pv_gcrs[1][0] << ", "
                  << pv_gcrs[1][1] << ", "
                  << pv_gcrs[1][2] << "]" << std::endl;

        // 计算速度大小
        double v_mag = sqrt(pv_gcrs[1][0]*pv_gcrs[1][0] +
                           pv_gcrs[1][1]*pv_gcrs[1][1] +
                           pv_gcrs[1][2]*pv_gcrs[1][2]);
        std::cout << "GCRS速度大小: " << v_mag << " m/s" << std::endl;
    }
    std::cout << std::endl;

    // 测试卫星速度转换
    std::cout << "========================================" << std::endl;
    std::cout << "示例7: 卫星位置和速度转换 (GCRS -> ITRS)" << std::endl;
    std::cout << "========================================" << std::endl;

    // GPS卫星在GCRS中的位置和速度
    double sat_pv_gcrs[2][3] = {
        {20000000.0, 15000000.0, 10000000.0},  // 位置 (m)
        {-1500.0, 2000.0, 500.0}                // 速度 (m/s)
    };

    double sat_pv_itrs[2][3];
    if (transformer.gcrs2itrsPV(sat_pv_gcrs, utc_mjd, sat_pv_itrs)) {
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "输入 GCRS 位置: ["
                  << sat_pv_gcrs[0][0] << ", "
                  << sat_pv_gcrs[0][1] << ", "
                  << sat_pv_gcrs[0][2] << "]" << std::endl;
        std::cout << "输入 GCRS 速度: ["
                  << sat_pv_gcrs[1][0] << ", "
                  << sat_pv_gcrs[1][1] << ", "
                  << sat_pv_gcrs[1][2] << "]" << std::endl;
        std::cout << "\n输出 ITRS 位置: ["
                  << sat_pv_itrs[0][0] << ", "
                  << sat_pv_itrs[0][1] << ", "
                  << sat_pv_itrs[0][2] << "]" << std::endl;
        std::cout << "输出 ITRS 速度: ["
                  << sat_pv_itrs[1][0] << ", "
                  << sat_pv_itrs[1][1] << ", "
                  << sat_pv_itrs[1][2] << "]" << std::endl;
    }
    std::cout << std::endl;

    std::cout << "========================================" << std::endl;
    std::cout << "所有测试完成!" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
