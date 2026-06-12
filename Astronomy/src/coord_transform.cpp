#include "coord_transform.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <direct.h>  // for _getcwd on Windows
#include <cctype>    // for std::isspace

CoordTransform::CoordTransform() {
}

CoordTransform::~CoordTransform() {
}

bool CoordTransform::initialize(const std::string& eop_file) {
    return loadEOPData(eop_file);
}

bool CoordTransform::initialize(const std::string& eop_file, const std::string& leap_file) {
    bool eop_ok = loadEOPData(eop_file);
    bool leap_ok = loadLeapSeconds(leap_file);
    return eop_ok && leap_ok;
}

// 安全的字符串转double函数，处理空格和无效值
static double safe_stod(const std::string& str, double default_val = 0.0) {
    // 去除前后空格
    std::string trimmed = str;

    // 去除前导空格
    size_t start = 0;
    while (start < trimmed.length() && std::isspace(trimmed[start])) {
        start++;
    }

    // 去除尾部空格
    size_t end = trimmed.length();
    while (end > start && std::isspace(trimmed[end - 1])) {
        end--;
    }

    if (start >= end) {
        return default_val;
    }

    trimmed = trimmed.substr(start, end - start);

    // 检查是否为空
    if (trimmed.empty()) {
        return default_val;
    }

    try {
        return std::stod(trimmed);
    } catch (const std::invalid_argument&) {
        return default_val;
    } catch (const std::out_of_range&) {
        return default_val;
    }
}

bool CoordTransform::loadEOPData(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开EOP文件: " << filename << std::endl;
        return false;
    }

    std::string line;
    eop_data.clear();

    while (std::getline(file, line)) {
        if (line.length() < 185) continue;

        EOPData eop;

        try {
            // 解析finals2000A.all格式
            // MJD在第8-15列
            std::string mjd_str = line.substr(7, 8);
            eop.mjd = safe_stod(mjd_str);

            // x_pole在第19-27列 (arcsec)
            std::string x_str = line.substr(18, 9);
            eop.x_pole = safe_stod(x_str);

            // y_pole在第38-46列 (arcsec)
            std::string y_str = line.substr(37, 9);
            eop.y_pole = safe_stod(y_str);

            // UT1-UTC在第59-68列 (seconds)
            std::string ut1_str = line.substr(58, 10);
            eop.ut1_utc = safe_stod(ut1_str);

            // LOD在第80-86列 (ms)
            if (line.length() >= 86) {
                std::string lod_str = line.substr(79, 7);
                eop.lod = safe_stod(lod_str, 0.0);
            } else {
                eop.lod = 0.0;
            }

            // dX, dY章动修正 (mas) - 在第98-106和116-124列
            if (line.length() >= 124) {
                std::string dx_str = line.substr(97, 9);
                std::string dy_str = line.substr(116, 9);
                eop.dx = safe_stod(dx_str, 0.0) * 0.001; // mas转arcsec
                eop.dy = safe_stod(dy_str, 0.0) * 0.001;
            } else {
                eop.dx = 0.0;
                eop.dy = 0.0;
            }

            eop_data.push_back(eop);
        } catch (...) {
            // 跳过无法解析的行
            continue;
        }
    }

    file.close();
    std::cout << "成功加载 " << eop_data.size() << " 条EOP数据" << std::endl;
    return !eop_data.empty();
}

double CoordTransform::getTAI_UTC(double mjd) {
    // 如果没有加载闰秒数据，使用默认值
    if (leap_seconds.empty()) {
        return 37.0; // 2017年1月1日后的默认值
    }

    // 查找适用的闰秒值
    for (int i = leap_seconds.size() - 1; i >= 0; i--) {
        if (mjd >= leap_seconds[i].mjd) {
            return leap_seconds[i].tai_utc;
        }
    }

    // 如果MJD早于所有闰秒记录，返回第一个值
    return leap_seconds[0].tai_utc;
}

bool CoordTransform::interpolateEOP(double mjd, EOPData& eop) {
    if (eop_data.empty()) {
        std::cerr << "EOP数据未加载" << std::endl;
        return false;
    }

    // 查找最接近的两个数据点进行线性插值
    size_t idx = 0;
    for (size_t i = 0; i < eop_data.size(); i++) {
        if (eop_data[i].mjd > mjd) {
            idx = (i > 0) ? i - 1 : 0;
            break;
        }
        idx = i;
    }

    if (idx >= eop_data.size() - 1) {
        // 超出范围，使用最后一个数据点
        eop = eop_data.back();
        return true;
    }

    // 线性插值
    const EOPData& eop1 = eop_data[idx];
    const EOPData& eop2 = eop_data[idx + 1];
    double t = (mjd - eop1.mjd) / (eop2.mjd - eop1.mjd);

    eop.mjd = mjd;
    eop.x_pole = eop1.x_pole + t * (eop2.x_pole - eop1.x_pole);
    eop.y_pole = eop1.y_pole + t * (eop2.y_pole - eop1.y_pole);
    eop.ut1_utc = eop1.ut1_utc + t * (eop2.ut1_utc - eop1.ut1_utc);
    eop.lod = eop1.lod + t * (eop2.lod - eop1.lod);
    eop.dx = eop1.dx + t * (eop2.dx - eop1.dx);
    eop.dy = eop1.dy + t * (eop2.dy - eop1.dy);

    return true;
}

bool CoordTransform::loadLeapSeconds(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        char cwd[1024];
        if (getcwd(cwd, sizeof(cwd)) != nullptr) {
            std::cerr << "当前工作目录: " << cwd << std::endl;
        }
        std::cerr << "无法打开闰秒文件: " << filename << std::endl;
        return false;
    }

    std::string line;
    leap_seconds.clear();

    while (std::getline(file, line)) {
        // 跳过注释和空行
        if (line.empty() || line[0] == '#') continue;

        // 解析格式: NTP_Time  DTAI  # comment
        std::istringstream iss(line);
        double ntp_time;
        int dtai;

        if (iss >> ntp_time >> dtai) {
            LeapSecond ls;
            // 将NTP时间转换为MJD
            // MJD = NTP_Time/86400 + 15020
            ls.mjd = ntp_time / 86400.0 + 15020.0;
            ls.tai_utc = dtai;
            leap_seconds.push_back(ls);
        }
    }

    file.close();
    std::cout << "成功加载 " << leap_seconds.size() << " 条闰秒数据" << std::endl;
    return !leap_seconds.empty();
}

void CoordTransform::utc2tt(double utc_mjd, double* tt_mjd1, double* tt_mjd2) {
    // TT = UTC + (TAI-UTC) + (TT-TAI)
    // TT-TAI = 32.184s (固定值)
    double tai_utc = getTAI_UTC(utc_mjd);
    double tt_utc = tai_utc + ERFA_TTMTAI; // TAI-UTC + 32.184s

    *tt_mjd1 = ERFA_DJM0;
    *tt_mjd2 = utc_mjd + tt_utc / 86400.0;
}

void CoordTransform::utc2ut1(double utc_mjd, double dut1, double* ut1_mjd1, double* ut1_mjd2) {
    *ut1_mjd1 = ERFA_DJM0;
    *ut1_mjd2 = utc_mjd + dut1 / 86400.0;
}

bool CoordTransform::gcrs2itrs(const double gcrs[3], double utc_mjd, double itrs[3]) {
    EOPData eop;
    if (!interpolateEOP(utc_mjd, eop)) {
        return false;
    }

    // 转换极移参数为弧度
    double xp = eop.x_pole * ERFA_DAS2R;
    double yp = eop.y_pole * ERFA_DAS2R;

    // 计算TT和UT1时间
    double tt1, tt2, ut1_1, ut1_2;
    utc2tt(utc_mjd, &tt1, &tt2);
    utc2ut1(utc_mjd, eop.ut1_utc, &ut1_1, &ut1_2);

    // 使用ERFA的C2T06A函数计算GCRS到ITRS的转换矩阵
    double rc2t[3][3];
    eraC2t06a(tt1, tt2, ut1_1, ut1_2, xp, yp, rc2t);

    // 创建临时变量用于转换
    double gcrs_temp[3] = {gcrs[0], gcrs[1], gcrs[2]};

    // 应用转换矩阵
    eraRxp(rc2t, gcrs_temp, itrs);

    return true;
}

bool CoordTransform::itrs2gcrs(const double itrs[3], double utc_mjd, double gcrs[3]) {
    EOPData eop;
    if (!interpolateEOP(utc_mjd, eop)) {
        return false;
    }

    double xp = eop.x_pole * ERFA_DAS2R;
    double yp = eop.y_pole * ERFA_DAS2R;

    double tt1, tt2, ut1_1, ut1_2;
    utc2tt(utc_mjd, &tt1, &tt2);
    utc2ut1(utc_mjd, eop.ut1_utc, &ut1_1, &ut1_2);

    // 计算转换矩阵
    double rc2t[3][3];
    eraC2t06a(tt1, tt2, ut1_1, ut1_2, xp, yp, rc2t);

    // 转置矩阵进行逆转换
    double rt2c[3][3];
    eraTr(rc2t, rt2c);

    // 创建临时变量
    double itrs_temp[3] = {itrs[0], itrs[1], itrs[2]};

    // 应用转换
    eraRxp(rt2c, itrs_temp, gcrs);

    return true;
}

bool CoordTransform::gcrs2cirs(const double gcrs[3], double tt_mjd, double cirs[3]) {
    double tt1, tt2;
    utc2tt(tt_mjd, &tt1, &tt2);

    // 使用C2I06A计算GCRS到CIRS的转换矩阵
    double rc2i[3][3];
    eraC2i06a(tt1, tt2, rc2i);

    // 创建临时变量
    double gcrs_temp[3] = {gcrs[0], gcrs[1], gcrs[2]};

    // 应用转换
    eraRxp(rc2i, gcrs_temp, cirs);

    return true;
}

bool CoordTransform::cirs2gcrs(const double cirs[3], double tt_mjd, double gcrs[3]) {
    double tt1, tt2;
    utc2tt(tt_mjd, &tt1, &tt2);

    double rc2i[3][3];
    eraC2i06a(tt1, tt2, rc2i);

    // 转置进行逆转换
    double ri2c[3][3];
    eraTr(rc2i, ri2c);

    // 创建临时变量
    double cirs_temp[3] = {cirs[0], cirs[1], cirs[2]};

    eraRxp(ri2c, cirs_temp, gcrs);

    return true;
}

bool CoordTransform::cirs2tirs(const double cirs[3], double ut1_mjd, double tirs[3]) {
    double ut1_1, ut1_2;
    ut1_1 = ERFA_DJM0;
    ut1_2 = ut1_mjd;

    // 计算地球自转角
    double era = eraEra00(ut1_1, ut1_2);

    // 构建旋转矩阵
    double r[3][3];
    eraIr(r);
    eraRz(era, r);

    // 创建临时变量
    double cirs_temp[3] = {cirs[0], cirs[1], cirs[2]};

    // 应用转换
    eraRxp(r, cirs_temp, tirs);

    return true;
}

bool CoordTransform::tirs2cirs(const double tirs[3], double ut1_mjd, double cirs[3]) {
    double ut1_1, ut1_2;
    ut1_1 = ERFA_DJM0;
    ut1_2 = ut1_mjd;

    double era = eraEra00(ut1_1, ut1_2);

    // 构建旋转矩阵（负角度）
    double r[3][3];
    eraIr(r);
    eraRz(-era, r);

    // 创建临时变量
    double tirs_temp[3] = {tirs[0], tirs[1], tirs[2]};

    eraRxp(r, tirs_temp, cirs);

    return true;
}

bool CoordTransform::tirs2itrs(const double tirs[3], double utc_mjd, double itrs[3]) {
    EOPData eop;
    if (!interpolateEOP(utc_mjd, eop)) {
        return false;
    }

    double xp = eop.x_pole * ERFA_DAS2R;
    double yp = eop.y_pole * ERFA_DAS2R;

    double tt1, tt2;
    utc2tt(utc_mjd, &tt1, &tt2);

    // 计算s'参数
    double sp = eraSp00(tt1, tt2);

    // 构建极移矩阵
    double rpom[3][3];
    eraPom00(xp, yp, sp, rpom);

    // 创建临时变量
    double tirs_temp[3] = {tirs[0], tirs[1], tirs[2]};

    // 应用转换
    eraRxp(rpom, tirs_temp, itrs);

    return true;
}

bool CoordTransform::itrs2tirs(const double itrs[3], double utc_mjd, double tirs[3]) {
    EOPData eop;
    if (!interpolateEOP(utc_mjd, eop)) {
        return false;
    }

    double xp = eop.x_pole * ERFA_DAS2R;
    double yp = eop.y_pole * ERFA_DAS2R;

    double tt1, tt2;
    utc2tt(utc_mjd, &tt1, &tt2);

    double sp = eraSp00(tt1, tt2);

    double rpom[3][3];
    eraPom00(xp, yp, sp, rpom);

    // 转置进行逆转换
    double rpom_t[3][3];
    eraTr(rpom, rpom_t);

    // 创建临时变量
    double itrs_temp[3] = {itrs[0], itrs[1], itrs[2]};

    eraRxp(rpom_t, itrs_temp, tirs);

    return true;
}

bool CoordTransform::transform(
    const double pos_in[3],
    CoordSystem from_sys,
    CoordSystem to_sys,
    double utc_mjd,
    double pos_out[3]
) {
    // 如果源和目标相同，直接复制
    if (from_sys == to_sys) {
        pos_out[0] = pos_in[0];
        pos_out[1] = pos_in[1];
        pos_out[2] = pos_in[2];
        return true;
    }

    // 定义转换路径: GCRS <-> CIRS <-> TIRS <-> ITRS
    double temp[3];

    // 先转换到GCRS
    if (from_sys == ITRS) {
        if (!itrs2gcrs(pos_in, utc_mjd, temp)) return false;
    } else if (from_sys == CIRS) {
        if (!cirs2gcrs(pos_in, utc_mjd, temp)) return false;
    } else if (from_sys == TIRS) {
        double cirs_temp[3];
        EOPData eop;
        if (!interpolateEOP(utc_mjd, eop)) return false;
        double ut1_1, ut1_2;
        utc2ut1(utc_mjd, eop.ut1_utc, &ut1_1, &ut1_2);
        if (!tirs2cirs(pos_in, ut1_2, cirs_temp)) return false;
        if (!cirs2gcrs(cirs_temp, utc_mjd, temp)) return false;
    } else {
        temp[0] = pos_in[0];
        temp[1] = pos_in[1];
        temp[2] = pos_in[2];
    }

    // 从GCRS转换到目标坐标系
    if (to_sys == GCRS || to_sys == ICRS) {
        pos_out[0] = temp[0];
        pos_out[1] = temp[1];
        pos_out[2] = temp[2];
        return true;
    } else if (to_sys == ITRS) {
        return gcrs2itrs(temp, utc_mjd, pos_out);
    } else if (to_sys == CIRS) {
        return gcrs2cirs(temp, utc_mjd, pos_out);
    } else if (to_sys == TIRS) {
        double cirs_temp[3];
        if (!gcrs2cirs(temp, utc_mjd, cirs_temp)) return false;
        EOPData eop;
        if (!interpolateEOP(utc_mjd, eop)) return false;
        double ut1_1, ut1_2;
        utc2ut1(utc_mjd, eop.ut1_utc, &ut1_1, &ut1_2);
        return cirs2tirs(cirs_temp, ut1_2, pos_out);
    }

    return false;
}

bool CoordTransform::gcrs2itrsPV(const double pv_gcrs[2][3], double utc_mjd, double pv_itrs[2][3]) {
    EOPData eop;
    if (!interpolateEOP(utc_mjd, eop)) {
        return false;
    }

    // 转换极移参数为弧度
    double xp = eop.x_pole * ERFA_DAS2R;
    double yp = eop.y_pole * ERFA_DAS2R;

    // 计算TT和UT1时间
    double tt1, tt2, ut1_1, ut1_2;
    utc2tt(utc_mjd, &tt1, &tt2);
    utc2ut1(utc_mjd, eop.ut1_utc, &ut1_1, &ut1_2);

    // 使用ERFA的C2T06A函数计算GCRS到ITRS的转换矩阵
    double rc2t[3][3];
    eraC2t06a(tt1, tt2, ut1_1, ut1_2, xp, yp, rc2t);

    // 创建临时变量
    double pv_temp[2][3];
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            pv_temp[i][j] = pv_gcrs[i][j];
        }
    }

    // 转换位置
    eraRxp(rc2t, pv_temp[0], pv_itrs[0]);

    // 转换速度需要考虑地球自转
    // v_ITRS = R * v_GCRS - omega × r_ITRS
    // 其中 omega 是地球自转角速度向量

    // 先转换速度向量
    double v_rotated[3];
    eraRxp(rc2t, pv_temp[1], v_rotated);

    // 计算地球自转角速度 (rad/s)
    double omega = 7.292115e-5; // 地球自转角速度 (rad/s)
    double omega_vec[3] = {0.0, 0.0, omega};

    // 计算 omega × r_ITRS
    double omega_cross_r[3];
    eraPxp(omega_vec, pv_itrs[0], omega_cross_r);

    // v_ITRS = v_rotated - omega × r_ITRS
    for (int i = 0; i < 3; i++) {
        pv_itrs[1][i] = v_rotated[i] - omega_cross_r[i];
    }

    return true;
}

bool CoordTransform::itrs2gcrsPV(const double pv_itrs[2][3], double utc_mjd, double pv_gcrs[2][3]) {
    EOPData eop;
    if (!interpolateEOP(utc_mjd, eop)) {
        return false;
    }

    double xp = eop.x_pole * ERFA_DAS2R;
    double yp = eop.y_pole * ERFA_DAS2R;

    double tt1, tt2, ut1_1, ut1_2;
    utc2tt(utc_mjd, &tt1, &tt2);
    utc2ut1(utc_mjd, eop.ut1_utc, &ut1_1, &ut1_2);

    // 计算转换矩阵
    double rc2t[3][3];
    eraC2t06a(tt1, tt2, ut1_1, ut1_2, xp, yp, rc2t);

    // 转置矩阵进行逆转换
    double rt2c[3][3];
    eraTr(rc2t, rt2c);

    // 创建临时变量
    double pv_temp[2][3];
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            pv_temp[i][j] = pv_itrs[i][j];
        }
    }

    // 转换位置
    eraRxp(rt2c, pv_temp[0], pv_gcrs[0]);

    // 转换速度: v_GCRS = R^T * (v_ITRS + omega × r_ITRS)
    double omega = 7.292115e-5;
    double omega_vec[3] = {0.0, 0.0, omega};

    // 计算 omega × r_ITRS
    double omega_cross_r[3];
    eraPxp(omega_vec, pv_temp[0], omega_cross_r);

    // v_ITRS + omega × r_ITRS
    double v_corrected[3];
    for (int i = 0; i < 3; i++) {
        v_corrected[i] = pv_temp[1][i] + omega_cross_r[i];
    }

    // 应用转置矩阵
    eraRxp(rt2c, v_corrected, pv_gcrs[1]);

    return true;
}

bool CoordTransform::transformPV(
    const double pv_in[2][3],
    CoordSystem from_sys,
    CoordSystem to_sys,
    double utc_mjd,
    double pv_out[2][3]
) {
    // 如果源和目标相同，直接复制
    if (from_sys == to_sys) {
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 3; j++) {
                pv_out[i][j] = pv_in[i][j];
            }
        }
        return true;
    }

    // 目前主要支持 GCRS <-> ITRS 的速度转换
    if (from_sys == GCRS && to_sys == ITRS) {
        return gcrs2itrsPV(pv_in, utc_mjd, pv_out);
    } else if (from_sys == ITRS && to_sys == GCRS) {
        return itrs2gcrsPV(pv_in, utc_mjd, pv_out);
    }

    // 对于其他坐标系，先转换位置，速度暂时只做旋转
    // 这是简化处理，完整实现需要考虑各个坐标系的时间导数
    double temp_pv[2][3];

    if (from_sys == ITRS) {
        if (!itrs2gcrsPV(pv_in, utc_mjd, temp_pv)) return false;
    } else {
        // 对于其他坐标系，先复制到临时变量
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 3; j++) {
                temp_pv[i][j] = pv_in[i][j];
            }
        }
    }

    if (to_sys == ITRS) {
        return gcrs2itrsPV(temp_pv, utc_mjd, pv_out);
    } else {
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 3; j++) {
                pv_out[i][j] = temp_pv[i][j];
            }
        }
        return true;
    }
}
