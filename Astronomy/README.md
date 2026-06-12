# 天文坐标系转换系统

基于ERFA库和finals2000A.all数据实现的高精度天文坐标系转换工具。

## 功能特性

- 支持多种天文坐标系之间的转换
- 使用IERS EOP数据进行高精度计算
- 基于IAU 2006/2000A岁差章动模型

## 支持的坐标系

1. **GCRS** (Geocentric Celestial Reference System) - 地心天球参考系
2. **ITRS** (International Terrestrial Reference System) - 国际地球参考系
3. **CIRS** (Celestial Intermediate Reference System) - 天球中间参考系
4. **TIRS** (Terrestrial Intermediate Reference System) - 地球中间参考系
5. **ICRS** (International Celestial Reference System) - 国际天球参考系

## 编译方法

```bash
mkdir build
cd build
cmake ..
make
```

## 使用示例

### 基本用法

```cpp
#include "coord_transform.h"

// 初始化转换器（带闰秒数据）
CoordTransform transformer;
transformer.initialize("finals2000A.all", "leap-seconds.list");

// 或者仅使用EOP数据（使用默认TAI-UTC值）
// transformer.initialize("finals2000A.all");

// ITRS坐标 (单位: 米)
double itrs_pos[3] = {-2148744.0, 4426641.0, 4044655.0};

// 转换到GCRS
double gcrs_pos[3];
double utc_mjd = 58849.0;  // 2020-01-01 00:00:00 UTC
transformer.itrs2gcrs(itrs_pos, utc_mjd, gcrs_pos);
```

### 使用通用接口

```cpp
double result[3];
transformer.transform(
    itrs_pos,      // 输入坐标
    ITRS,          // 源坐标系
    GCRS,          // 目标坐标系
    utc_mjd,       // UTC时间(MJD)
    result         // 输出坐标
);
```

### 位置和速度转换

```cpp
// 输入: 位置和速度 [pos, vel]
double pv_itrs[2][3] = {
    {-2148744.0, 4426641.0, 4044655.0},  // 位置 (m)
    {0.0, 0.0, 0.0}                       // 速度 (m/s)
};

double pv_gcrs[2][3];
transformer.itrs2gcrsPV(pv_itrs, utc_mjd, pv_gcrs);

// 或使用通用接口
transformer.transformPV(pv_itrs, ITRS, GCRS, utc_mjd, pv_gcrs);
```

## API说明

### 初始化

```cpp
// 方法1: 加载EOP和闰秒数据（推荐）
bool initialize(const std::string& eop_file, const std::string& leap_file);

// 方法2: 仅加载EOP数据（使用默认闰秒值37秒）
bool initialize(const std::string& eop_file);
```

加载闰秒数据可以提供精确的UTC到TT时间转换，特别是对于历史数据分析。

### 坐标转换函数

```cpp
// GCRS <-> ITRS
bool gcrs2itrs(const double gcrs[3], double utc_mjd, double itrs[3]);
bool itrs2gcrs(const double itrs[3], double utc_mjd, double gcrs[3]);

// GCRS <-> CIRS
bool gcrs2cirs(const double gcrs[3], double tt_mjd, double cirs[3]);
bool cirs2gcrs(const double cirs[3], double tt_mjd, double gcrs[3]);

// CIRS <-> TIRS
bool cirs2tirs(const double cirs[3], double ut1_mjd, double tirs[3]);
bool tirs2cirs(const double tirs[3], double ut1_mjd, double cirs[3]);

// TIRS <-> ITRS
bool tirs2itrs(const double tirs[3], double utc_mjd, double itrs[3]);
bool itrs2tirs(const double itrs[3], double utc_mjd, double tirs[3]);

// 通用转换接口
bool transform(const double pos_in[3], CoordSystem from_sys,
               CoordSystem to_sys, double utc_mjd, double pos_out[3]);
```

### 位置和速度转换函数

```cpp
// GCRS <-> ITRS (位置和速度)
bool gcrs2itrsPV(const double pv_gcrs[2][3], double utc_mjd, double pv_itrs[2][3]);
bool itrs2gcrsPV(const double pv_itrs[2][3], double utc_mjd, double pv_gcrs[2][3]);

// 通用位置速度转换接口
bool transformPV(const double pv_in[2][3], CoordSystem from_sys,
                 CoordSystem to_sys, double utc_mjd, double pv_out[2][3]);
```

**注意**: 速度转换考虑了地球自转效应。对于固定在地球上的点（ITRS速度为0），转换到GCRS后会有非零速度，这是由于地球自转引起的。

## 坐标系转换链

```
GCRS (天球) <--岁差章动--> CIRS <--地球自转--> TIRS <--极移--> ITRS (地球)
```

## 数据文件

### finals2000A.all

IERS提供的地球定向参数(EOP)数据文件，包含：
- 极移参数 (x_pole, y_pole)
- UT1-UTC时间差
- 日长变化 (LOD)
- 章动修正 (dX, dY)

下载地址: https://datacenter.iers.org/data/latestVersion/finals2000A.all

### leap-seconds.list

IERS提供的闰秒数据文件，包含：
- 历史闰秒记录（1972年至今）
- TAI-UTC时间差
- 用于精确的UTC到TT时间转换

下载地址: https://hpiers.obspm.fr/iers/bul/bulc/ntp/leap-seconds.list

**注意**: 如果不提供闰秒文件，系统将使用默认值（37秒，适用于2017年后）。对于历史数据分析，建议使用完整的闰秒文件以获得准确的时间转换。

### leap-seconds.dat

闰秒数据文件，包含TAI-UTC的历史值。格式：
```
# MJD TAI-UTC(seconds)
41317.0 10
41499.0 11
...
57754.0 37
```

如果不提供闰秒文件，系统将使用默认值37秒（2017年后）。

## 技术细节

### 时间系统

- **UTC**: 协调世界时
- **UT1**: 世界时1 (与地球自转相关)
- **TT**: 地球时 (TT = UTC + ΔAT + 32.184s)

### 转换精度

- 位置精度: 毫米级
- 使用IAU 2006/2000A模型
- 考虑极移、章动、岁差等效应

## 依赖项

- ERFA库 (Essential Routines for Fundamental Astronomy)
- C++11标准库
- CMake 3.10+

## 参考文献

1. IERS Conventions (2010)
2. IAU 2006/2000A Precession-Nutation Model
3. ERFA Software Documentation

## 许可证

本项目使用ERFA库，遵循其许可证条款。
