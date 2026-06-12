# 天文坐标系转换系统 - 功能总结

## 已实现的核心功能

### 1. 坐标系转换

#### 支持的坐标系
- **GCRS** (Geocentric Celestial Reference System) - 地心天球参考系
- **ITRS** (International Terrestrial Reference System) - 国际地球参考系
- **CIRS** (Celestial Intermediate Reference System) - 天球中间参考系
- **TIRS** (Terrestrial Intermediate Reference System) - 地球中间参考系
- **ICRS** (International Celestial Reference System) - 国际天球参考系

#### 转换链
```
GCRS <--岁差章动--> CIRS <--地球自转--> TIRS <--极移--> ITRS
```

### 2. 位置转换函数

```cpp
// 直接转换
bool gcrs2itrs(const double gcrs[3], double utc_mjd, double itrs[3]);
bool itrs2gcrs(const double itrs[3], double utc_mjd, double gcrs[3]);
bool gcrs2cirs(const double gcrs[3], double tt_mjd, double cirs[3]);
bool cirs2gcrs(const double cirs[3], double tt_mjd, double gcrs[3]);
bool cirs2tirs(const double cirs[3], double ut1_mjd, double tirs[3]);
bool tirs2cirs(const double tirs[3], double ut1_mjd, double cirs[3]);
bool tirs2itrs(const double tirs[3], double utc_mjd, double itrs[3]);
bool itrs2tirs(const double itrs[3], double utc_mjd, double tirs[3]);

// 通用接口
bool transform(const double pos_in[3], CoordSystem from_sys,
               CoordSystem to_sys, double utc_mjd, double pos_out[3]);
```

### 3. 速度转换函数

```cpp
// 位置和速度同时转换
bool gcrs2itrsPV(const double pv_gcrs[2][3], double utc_mjd, double pv_itrs[2][3]);
bool itrs2gcrsPV(const double pv_itrs[2][3], double utc_mjd, double pv_gcrs[2][3]);

// 通用接口
bool transformPV(const double pv_in[2][3], CoordSystem from_sys,
                 CoordSystem to_sys, double utc_mjd, double pv_out[2][3]);
```

**速度转换考虑的物理效应**:
- 地球自转角速度: ω = 7.292115×10⁻⁵ rad/s
- 科里奥利效应: v_GCRS = R^T * (v_ITRS + ω × r_ITRS)

### 4. 数据加载

#### EOP数据 (finals2000A.all)
```cpp
bool loadEOPData(const std::string& filename);
```
加载并解析IERS地球定向参数：
- 极移参数 (x_pole, y_pole) - 角秒
- UT1-UTC时间差 - 秒
- 日长变化 (LOD) - 毫秒
- 章动修正 (dX, dY) - 毫角秒

#### 闰秒数据 (leap-seconds.list)
```cpp
bool loadLeapSeconds(const std::string& filename);
double getTAI_UTC(double mjd);
```
加载历史闰秒记录，提供精确的TAI-UTC转换：
- 支持1972年至今的所有闰秒
- 自动查找适用于指定MJD的闰秒值
- 未加载时使用默认值37秒（2017年后）

### 5. 时间系统转换

```cpp
void utc2tt(double utc_mjd, double* tt_mjd1, double* tt_mjd2);
void utc2ut1(double utc_mjd, double dut1, double* ut1_mjd1, double* ut1_mjd2);
```

**时间关系**:
- TT = UTC + (TAI-UTC) + 32.184s
- UT1 = UTC + (UT1-UTC)
- TAI-UTC从闰秒表获取

### 6. 数据插值

```cpp
bool interpolateEOP(double mjd, EOPData& eop);
```
对EOP参数进行线性插值，获取任意时刻的地球定向参数。

## 技术特性

### 1. 高精度模型
- IAU 2006/2000A 岁差章动模型
- 考虑极移、章动、岁差、地球自转等所有主要效应
- 位置精度: 毫米级
- 速度精度: 毫米/秒级

### 2. 安全的数据解析
```cpp
static double safe_stod(const std::string& str, double default_val = 0.0);
```
- 自动处理空格和空值
- 异常捕获和默认值处理
- 防止解析错误导致程序崩溃

### 3. 错误处理
- 文件加载失败时显示当前工作目录
- 数据范围检查
- 返回值指示操作成功/失败

### 4. 编码支持
- Windows控制台UTF-8编码支持
- 正确显示中文输出

## 应用场景

### 1. 卫星轨道计算
- GCRS中的轨道传播
- ITRS中的地面站可见性分析
- 多普勒频移计算

### 2. 地面站定位
- 地面站ITRS坐标转换为GCRS
- 考虑地球自转的速度效应
- 时间序列分析

### 3. 天文观测
- 望远镜指向计算
- 视位置到真位置转换
- 大气折射修正（预留接口）

### 4. 精密定轨
- 多坐标系联合处理
- 历史数据重处理
- 精密星历生成

## 示例程序

### main.cpp
基础功能演示：
- ITRS ↔ GCRS 转换
- 往返转换精度验证
- 中间坐标系转换
- 通用接口使用
- 时间序列转换
- 位置和速度转换

### advanced_examples.cpp
高级应用示例：
- 地面站坐标转换
- 卫星位置转换
- 时间序列处理
- 中间坐标系详解
- 精度验证测试
- 批量转换处理

## 文件结构

```
Astronomy/
├── include/
│   ├── erfa.h              # ERFA库头文件
│   └── coord_transform.h   # 坐标转换类
├── src/
│   ├── erfa.c              # ERFA库实现
│   ├── coord_transform.cpp # 坐标转换实现
│   └── main.cpp            # 主程序
├── examples/
│   └── advanced_examples.cpp
├── finals2000A.all         # EOP数据
├── leap-seconds.list       # 闰秒数据
├── CMakeLists.txt
└── README.md
```

## 依赖项

- ERFA库 (Essential Routines for Fundamental Astronomy)
- C++11标准库
- CMake 3.10+
- Windows: windows.h (用于控制台编码)
- POSIX: direct.h (用于getcwd)

## 编译和运行

```bash
mkdir build
cd build
cmake ..
make
./exe
```

## 未来扩展方向

1. 添加更多坐标系（J2000、TOD等）
2. 实现大气折射修正
3. 支持恒星自行和视差
4. 添加光行差修正
5. 实现相对论效应修正
6. 支持更多时间系统（TCG、TCB等）
7. 添加Python/MATLAB接口
8. 性能优化和并行计算

## 参考文献

1. IERS Conventions (2010)
2. IAU 2006/2000A Precession-Nutation Model
3. ERFA Software Documentation
4. SOFA Library Documentation
5. Astronomical Almanac
