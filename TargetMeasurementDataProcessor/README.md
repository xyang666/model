# TargetMeasurementDataProcessingModel

## 概述

目标测量数据处理模型，用于处理雷达、光电（EO）和红外（IR）传感器测量数据，
通过误差传播与目标跟踪滤波器输出目标在世界系下的位置、速度，
以及误差椭球、卡方统计和跟踪质量等指标。

对外提供纯 C API，内部使用 C++ 与 Eigen 线性代数库实现。

## 目录结构

```
include/
├── TM_Interface.h              C API 类型与函数声明
├── Plugin.h                    插件桥接类
├── Framework.h                 引擎调度接口
├── tracker/
│   ├── Tracker.h               跟踪器抽象基类
│   ├── RadarTracker.h          雷达跟踪器
│   ├── EOTracker.h             光电跟踪器
│   └── IRTracker.h             红外跟踪器
├── filter/
│   ├── FilterBase.h            滤波器统一抽象基类（回调式接口）
│   ├── FilterTypes.h           滤波器类型与状态枚举
│   ├── KalmanFilterND.h        N 维线性 Kalman 滤波器
│   ├── ExtendedKalmanFilter.h  扩展 Kalman 滤波器（支持非线性）
│   ├── UnscentedKalmanFilter.h 无迹 Kalman 滤波器（Sigma 点）
│   ├── QualityEstimator.h      误差评估组件
│   └── ...
└── utils/
    ├── CoordinateTransform.h   参考系变换（CoordTransform、ecefToNed 等）
    └── AttitudeTransform.h     欧拉角 → 旋转矩阵

src/
├── main.cpp                    演示程序
├── Plugin.cpp                  C API 实现
├── Framework.cpp               引擎调度桩
├── CoordinateTransform.cpp     LLA↔ECEF↔NED 变换
├── ProcessingPipeline.cpp      ProcessMeasurement 管线
└── tracker/
    ├── Tracker.cpp             基类构造/初始化
    ├── TrackerUtils.cpp        输出填充 + 查询方法
    ├── RadarTracker.cpp        雷达观测模型（RBE → 世界系）
    ├── EOTracker.cpp           光电观测模型（占位）
    └── IRTracker.cpp           红外观测模型（占位）
```

## 数据流

```
computeObservation()      ← 子类实现：z、H、R
       │
       v
filter->predict(dt)       ← 状态外推（调用 setStateFunc 的回调）
       │
       v
filter->setR(R)           ← 每帧更新测量噪声
filter->update(z)         ← 测量更新（调用 setMeasureFunc 的回调）
       │
       v
quality.recordUpdate()    ← NIS 统计
       │
       v
fill_output()             ← 输出位置、速度、协方差、质量指标
```

## 滤波器架构

`FilterBase` 用四个回调函数定义系统模型：

| 方法 | 函数签名 | 用途 |
|------|---------|------|
| `setStateFunc` | `(x, dt) → x'` | 状态转移预测 |
| `setStateJacobian` | `(x, dt) → F` | 协方差传播 Jacobian |
| `setMeasureFunc` | `(x) → z_pred` | 预测测量值 |
| `setMeasureJacobian` | `(x) → H` | 观测 Jacobian |

支持三种方式使用：
- **解析法** — 手动推导 Jacobian 公式，传入闭包
- **数值法** — 从 f/h 自动做有限差分，不需推导
- **无导法** — 用 UKF，不需要任何 Jacobian

## C API

```c
void TM_ProcessMeasurement(const void* measurement, int measSize,
                           TM_LocationOutput* output);
int  TM_NoDetectUpdate(double simTime, TM_LocationOutput* output);
int  TM_Reset();
int  TM_GetFilterState();
double TM_GetChiSquaredMean();
int  TM_SetConfig(const TM_Config* cfg);
int  TM_GetConfig(TM_Config* cfg);
```

## 坐标参考系

| 宏 | 值 | 说明 |
|----|----|------|
| `TM_FRAME_ECEF` | 0 | 地心地固直角坐标系（当前世界系） |
| `TM_FRAME_NED` | 1 | 北-东-地 局部坐标系 |
| `TM_FRAME_ENU` | 2 | 东-北-天 局部坐标系 |
| `TM_FRAME_BODY` | 3 | 传感器本体坐标系 |
| `TM_FRAME_LLA` | 4 | WGS84 大地坐标 |
| `TM_FRAME_ECI` | 5 | 地心惯性系（J2000） |
| `TM_FRAME_ORBITAL` | 6 | 轨道系：x-矢径方向(+R), z-轨道正法向 |

位置/速度变换工具：
- `ecefToNed` / `nedToEcef` — ECEF↔NED
- `llaToEcef` / `ecefToLla` — 大地坐标↔ECEF
- `CoordTransform` — 任意两笛卡尔参考系间 SE3 变换（`applyPos` / `applyVel`）

## 构建

```powershell
cmake -S . -B build
cmake --build build --config Release
```
