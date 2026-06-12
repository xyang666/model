# Solver — 常微分方程数值求解库

基于 Runge-Kutta 方法的 C++11 常微分方程（ODE）数值求解库。类架构参考 [hipparchus-ode](https://hipparchus.org/)。

## 特性

- **8 种积分器** — 包含定步长 Runge-Kutta、自适应（嵌入）Runge-Kutta、以及变步长 Adams-Bashforth-Moulton 预测-校正方法
- **观察者模式** — 可在积分步中挂载回调，用于日志记录、数据采集或提前终止
- **事件检测** — 检测过零点，并通过 Brent 方法精确定位事件时刻
- **模板泛化** — 支持将任意可调用对象（lambda、仿函数、函数指针）包装为 ODE
- **零外部依赖** — 兼容层完全自包含，无需第三方库

## 积分器

### 定步长方法

| 类名 | 阶数 | 级数 | 参考文献 |
|------|------|------|----------|
| `RK4` | 4 | 4 | 经典 Runge-Kutta |
| `RK8` | 8 | 10 | Shanks, E. Baylis |
| `RKV8` | 8 | 11 | Verner (1972) |

### 自适应（变步长）方法

| 类名 | 阶数 | 级数 | 参考文献 |
|------|------|------|----------|
| `RKCK` | 4(5) | 6 | Cash-Karp |
| `RKF45` | 4(5) | 6 | Fehlberg |
| `RKF56` | 5(6) | 8 | Fehlberg |
| `RKF78` | 7(8) | 13 | Fehlberg |

自适应方法利用嵌入对估计局部截断误差，自动调整步长以满足用户指定的绝对和相对误差容限。

### 多步方法

| 类名 | 阶数 | 类型 | 参考文献 |
|------|------|------|----------|
| `ABM` | 1–6 | PECE 预测-校正 | Adams-Bashforth-Moulton |

`ABM` 是一种变步长、变系数线性多步法。采用 PECE（Predict-Evaluate-Correct-Evaluate）模式：使用 Adams-Bashforth 公式显式预测，随后使用 Adams-Moulton 公式隐式校正。每步积分权重通过实际时间网格上的 Vandermonde 系统动态计算，天然支持非均匀步长。前几步使用 RK4 自启动以填充历史缓冲区。局部误差通过 Milne 装置（预测值与校正值之差）进行估计，并送入步长控制器。

## 快速开始

### 构建

```bash
cmake -B build
cmake --build build
```

### 最小示例

```cpp
#include <Solver/Solver.hpp>
#include <cstdio>
#include <cmath>

using namespace Solver;

int main()
{
    // 指数衰减: dy/dt = -0.5*y
    auto decay = [](const double* y, double* dy, double t) -> errc_t {
        dy[0] = -0.5 * y[0];
        return eNoError;
    };

    double y[1] = { 100.0 };  // 初始量
    double t = 0.0;

    RK4 integrator;
    integrator.setStepSize(0.1);
    integrator.integrate(decay, 1, y, t, 5.0);

    // 解析解: y(t) = 100 * exp(-0.5*t), y(5) ≈ 8.2085
    std::printf("y(5) = %.6f (解析解: %.6f)\n", y[0], 100.0 * std::exp(-2.5));
    return 0;
}
```

## 架构

```
IODEIntegrator          (纯虚接口)
  └── ODEIntegrator     (基类实现，包含观察者/事件管理)
        ├── ODEFixedStepIntegrator    (定步长)
        │     ├── RK4
        │     ├── RK8
        │     └── RKV8
        └── ODEVarStepIntegrator     (自适应步长)
              ├── RKCK
              ├── RKF45
              ├── RKF56
              ├── RKF78
              └── ABM
```

### 核心类

- **`OrdinaryDifferentialEquation`**（别名 `ODE`）— ODE 系统抽象基类。实现 `evaluate()` 或使用 `make_ode(func, dim)` 包装 lambda。
- **`IODEIntegrator`** / **`ODEIntegrator`** — 积分器接口，提供 `integrate()`、`singleStep()` 以及观察者/事件管理。
- **`ODEStateObserver`** — 每步积分成功后回调。可使用 `ODEStateVectorCollector` 记录状态历史。
- **`ODEEventDetector`** — 检测用户定义的切换函数的过零点。通过 `addEventDetector()` 挂载检测器。

## 示例

`examples/` 目录包含三个示例程序：

1. **谐振子**（`01-harmonic-oscillator.cpp`）— 用 RK4 求解 `y'' = -k*y`，并与解析解对比验证精度。
2. **自适应方法对比**（`02-adaptive-comparison.cpp`）— 在 Van der Pol 振子上对比 RKF45/RKF56/RKF78/RKCK/ABM 的步数效率与精度。
3. **步进观察者**（`03-event-detection.cpp`）— 演示观察者模式，在钟摆求解中监控每一步的状态。

构建后运行示例：

```bash
./build/01-harmonic-oscillator
./build/02-adaptive-comparison
./build/03-event-detection
```

## 参数配置

自适应积分器暴露以下可调参数：

```cpp
RKF78 integrator;
integrator.setInitialStepSize(60.0);   // 初始步长
integrator.setMaxAbsErr(1e-10);        // 绝对误差容限
integrator.setMaxRelErr(1e-13);        // 相对误差容限
// 高级参数: minStepScaleFactor_, maxStepScaleFactor_, safetyCoeffLow_, safetyCoeffHigh_

ABM abm(4);                            // Adams-Bashforth-Moulton, 4 阶
abm.setInitialStepSize(0.1);
abm.setMaxAbsErr(1e-8);
abm.setMaxRelErr(1e-8);
```

## 依赖

无外部依赖。库完全自包含。

**需要 C++11** 或更高版本。

## 许可证

Apache 2.0 — 详见 [LICENSE](http://www.apache.org/licenses/LICENSE-2.0)。

## 参考文献

- [hipparchus-ode](https://hipparchus.org/) — Java ODE 库，类结构参考来源
- Shanks, E. Baylis. "Solutions of Differential Equations by Evaluations of Functions"
- Verner. "Some Explicit Runge-Kutta Methods of High Order" (1972)
- Cash & Karp. "A Variable Order Runge-Kutta Method for Initial Value Problems with Rapidly Varying Right-Hand Sides"
- Fehlberg. "Classical Fourth- and Lower Order Runge-Kutta Formulas with Stepsize Control"
- Fehlberg. "Classical Fifth-, Sixth-, Seventh-, and Eighth-Order Runge-Kutta Formulas with Stepsize Control"
- Shampine & Gordon. "Computer Solution of Ordinary Differential Equations: The Initial Value Problem" (1975) — 变步长 Adams 方法经典参考
- Hairer, Norsett & Wanner. "Solving Ordinary Differential Equations I: Nonstiff Problems" (1993)
