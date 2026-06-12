# ESM

电子支援措施（Electronic Support Measures）传感器功能库。

实现被动射频信号侦收的完整模型，涵盖链路预算、方向性天线、频段扫描调度、频率相关检测门限（含 CW/PULSED 分离）、PSOS 概率扫描累积探测、辐射源航迹维护、极化/带宽效应、系统噪声温度模型等核心环节。适用于 RWR（雷达告警接收机）、ELINT（电子情报）、COMINT（通信情报）等场景的仿真建模。

---

## 目录

- [ESM](#esm)
  - [目录](#目录)
  - [工作原理](#工作原理)
  - [项目结构](#项目结构)
  - [类说明](#类说明)
    - [FreqTable](#freqtable)
    - [AntennaPattern](#antennapattern)
    - [Antenna](#antenna)
    - [EM\_Transmitter](#em_transmitter)
    - [EM\_Receiver](#em_receiver)
    - [EM\_Interaction](#em_interaction)
    - [EmitterTrack](#emittertrack)
    - [ESM\_Sensor](#esm_sensor)
  - [天线模型](#天线模型)
    - [坐标约定（ENU 平地）](#坐标约定enu-平地)
    - [三种方向图](#三种方向图)
    - [FOV 约束](#fov-约束)
    - [使用示例](#使用示例)
  - [链路预算](#链路预算)
    - [基础公式](#基础公式)
    - [极化效应](#极化效应)
    - [带宽效应](#带宽效应)
    - [系统噪声温度模型](#系统噪声温度模型)
  - [检测模型](#检测模型)
    - [直接探测](#直接探测)
    - [PSOS（概率扫描累积探测）](#psos概率扫描累积探测)
  - [构建方法](#构建方法)
  - [快速上手](#快速上手)
  - [依赖](#依赖)

---

## 工作原理

ESM 传感器是**纯被动**设备，自身不发射任何信号，仅监听空间中的射频辐射源。

```
辐射源 (ESM_Transmitter)
    │  峰值功率 P_t + 天线增益 G_t → EIRP
    │  信号类型：CW（连续波）/ PULSED（脉冲，含占空比、PRF）
    │  极化：H / V / LHCP / RHCP
    │  扫描模式：FIXED / AZ_SCAN / EL_SCAN / AZ_EL_SCAN
    │  发射天线方向图（PA 计算输入）
    ▼
链路预算 (ESM_Interaction::compute)
    │  1. 几何计算（range、azimuth、elevation）
    │  2. FOV 检查（接收天线方位/仰角/距离范围）
    │  3. 自由空间路径损耗 FSPL = 20·log10(4π·R·f/c)
    │  4. 接收天线增益 G_r(θ)（方向图 + 频率修正）
    │  5. 极化效应 / 带宽效应修正
    │  6. 接收功率 P_r = EIRP + G_r − FSPL + 极化 + 带宽
    │  7. 噪声底（经典公式 或 系统噪声温度模型）
    │  8. SNR（含杂波/干扰项）
    │  9. 扫描调度检查（fmod(t, revisit) < dwell?）
    │  10. 频率相关门限/灵敏度（CW/PULSED 分离路由）
    │  11. 脉冲占空比惩罚：SNR_eff = SNR + 10·log10(duty_cycle)
    │
    ├─ 直接模式 → SNR_eff ≥ threshold && scan_available && P_r ≥ sensitivity
    │
    └─ PSOS 模式 → PA × PF 扫描重叠模型，跨帧贝叶斯累积
                   发射天线波束扫描时方位重叠概率 PA
                   接收机频段驻留时间占比 PF = dwell / revisit
                   单帧 PSS_frame = 1 - (1 - PA·PF)^dwell_count
                   累积 pd_cum = 1 - (1 - pd_cum_old)·(1 - PSS_frame)
                   达到确认门限后更新 EmitterTrack
    ▼
航迹列表 (vector<EmitterTrack>)
    └─ 超过 coast_time 无观测 → 自动删除
```

每帧调用 `ESM_Sensor::update(sim_time, transmitters)`，传感器自动完成扫描调度、链路预算、检测判决和航迹管理。

---

## 项目结构

```
ESM/
├── CMakeLists.txt
├── README.md
├── include/
│   ├── AntennaPattern.hpp   # 天线方向图（各向同性 / 高斯 / Sinc²）
│   ├── Antenna.hpp          # 带指向的天线（方向图 + 波束指向 + FOV）
│   ├── FreqTable.hpp        # 频率索引线性插值查找表
│   ├── ESM_Transmitter.hpp   # 辐射源（CW/PULSED + 极化 + 扫描参数）
│   ├── ESM_Receiver.hpp      # 接收机（频段调度 + 分离阈值表 + PSOS 配置）
│   ├── ESM_Interaction.hpp   # 单向电磁交互（链路预算 + 检测判决）
│   ├── EmitterTrack.hpp     # 已探测辐射源航迹（含 PSOS 累积状态）
│   └── ESM_Sensor.hpp       # 传感器主类
└── src/
    ├── AntennaPattern.cpp   # 三种方向图增益计算 + GetGainThresholdFraction
    ├── Antenna.cpp          # 指向坐标变换 + 增益查询 + FOV 检查
    ├── ESM_Interaction.cpp   # 几何计算 + 链路预算 + 检测逻辑
    ├── ESM_Sensor.cpp       # 扫描 / 直接探测 / PSOS 累积 / 航迹老化
    └── main.cpp             # 演示程序
```

---

## 类说明

### FreqTable

> [include/FreqTable.hpp](include/FreqTable.hpp)

频率索引查找表，支持分段线性插值，范围外自动钳位到边界值。

| 方法 | 说明 |
|------|------|
| `add(freq_hz, value)` | 添加一个频率-值条目 |
| `build()` | 按频率升序排序（添加完成后调用一次） |
| `lookup(freq_hz)` | 返回线性插值结果，范围外钳位到边界值 |
| `empty()` | 表是否为空 |

用于 `detection_threshold_table`、`sensitivity_table`、`gain_adjustment_table` 等场景。

**使用示例：**
```cpp
FreqTable t;
t.add(2e9,  10.0);  // 2 GHz → 10 dB 门限
t.add(6e9,   8.0);  // 6 GHz →  8 dB
t.add(12e9, 12.0);  // 12 GHz → 12 dB
t.build();
double thr = t.lookup(3e9); // ≈ 9.3 dB（线性插值）
```

---

### AntennaPattern

> [include/AntennaPattern.hpp](include/AntennaPattern.hpp)
> [src/AntennaPattern.cpp](src/AntennaPattern.cpp)

天线辐射方向图模型，输入为**相对于波束轴的偏置角**（off_az, off_el），输出为增益（dBi）。

| 成员 | 类型 | 说明 |
|------|------|------|
| `type` | `Type` | `ISOTROPIC` / `GAUSSIAN` / `SINC2` |
| `peak_gain_dBi` | `double` | 轴向（boresight）峰值增益（dBi） |
| `az_beamwidth_rad` | `double` | 方位 3 dB 波束宽度（rad） |
| `el_beamwidth_rad` | `double` | 仰角 3 dB 波束宽度（rad） |
| `back_lobe_floor_dB` | `double` | 后瓣增益下限（dB，相对峰值），默认 -30 |
| `gain_adjustment_table` | `FreqTable` | 频率相关增益修正表；非空时叠加到计算增益 |
| `minimum_gain_dBi` | `double` | 硬增益下限（dBi），默认 -40 |

| 方法 | 说明 |
|------|------|
| `gain_dBi(off_az, off_el, freq)` | 查询指定偏置角和频率处的增益 |
| `GetGainThresholdFraction(threshold_norm, min_az, max_az)` | 计算方位区间内功率增益超过门限的比例（用于 PSOS 的 PA 计算） |

**三种方向图公式：**

| 类型 | 公式 |
|------|------|
| ISOTROPIC | $G = G_{peak}$ |
| GAUSSIAN | $G = G_{peak} - 12\left[\left(\frac{\theta_{az}}{BW_{az}}\right)^2 + \left(\frac{\theta_{el}}{BW_{el}}\right)^2\right]$ |
| SINC2 | $G = G_{peak} + 20\log_{10}\left\|\mathrm{sinc}\!\left(\frac{\pi\theta}{BW}\right)\right\|^2$ |

**计算流程：**

```
Antenna::gain_dBi(src_az, src_el, freq)
  │
  ├─1. 坐标系变换（compute_offsets）
  │    将全局 (src_az, src_el) 投影到波束切平面：
  │      bx = 波束轴单位向量（boresight_az, boresight_el）
  │      by = 水平垂直轴（boresight_az + 90°, el=0）
  │      bz = bx × by（仰角轴）
  │      theta     = acos(clamp(bx · src, -1, 1))    ← 总偏置角
  │      off_az    = theta · (by · src) / proj_mag   ← 方位偏置分量
  │      off_el    = theta · (bz · src) / proj_mag   ← 仰角偏置分量
  │
  ├─2. AntennaPattern::gain_dBi(off_az, off_el, freq)
  │    │
  │    ├─ ISOTROPIC:  relative_dB = 0.0
  │    │              无论偏置角多大，增益始终为峰值增益
  │    │
  │    ├─ GAUSSIAN:   relative_dB = -12·[(off_az/BW_az)² + (off_el/BW_el)²]
  │    │              波束轴处 0 dB，3dB 点处 -3 dB，随偏置角二次衰减
  │    │              方位和仰角分量独立叠加（椭圆波束）
  │    │
  │    └─ SINC2:      theta = sqrt(off_az² + off_el²)
  │                   BW_avg = (BW_az + BW_el) / 2
  │                   x = π · theta / BW_avg
  │                   relative_dB = 20·log10(|sin(x)/x|²)
  │                   有真实副瓣，孔径天线/相控阵适用
  │
  ├─3. 后瓣钳位:    relative_dB = max(relative_dB, back_lobe_floor_dB)
  ├─4. 叠加峰值得:  gain = peak_gain_dBi + relative_dB
  ├─5. 频率修正:    gain += gain_adjustment_table.lookup(freq)   （表非空时）
  └─6. 硬下限:      gain = max(gain, minimum_gain_dBi)
```

**归一化功率模式（PSOS 的 PA 计算使用）：**

`norm_power_at_azimuth(az_rad)` 返回方位面上（el=0）某角度处相对峰值的线性功率增益 [0, 1]：

| 类型 | 公式 |
|------|------|
| ISOTROPIC | $P_n = 1$ |
| GAUSSIAN | $P_n = \exp\left(-4\ln 2 \cdot (az / BW_{az})^2\right)$ |
| SINC2 | $P_n = \left|\mathrm{sinc}(\pi \cdot az / BW_{avg})\right|^2$ |

`GetGainThresholdFraction` 基于此模式计算方位扫描范围内超过门限的角度占比，用于 PSOS 的 PA（方位重叠概率）。GAUSSIAN 使用解析反解，SINC2 在 1000 个等分点上数值采样。

**GetGainThresholdFraction 算法**（用于 PSOS 方位重叠概率）：

```
ISOTROPIC → 始终返回 1.0
GAUSSIAN  → 解析解：az_half = BW · sqrt(-ln(threshold_norm) / a)
             其中 a = 4·ln(2)，覆盖比例 = min(1, 2·az_half / scan_span)
SINC2     → 数值采样，NUM_PA_SAMPLE_POINTS = 1000 个等分点
```

---

### Antenna

> [include/Antenna.hpp](include/Antenna.hpp)
> [src/Antenna.cpp](src/Antenna.cpp)

带波束指向的定向天线，将全局坐标系中的信号到达方向转换为相对于波束轴的偏置角，再查询 `AntennaPattern`。

| 成员 | 类型 | 说明 |
|------|------|------|
| `pattern` | `AntennaPattern` | 方向图模型 |
| `boresight_az_rad` | `double` | 波束指向方位角（rad，北向=0，顺时针+） |
| `boresight_el_rad` | `double` | 波束指向仰角（rad） |
| `scan_mode` | `ScanMode` | 波束扫描模式：`FIXED` / `AZ_SCAN` / `EL_SCAN` / `AZ_EL_SCAN` |
| `scan_min_az_rad` | `double` | 方位扫描范围下界 |
| `scan_max_az_rad` | `double` | 方位扫描范围上界 |
| `fov_min_az_rad` | `double` | FOV 方位下界 |
| `fov_max_az_rad` | `double` | FOV 方位上界 |
| `fov_min_el_rad` | `double` | FOV 仰角下界 |
| `fov_max_el_rad` | `double` | FOV 仰角上界 |
| `min_range_m` | `double` | 最小探测距离 |
| `max_range_m` | `double` | 最大探测距离 |

| 方法 | 说明 |
|------|------|
| `gain_dBi(src_az, src_el, freq)` | 信号来向的接收增益（dBi） |
| `off_boresight_angle_rad(az, el)` | 信号来向与波束轴的夹角（rad） |
| `within_fov(az, el, range)` | 信号来向是否在 FOV 范围内 |

---

### ESM_Transmitter

> [include/ESM_Transmitter.hpp](include/ESM_Transmitter.hpp)

表示一个射频辐射源。

| 成员 | 类型 | 说明 |
|------|------|------|
| `id` | `string` | 辐射源唯一标识 |
| `position` | `Eigen::Vector3d` | 位置（米，ENU 坐标系） |
| `frequency_hz` | `double` | 载波频率（Hz） |
| `power_dBm` | `double` | 峰值发射功率（dBm） |
| `antenna_gain_dBi` | `double` | 发射天线增益（dBi） |
| `signal_bandwidth_hz` | `double` | 发射信号带宽（Hz）；0 = 使用接收机带宽 |
| `signal_type` | `SignalType` | `CW`（连续波）或 `PULSED`（脉冲） |
| `duty_cycle` | `double` | 发射占空比 [0..1]；仅 PULSED 时生效 |
| `polarization` | `Polarization` | 极化方式：`NONE` / `HORIZONTAL` / `VERTICAL` / `LHCP` / `RHCP` |
| `active` | `bool` | 是否正在发射 |

**脉冲参数（仅 PULSED 时有效）：**

| 成员 | 类型 | 说明 |
|------|------|------|
| `pulse_width_s` | `double` | 脉冲宽度（s） |
| `pulse_repetition_frequency_hz` | `double` | 脉冲重复频率 PRF（Hz） |
| `pulse_repetition_interval_s` | `double` | 脉冲重复间隔 PRI（s） |
| `pulse_compression_ratio` | `double` | 脉冲压缩比（绝对比值） |

**功率设定：**

| 成员 | 类型 | 说明 |
|------|------|------|
| `use_peak_power` | `bool` | `true` 返回峰值功率，`false`（默认）返回平均功率 |

**发射天线方向图 / 扫描参数（用于 PSOS 方位重叠概率 PA 计算）：**

| 成员 | 类型 | 说明 |
|------|------|------|
| `tx_antenna.pattern` | `TxAntennaModel::Pattern` | 发射天线方向图类型：`ISOTROPIC` / `GAUSSIAN` / `SINC2` |
| `tx_antenna.gain_dBi` | `double` | 发射天线峰值增益（dBi） |
| `tx_antenna.beamwidth_az_rad` | `double` | 发射方位 3dB 波束宽度（rad） |
| `tx_antenna.beamwidth_el_rad` | `double` | 发射仰角 3dB 波束宽度（rad） |
| `scan_mode` | `ScanMode` | `FIXED` / `AZ_SCAN` / `EL_SCAN` / `AZ_EL_SCAN` |
| `scan_min_az_rad` | `double` | 方位扫描范围下界 |
| `scan_max_az_rad` | `double` | 方位扫描范围上界 |

| 方法 | 说明 |
|------|------|
| `eirp_dBm()` | 等效全向辐射功率（峰值）= `power_dBm + antenna_gain_dBi` |
| `average_power_dBm()` | 平均功率（dBm），含占空比修正 |
| `get_power_dBm()` | 按 `use_peak_power` 返回峰值或平均功率 |

---

### ESM_Receiver

> [include/ESM_Receiver.hpp](include/ESM_Receiver.hpp)

ESM 接收机，支持多频段监听、扫描调度、频率相关检测门限（CW/PULSED 分离）和 PSOS 配置。

**FrequencyBand**

| 字段 | 类型 | 说明 |
|------|------|------|
| `lower_hz` | `double` | 频段下边界（Hz） |
| `upper_hz` | `double` | 频段上边界（Hz） |
| `dwell_time_s` | `double` | 每次扫描驻留时长（s），默认 1.0 |
| `revisit_time_s` | `double` | 扫描周期（s），默认 1.0（即始终驻留） |

**扫描调度规则：**
```
在时刻 t，当 fmod(t, revisit_time_s) < dwell_time_s 时，该频段处于激活状态。
dwell_time_s == revisit_time_s → 始终激活（固定接收）
```

**ESM_Receiver 成员：**

| 成员 | 类型 | 说明 |
|------|------|------|
| `position` | `Eigen::Vector3d` | 接收机位置（米，ENU） |
| `frequency_bands` | `vector<FrequencyBand>` | 监听频段列表（含扫描调度） |
| `antenna` | `Antenna` | 接收天线（方向图 + 波束指向 + FOV） |
| `noise_figure_dB` | `double` | 噪声系数（dB），默认 5.0 |
| `bandwidth_hz` | `double` | 瞬时接收带宽（Hz），默认 10 MHz |
| `detection_threshold_dB` | `double` | 默认最小 SNR 门限（dB），默认 10.0 |
| `rcvr_function` | `RcvrFunction` | 功能分类：`PASSIVE_SENSOR` / `COMM` / `SENSOR` / `INTERFERER` |
| `polarization` | `Polarization` | 接收机极化 |

**频率相关表（路由详见查询方法）：**

| 成员 | 类型 | 说明 |
|------|------|------|
| `detection_threshold_table` | `FreqTable` | 通用 SNR 门限表（dB），非空时覆盖标量 |
| `sensitivity_table` | `FreqTable` | 通用灵敏度表（dBm），非空时附加判决条件 |
| `cw_detection_threshold_table` | `FreqTable` | CW 信号 SNR 门限表，优先级高于通用表 |
| `pulsed_detection_threshold_table` | `FreqTable` | PULSED 信号 SNR 门限表，优先级高于通用表 |
| `cw_sensitivity_table` | `FreqTable` | CW 信号灵敏度表，优先级高于通用表 |
| `pulsed_sensitivity_table` | `FreqTable` | PULSED 信号灵敏度表，优先级高于通用表 |

**天线 / 馈线损耗：**

| 成员 | 类型 | 说明 |
|------|------|------|
| `antenna_ohmic_loss_dB` | `double` | 天线欧姆损耗（dB），0 = 无损耗 |
| `receive_line_loss_dB` | `double` | 接收馈线损耗（dB），0 = 无损耗 |

**噪声模型：**

| 成员 | 类型 | 说明 |
|------|------|------|
| `noise_multiplier` | `double` | 噪声乘数（绝对比值），1.0 = 标称噪声 |
| `explicit_noise_power_dBm` | `double` | 显式噪声功率（dBm），> -1e20 时覆盖公式计算 |

**PSOS 配置：**

| 成员 | 类型 | 说明 |
|------|------|------|
| `psos_enabled` | `bool` | 是否启用 PSOS 概率累积探测，默认 `false` |
| `psos_confirm_threshold` | `double` | PSOS 确认所需累积 Pd，默认 0.9 |
| `psos_frame_time_s` | `double` | 传感器更新帧周期（s），用于 dwell count 计算 |
| `required_pd` | `double` | 所需累积探测概率，>= 0 时覆盖 `psos_confirm_threshold` |

**查询方法：**

| 方法 | 说明 |
|------|------|
| `can_receive(freq)` | 频率是否在任意频段内（不检查扫描状态） |
| `is_scanning(freq, sim_time)` | 频率在频段内且当前时刻处于驻留窗口 |
| `get_detection_threshold_dB(freq, signal_type)` | 按信号类型路由的 SNR 门限 |
| `get_sensitivity_dBm(freq, signal_type)` | 按信号类型路由的灵敏度 |
| `noise_floor_dBm(elevation)` | 噪声底（经典公式或系统噪声温度模型） |
| `effective_noise_floor_dBm(elevation)` | 含噪声乘数的有效噪声底 |
| `ComputeSignalToNoise(signal, noise, clutter, interference)` | 综合 SNR 计算 (dB) |
| `GetPolarizationEffect(tx_polarization)` | 极化失配损耗 (dB) |
| `GetBandwidthEffect(signal_bw_hz)` | 带宽失配损耗 (dB) |
| `UpdateNoisePower(pulse_width_s)` | 按脉宽更新等效噪声带宽 |

**检测门限路由逻辑：**
```
get_detection_threshold_dB(freq, signal_type):
  if signal_type == PULSED && pulsed_detection_threshold_table non-empty
    → pulsed_detection_threshold_table.lookup(freq)
  else if signal_type == CW && cw_detection_threshold_table non-empty
    → cw_detection_threshold_table.lookup(freq)
  else if detection_threshold_table non-empty
    → detection_threshold_table.lookup(freq)
  else → detection_threshold_dB (scalar)
```

灵敏度路由逻辑同上。

---

### ESM_Interaction

> [include/ESM_Interaction.hpp](include/ESM_Interaction.hpp)
> [src/ESM_Interaction.cpp](src/ESM_Interaction.cpp)

单次单向电磁交互，封装从几何计算到检测判决的完整流程。

| 成员 | 类型 | 说明 |
|------|------|------|
| `transmitter` | `const ESM_Transmitter*` | 辐射源指针 |
| `receiver` | `const ESM_Receiver*` | 接收机指针 |
| `sim_time` | `double` | 当前仿真时刻（s），用于扫描调度检查 |

**几何关系：**

| 成员 | 类型 | 说明 |
|------|------|------|
| `range_m` | `double` | 收发距离（m） |
| `azimuth_rad` | `double` | 接收机→辐射源方位角（rad，北=0 顺时针+） |
| `elevation_rad` | `double` | 仰角（rad） |

**链路预算：**

| 成员 | 类型 | 说明 |
|------|------|------|
| `path_loss_dB` | `double` | 自由空间路径损耗（dB） |
| `antenna_gain_dBi` | `double` | 信号到达方向处的接收天线增益（dBi） |
| `received_power_dBm` | `double` | 接收机输入端功率（dBm） |
| `noise_floor_dBm` | `double` | 噪声底（dBm） |
| `snr_dB` | `double` | SNR（dB，脉冲惩罚前） |

**传播与环境（接口预留，当前返回默认值）：**

| 成员 | 类型 | 说明 |
|------|------|------|
| `propagation_factor` | `double` | 方向图传播因子；1.0 = 自由空间 |
| `absorption_factor` | `double` | 大气吸收；1.0 = 无损耗 |
| `terrain_masked` | `bool` | 是否被地形遮蔽 |
| `horizon_masked` | `bool` | 是否被地球曲率遮蔽 |
| `zone_attenuation_dB` | `double` | 区域衰减附加损耗（dB） |
| `interference_power_dBm` | `double` | 总干扰功率（dBm） |
| `interference_factor` | `double` | 干扰效应因子 [0..1] |
| `doppler_frequency_hz` | `double` | 多普勒频移（Hz） |
| `doppler_speed_mps` | `double` | 多普勒速度（m/s） |

**检测门限：**

| 成员 | 类型 | 说明 |
|------|------|------|
| `effective_threshold_dB` | `double` | 实际 SNR 门限（频率查表或标量） |
| `sensitivity_dBm` | `double` | 最小可检测功率（dBm） |
| `snr_margin_dB` | `double` | SNR_eff − 门限，> 0 表示可探 |

**状态标记：**

| 成员 | 类型 | 说明 |
|------|------|------|
| `scan_available` | `bool` | 接收机在 sim_time 是否处于该频段驻留窗口 |
| `detected` | `bool` | 最终检测判决 |

| 方法 | 说明 |
|------|------|
| `compute()` | 执行全部计算；`false` 表示无需交互（非激活或频外） |

**`compute()` 判决流程：**
```
1. transmitter->active ?                     → 否 → return false
2. can_receive(freq) ?                       → 否 → return false
3. 计算几何（range / az / el）
4. FOV 约束检查（within_fov）
5. 计算链路预算（FSPL、天线增益、极化/带宽修正、P_r、SNR）
6. 传播 / 环境桩函数（当前返回默认值）
7. scan_available = is_scanning(freq, sim_time)
8. effective_threshold = get_detection_threshold_dB(freq, signal_type)
9. sensitivity_dBm     = get_sensitivity_dBm(freq, signal_type)
10. PULSED: snr_margin = SNR + 10·log10(duty_cycle) − threshold
    CW:     snr_margin = SNR − threshold
11. detected = scan_available
             && snr_for_detection >= effective_threshold
             && received_power_dBm >= sensitivity_dBm
```

---

### EmitterTrack

> [include/EmitterTrack.hpp](include/EmitterTrack.hpp)

已探测到的辐射源航迹，由 `ESM_Sensor` 创建和维护。

| 成员 | 类型 | 说明 |
|------|------|------|
| `emitter_id` | `string` | 辐射源 ID |
| `frequency_hz` | `double` | 探测频率（Hz） |
| `azimuth_rad` | `double` | 方位角（rad） |
| `elevation_rad` | `double` | 仰角（rad） |
| `range_m` | `double` | 距离估计（m，0 = 未知） |
| `received_power_dBm` | `double` | 最近接收功率（dBm） |
| `antenna_gain_dBi` | `double` | 最近探测所用接收天线增益（dBi） |
| `snr_dB` | `double` | 最近 SNR（dB） |
| `first_detect_time` | `double` | 首次观测时刻（s） |
| `last_detect_time` | `double` | 最近确认探测时刻（s） |
| `last_update_time` | `double` | 最近带内观测时刻（s，驱动老化） |
| `detect_count` | `int` | 累计探测次数 |

**PSOS 状态：**

| 成员 | 类型 | 说明 |
|------|------|------|
| `psos_cumulative_pd` | `double` | 累积检测概率，初始 0.0 |
| `psos_confirmed` | `bool` | 累积 Pd >= 确认阈值后置 true |

---

### ESM_Sensor

> [include/ESM_Sensor.hpp](include/ESM_Sensor.hpp)
> [src/ESM_Sensor.cpp](src/ESM_Sensor.cpp)

传感器主类，驱动完整的感知-决策-维迹循环，支持直接探测和 PSOS 两种模式。

| 成员 | 类型 | 说明 |
|------|------|------|
| `receiver` | `ESM_Receiver` | 接收机配置 |
| `coast_time` | `double` | 航迹保持时间（s），默认 5.0 |

| 方法 | 说明 |
|------|------|
| `update(t, transmitters)` | 扫描所有辐射源，更新航迹，老化过期航迹 |
| `attempt_detect(xmtr, t)` | 对单个辐射源执行链路预算，返回完整 `ESM_Interaction` |
| `tracks()` | 返回当前活跃航迹列表（只读） |

**两种检测模式流程：**

```
// 直接模式（psos_enabled == false，默认）
for each transmitter:
    r = attempt_detect(xmtr, sim_time)
    if !r.transmitter: skip            // 非激活 / 频外 / FOV 外
    track = get_or_create_track(id)
    track.last_update_time = sim_time  // 始终更新（驱动老化）
    if r.detected:
        fill_track_data(track, r)      // 刷新测量值

// PSOS 模式（psos_enabled == true）
for each transmitter:
    r = attempt_detect(xmtr, sim_time)
    if !r.transmitter: skip
    track = get_or_create_track(id)
    track.last_update_time = sim_time
    psos = compute_psos(r, track)       // PA × PF 扫描重叠模型
    track.psos_cumulative_pd = psos.pd_cum
    if psos.pss > 0:                   // 有可能检测
        if psos.pd_cum >= required_pd: // 达到确认门限
            track.psos_confirmed = true
            fill_track_data(track, r)
    else:                              // PSS == 0 → 衰减
        track.psos_cumulative_pd *= 0.8
        if psos_cumulative_pd < 0.5: psos_cumulative_pd = 0.0

age_tracks(sim_time)  // 删除 last_update_time 过期的航迹
```

**PSOS 算法（compute_psos）：**

详见下方 [PSOS 模型](#psos概率扫描累积探测) 章节。

---

## 天线模型

### 坐标约定（ENU 平地）

- **方位角**：在水平面内，从 +Y（北）起顺时针为正（rad）
- **仰角**：相对水平面，向上为正（rad）
- **波束坐标系**：以波束轴为 X 轴，`by` = 水平垂直轴，`bz` = 仰角轴

### 三种方向图

```
增益(dB)
  0 |****ISOTROPIC*****
    |
-10 |      .**GAUSSIAN**.
    |    .`              `.
-20 |  .`                  `.
    | .*----SINC2----*       * .
-30 |_ _ _ _ _ _ _ _ _ _ _ _ _ _  ← back_lobe_floor
    |-90  -60  -30   0   30  60  90  偏置角(deg)
```

| 类型 | 适用场景 |
|------|---------|
| ISOTROPIC | 全向天线（螺旋天线、偶极子） |
| GAUSSIAN | 定向天线通用近似，参数直观 |
| SINC2 | 孔径天线、相控阵，副瓣更真实 |

### FOV 约束

接收天线支持视场角（FOV）约束，信号来向超出方位/仰角/距离范围时直接判为不可检测：

```cpp
antenna.within_fov(azimuth_rad, elevation_rad, range_m)
```
- `fov_min_az_rad` / `fov_max_az_rad`：方位 FOV
- `fov_min_el_rad` / `fov_max_el_rad`：仰角 FOV
- `min_range_m` / `max_range_m`：距离 FOV

### 使用示例

```cpp
// 高斯定向天线，波束指向东北（45°），60°×40° 波束宽度
Antenna ant;
ant.pattern.type               = AntennaPattern::Type::GAUSSIAN;
ant.pattern.peak_gain_dBi      = 8.0;
ant.pattern.az_beamwidth_rad   = 60.0 * M_PI / 180.0;
ant.pattern.el_beamwidth_rad   = 40.0 * M_PI / 180.0;
ant.pattern.back_lobe_floor_dB = -20.0;
ant.boresight_az_rad = 45.0 * M_PI / 180.0;
ant.boresight_el_rad = 0.0;

double g = ant.gain_dBi(0.0, 0.0, 3e9); // 来自正北（偏置45°），增益有所衰减
```

---

## 链路预算

### 基础公式

采用 **Friis 自由空间传输方程**，接收天线增益为方向性增益 $G_r(\theta)$：

$$
P_r\,(\text{dBm}) = P_t + G_t + G_r(\theta) - \text{FSPL} + \text{PolLoss} + \text{BwLoss}
$$

$$
\text{FSPL}\,(\text{dB}) = 20\log_{10}\!\left(\frac{4\pi R f}{c}\right)
$$

$$
\text{SNR}\,(\text{dB}) = 10\log_{10}\!\left(\frac{S}{N + C + I}\right)
$$

| 符号 | 含义 | 单位 |
|------|------|------|
| $P_t$ | 发射峰值功率 | dBm |
| $G_t$ | 发射天线增益 | dBi |
| $G_r(\theta)$ | 接收天线方向性增益 | dBi |
| $R$ | 收发距离 | m |
| $f$ | 载波频率 | Hz |
| $c$ | 光速 3×10⁸ | m/s |
| PolLoss | 极化失配损耗 | dB |
| BwLoss | 带宽失配损耗 | dB |
| $N$ | 噪声功率（线性） | mW |
| $C$ | 杂波功率（线性） | mW |
| $I$ | 干扰功率（线性） | mW |

### 极化效应

发射与接收极化不匹配时的信号功率损耗：

| 组合 | 损耗 |
|------|------|
| 同向极化 | 0 dB |
| 正交线极化（H↔V） | -30 dB |
| 反向圆极化（LHCP↔RHCP） | -30 dB |
| 圆↔线 | -3 dB |
| 一方为 NONE | 0 dB（未知视为匹配） |

### 带宽效应

信号带宽超过接收机带宽时，有效接收功率下降：

$$
\Delta P_{\text{BW}} = \begin{cases}
0, & B_{\text{sig}} \le B_{\text{rcv}} \\
10\log_{10}(B_{\text{rcv}} / B_{\text{sig}}), & B_{\text{sig}} > B_{\text{rcv}}
\end{cases}
$$

### 系统噪声温度模型

当存在天线欧姆损耗或馈线损耗时，使用系统噪声温度模型计算噪声底：

$$
T_{\text{ant}}(el) = \begin{cases}
50 + 20\cos(el), & el > 0^\circ \\
290\,\text{K}, & el \le 0^\circ
\end{cases}
$$

$$
T_{\text{sys}} = T_{\text{ant}} \cdot L_{\text{line}} + T_0(L_{\text{line}} - 1) + T_0(\text{NF}_{\text{lin}} - 1)
$$

其中 $T_0 = 290\,\text{K}$，$L_{\text{line}}$ 包含欧姆损耗和馈线损耗的线性值。

无损耗时使用经典热噪声公式：

$$
P_{\text{noise}} = -174 + 10\log_{10}(B) + \text{NF} \quad (\text{dBm})
$$

含 `noise_multiplier` 时：$P_{\text{noise\_eff}} = P_{\text{noise}} + 10\log_{10}(M)$

---

## 检测模型

### 直接探测

同时满足以下三个条件时判为探测成功：

| 条件 | 说明 |
|------|------|
| `scan_available` | 接收机正在驻留该频段（扫描调度） |
| `snr_eff >= threshold` | 有效 SNR 超过门限（含脉冲占空比惩罚） |
| `P_r >= sensitivity` | 接收功率超过灵敏度下限 |

**脉冲信号占空比惩罚：**

$$
\text{SNR}_{eff} = \text{SNR} + 10\log_{10}(d),\quad d \in (0,\,1]
$$

占空比越小，有效 SNR 越低，探测越困难。

**检测门限按信号类型分离：**

CW 和 PULSED 信号可分别配置独立的频率相关检测门限表和灵敏度表。路由优先级：
1. `pulsed_detection_threshold_table`（PULSED 信号）
2. `cw_detection_threshold_table`（CW 信号）
3. `detection_threshold_table`（通用回退）
4. `detection_threshold_dB`（标量回退）

### PSOS（概率扫描累积探测）

适用于扫描雷达波束周期性扫过 ESM 的场景：发射天线波束窄且扫描范围大，ESM 每帧只能收到很短的照射时间，单帧检测概率较低，需要跨帧累积。

**模型：扫描重叠概率（PA × PF）**

与 AFSIM WSFESM_SENSOR 的 `ComputePSOS_Effects` 算法对齐。

**PA（方位重叠概率）：**

发射天线在扫描周期内，波束指向满足检测门限要求的时间占比。

```
G_req_norm = S_req / (S_iso · G_t_max)
其中：
  S_req     = 10^(threshold/10) · 10^(noise_floor/10)  （所需最小接收功率，线性 mW）
  S_iso     = 10^(P_r/10)                                （各向同性接收功率，线性 mW）
  G_t_max   = 10^(tx_antenna.gain_dBi/10)               （发射峰值增益，线性）

PA = AntennaPattern::GetGainThresholdFraction(G_req_norm, scan_min_az, scan_max_az)
```

**PF（频率重叠概率）：**

接收机在目标频段的驻留时间占比：

$$
\text{PF} = \frac{\text{dwell\_time}}{\text{revisit\_time}}
$$

**单次扫描检测概率 PSS：**

$$
\text{PSS} = \text{PA} \times \text{PF}
$$

**帧内多次驻留：**

$$
\text{PSS}_{\text{frame}} = 1 - (1 - \text{PSS})^{N_{\text{dwell}}}
$$

$$
N_{\text{dwell}} = \max\left(1,\; \frac{\text{psos\_frame\_time\_s}}{\text{revisit\_time\_s}}\right)
$$

**贝叶斯累积：**

$$
P_{\text{cum}}^{(n)} = 1 - \left(1 - P_{\text{cum}}^{(n-1)}\right) \cdot \left(1 - \text{PSS}_{\text{frame}}\right)
$$

**确认判决：**

- $\text{PSS} > 0$ 且 $P_{\text{cum}} \ge \text{required\_pd}$ → 确认探测，更新航迹
- $\text{PSS} = 0$ → $P_{\text{cum}} \gets P_{\text{cum}} \times 0.8$；若 $P_{\text{cum}} < 0.5$ 则置零

**`required_pd` 取值规则：**
- `required_pd >= 0` → 使用 `required_pd`
- `required_pd < 0`（默认 -1）→ 使用 `psos_confirm_threshold`

---

## 构建方法

**依赖：** CMake ≥ 3.14，C++14 编译器，Eigen 3.4

```bash
# 在 ESM 目录下
cmake -B build -S .
cmake --build build

# 运行演示
./build/esm_demo          # Linux / macOS
build\Debug\esm_demo.exe  # Windows
```

Eigen 路径默认为 `../3rdparty/eigen-3.4.0`，可通过 CMake 变量覆盖：

```bash
cmake -B build -DEIGEN3_INCLUDE_DIR=<your_eigen_path>
```

---

## 快速上手

```cpp
#include "ESM_Sensor.hpp"

// 1. 配置接收机
ESM_Sensor esm;
esm.receiver.position            = Eigen::Vector3d(0, 0, 10);
esm.receiver.noise_figure_dB     = 6.0;
esm.receiver.bandwidth_hz        = 20e6;
esm.receiver.detection_threshold_dB = 8.0;
esm.coast_time                   = 5.0;

// 扫描频段：S 波段每 2 s 扫一次，驻留 0.5 s；X 波段始终监听
{
    FrequencyBand band(2e9, 4e9);
    band.dwell_time_s = 0.5;
    band.revisit_time_s = 2.0;
    esm.receiver.frequency_bands.push_back(band);
}
esm.receiver.frequency_bands.push_back({8e9, 12e9}); // X 波段（固定监听）

// 频率相关 SNR 门限
esm.receiver.detection_threshold_table.add(2e9,  12.0);
esm.receiver.detection_threshold_table.add(6e9,   8.0);
esm.receiver.detection_threshold_table.add(12e9, 10.0);
esm.receiver.detection_threshold_table.build();

// 2. 定义辐射源（脉冲雷达，占空比 10%）
ESM_Transmitter radar;
radar.id             = "threat-radar";
radar.position       = Eigen::Vector3d(0, 50000, 0);
radar.frequency_hz   = 3e9;
radar.power_dBm      = 80.0;
radar.antenna_gain_dBi = 30.0;
radar.signal_type    = SignalType::PULSED;
radar.duty_cycle     = 0.1;

// 3. 启用 PSOS（可选）
esm.receiver.psos_enabled           = true;
esm.receiver.psos_confirm_threshold = 0.9;

// 4. 每帧调用 update()
std::vector<ESM_Transmitter> transmitters = {radar};
for (double t = 0; t < 30.0; t += 1.0)
{
    esm.update(t, transmitters);
    for (const auto& track : esm.tracks())
        // track.azimuth_rad, track.snr_dB, track.psos_confirmed ...
}
```

---

## 依赖

| 库 | 版本 | 用途 |
|----|------|------|
| [Eigen](https://eigen.tuxfamily.org) | 3.4.0 | 三维向量运算 |
| C++ 标准库 | C++11 | `vector` / `string` / `cmath` |
