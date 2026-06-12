#pragma once

#ifdef _WIN32
#ifdef TM_EXPORTS
#define TM_EXPORT __declspec(dllexport)
#else
#define TM_EXPORT __declspec(dllimport)
#endif
#else
#define TM_EXPORT __attribute__((visibility("default")))
#endif

/*
 * TargetMeasurementDataProcessingModel
 * -----------------------------------
 * 目标测量数据处理模块，用于将雷达 / 光电 / 红外传感器测量值
 * 通过误差传递与滤波器输出目标位置、速度和质量指标。
 */

// 传感器类型枚举
#define TM_SENSOR_RADAR 0 /**< 雷达 */
#define TM_SENSOR_EO 1    /**< 光电 */
#define TM_SENSOR_IR 2    /**< 红外 */

#include "filter/FilterTypes.h"

// 坐标参考系枚举
#define TM_FRAME_ECEF 0  /**< 地心地固直角坐标系 */
#define TM_FRAME_WORLD 0 /**< 世界系直角坐标（与 ECEF 一致） */
#define TM_FRAME_NED 1   /**< 北-东-地 局部坐标系 */
#define TM_FRAME_ENU 2   /**< 东-北-天 局部坐标系 */
#define TM_FRAME_BODY 3   /**< 传感器本体坐标系 */
#define TM_FRAME_LLA 4    /**< WGS84 大地坐标 [纬度, 经度, 高度] */
#define TM_FRAME_ECI 5    /**< 地心惯性系（J2000） */
#define TM_FRAME_ORBITAL 6 /**< 轨道系：x-矢径方向(+R), z-轨道正法向 */

#ifdef __cplusplus
extern "C"
{
#endif

    // ============================================================================
    // 配置参数
    // ============================================================================
    struct TM_Config
    {
        int sensorType;      /**< 传感器类型，TM_SENSOR_* */
        int filterType;      /**< 滤波器类型，TM_FILTER_* */
        int track_init_m;    /**< 航迹建立所需 M 次检测 */
        int track_init_n;    /**< 航迹建立窗口 N */
        int track_maint_m;   /**< 航迹维持所需 M 次检测 */
        int track_maint_n;   /**< 航迹维持窗口 N */
        double track_coast_s; /**< 航迹 coast 超时时间（秒） */
    };

    // ============================================================================
    // 测量输入（通用：传感器位置 / 姿态）
    // ============================================================================
    /**
     * 传感器本体系定义（右手系）：
     *   x — 前向（瞄准线 boresight）
     *   y — 右向
     *   z — 下向
     *
     * 球坐标 RBE（在本体系中）：
     *   range     — 斜距
     *   azimuth   — 方位角，x-y 平面内从 x 轴逆时针为正
     *   elevation — 俯仰角，从 x-y 平面向上为正
     *
     * attitude 为坐标转换矩阵（行优先），将向量从本体系坐标转换到 referenceFrame 系坐标。
     *   例如 referenceFrame = TM_FRAME_NED 时：v_ned = attitude * v_body。
     */
    struct TM_MeasurementInput
    {
        double simTime;     /**< 仿真时间，必须单调递增 */
        int targetId;       /**< 目标 ID（-1=不关联到特定目标, >=0 为具体目标） */
        int sensorType;     /**< 传感器类型，TM_SENSOR_* */
        int referenceFrame; /**< 传感器位置 / 姿态的参考系，TM_FRAME_* */
        double position[3]; /**< 传感器位置（米） */
        double attitude[9]; /**< 坐标转换矩阵 3×3 行优先，将向量从本体系坐标转换到 referenceFrame 系坐标 */
    };

// ============================================================================
// 传感器专用测量结构体
// ============================================================================
#define TM_MEAS_RANGE (1 << 0)      /**< 距离测量有效 */
#define TM_MEAS_AZIMUTH (1 << 1)    /**< 方位角测量有效 */
#define TM_MEAS_ELEVATION (1 << 2)  /**< 俯仰角测量有效 */
#define TM_MEAS_RANGE_RATE (1 << 3) /**< 径向速率测量有效 */

    /** @brief 雷达测量值（RBE + 径向速率） */
    struct TM_RadarMeasurement
    {
        TM_MeasurementInput input; /**< 通用测量输入（时间、位置、姿态） */
        double range;              /**< 斜距（米） */
        double azimuth;            /**< 方位角（弧度，正北为0，顺时针为正） */
        double elevation;          /**< 俯仰角（弧度，水平面以上为正） */
        double rangeRate;          /**< 径向速率（米/秒，目标远离为正） */

        double rangeError;     /**< 距离误差标准差（米） */
        double azimuthError;   /**< 方位角误差标准差（弧度） */
        double elevationError; /**< 俯仰角误差标准差（弧度） */
        double rangeRateError; /**< 速率误差标准差（米/秒） */

        int validFlags; /**< TM_MEAS_* 组合 */
    };

    /** @brief 光电测量值（像素坐标） */
    struct TM_EOMeasurement
    {
        TM_MeasurementInput input; /**< 通用测量输入（时间、位置、姿态） */
        double pixelU;             /**< 像素列坐标 */
        double pixelV;             /**< 像素行坐标 */
        double pixelErrorU;        /**< 列坐标误差标准差（像素） */
        double pixelErrorV;        /**< 行坐标误差标准差（像素） */
        int validFlags;            /**< 有效位标志 */
    };

    /** @brief 红外测量值（像素坐标） */
    struct TM_IRMeasurement
    {
        TM_MeasurementInput input; /**< 通用测量输入（时间、位置、姿态） */
        double pixelU;             /**< 像素列坐标 */
        double pixelV;             /**< 像素行坐标 */
        double pixelErrorU;        /**< 列坐标误差标准差（像素） */
        double pixelErrorV;        /**< 行坐标误差标准差（像素） */
        int validFlags;            /**< 有效位标志 */
    };

    // ============================================================================
    // 定位结果输出
    // ============================================================================
    struct TM_LocationOutput
    {
        double position[3]; /**< 位置（米，参考系由 referenceFrame 指定） */
        double velocity[3]; /**< 速度（米/秒，参考系由 referenceFrame 指定） */

        double stateCovariance[36]; /**< 6×6 状态协方差矩阵（行优先） */

        double positionRmsError; /**< 位置 DRMS 误差（米） */
        double velocityRmsError; /**< 速度 RMS 误差（米/秒） */
        double chiSquared;       /**< 卡方归一化距离 */
        double avgChiSquared;    /**< 5 帧窗口平均卡方 */
        double chiSquaredMean;   /**< 历史平均卡方 */
        double trackQuality;     /**< 跟踪质量指标 [0,1] */
        double sigmaA;           /**< 位置误差椭球最大轴（米） */
        double sigmaB;           /**< 位置误差椭球中轴（米） */
        double sigmaC;           /**< 位置误差椭球最小轴（米） */

        double updateTime;  /**< 本次输出更新时刻（秒） */
        int referenceFrame; /**< 输出参考系，TM_FRAME_* */
        int filterState;    /**< 滤波器状态，TM_STATE_* */
        int valid;          /**< 输出有效标志：1=有效，0/负数=无效 */
        int updateCount;    /**< 成功测量更新计数 */
    };

    // ============================================================================
    // 对外 C API
    // ============================================================================
    TM_EXPORT bool TM_Initialize(const TM_Config *cfg);
    TM_EXPORT bool TM_Finalize();

    TM_EXPORT int TM_ProcessMeasurement(const void *measurement,
                                        int measSize,
                                        TM_LocationOutput *output);

    TM_EXPORT int TM_NoDetectUpdate(double simTime, TM_LocationOutput *output);
    TM_EXPORT int TM_Reset();
    TM_EXPORT int TM_GetFilterState();
    TM_EXPORT double TM_GetChiSquaredMean();
    TM_EXPORT int TM_SetConfig(const TM_Config *cfg);
    TM_EXPORT int TM_GetConfig(TM_Config *cfg);
    TM_EXPORT int TM_GetTrackCount();
    TM_EXPORT int TM_GetTrack(int index, TM_LocationOutput *output);

#ifdef __cplusplus
}
#endif
