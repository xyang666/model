#pragma once

#ifdef _WIN32
#ifdef IP_EXPORTS
#define IP_EXPORT __declspec(dllexport)
#else
#define IP_EXPORT __declspec(dllimport)
#endif
#else
#define IP_EXPORT __attribute__((visibility("default")))
#endif

/*
 * ImageProcessor
 * --------------
 * 图像目标探测、跟踪与识别处理模块。
 *
 * 接收图像/视频帧输入，提取检测目标
 * 为每个目标维护航迹（视频流持续跟踪 / 静态图像一次性报告）
 * 基于 Johnson 准则的目标识别状态机（检测→分类→识别）
 * 可选滤波器平滑测量
 * 输出航迹：位置、速度、协方差、识别状态、像素信息等
 *
 * 对外提供 C API，内部封装 ImageProcessor 核心实现。
 */

// ============================================================================
// 传感器类型
// ============================================================================
#define IP_SENSOR_EO 0  /**< 光电（可见光） */
#define IP_SENSOR_IR 1  /**< 红外 */
#define IP_SENSOR_SAR 2 /**< 合成孔径雷达 */

// ============================================================================
// 目标识别状态（图像处理器目标识别状态枚举）
// ============================================================================
#define IP_STATE_UNDETECTED 0
#define IP_STATE_WAITING_DETECTION 1
#define IP_STATE_DETECTED 2
#define IP_STATE_WAITING_CLASSIFICATION 3
#define IP_STATE_CLASSIFIED 4
#define IP_STATE_WAITING_IDENTIFICATION 5
#define IP_STATE_IDENTIFIED 6

// ============================================================================
// 滤波器生命周期状态
// ============================================================================
#define IP_FILTER_UNINITIALIZED 0
#define IP_FILTER_INITIALIZING 1
#define IP_FILTER_TRACKING 2
#define IP_FILTER_COASTING 3

// ============================================================================
// 滤波器类型
// ============================================================================
#define IP_FILTER_ALPHA_BETA 0
#define IP_FILTER_ALPHA_BETA_GAMMA 1
#define IP_FILTER_KALMAN 2
#define IP_FILTER_EKF 3
#define IP_FILTER_UKF 4
#define IP_FILTER_NONE -1

// 滤波器生命周期状态
#define IP_FILTER_STATE_UNINITIALIZED 0
#define IP_FILTER_STATE_INITIALIZING 1
#define IP_FILTER_STATE_TRACKING 2
#define IP_FILTER_STATE_COASTING 3

// 注：滤波器状态/类型常量与 TargetMeasurementDataProcessor 的
// TM_STATE_* / TM_FILTER_* 含义一致，值对齐。
// 本项目 C API 使用 IP_FILTER_* 前缀。
// 具体滤波器实现请参考 TargetMeasurementDataProcessor 的 filter/ 目录。

#ifdef __cplusplus
extern "C"
{
#endif

    // ============================================================================
    // 配置参数
    // ============================================================================
    /**
     * IP_Config
     * ---------
     * 图像处理器运行配置参数。
     */
    struct IP_Config
    {
        int filterType;            /**< 滤波器类型 */
        double processNoiseSigmaX; /**< 过程噪声 X 方向标准差 */
        double processNoiseSigmaY; /**< 过程噪声 Y 方向标准差 */
        double processNoiseSigmaZ; /**< 过程噪声 Z 方向标准差 */

        double minDetectPixelCount; /**< 最小检测像素数 */
        double minClassPixelCount;  /**< 最小分类像素数 */
        double minIdentPixelCount;  /**< 最小识别像素数 */
        double averageAspectRatio;  /**< 目标平均长宽比（用于 Johnson 准则） */

        double detectionSAF;      /**< 检测场景分析因子（Johnson N50） */
        double classificationSAF; /**< 分类场景分析因子 */
        double identificationSAF; /**< 识别场景分析因子 */

        double detectionDelayTime;      /**< 检测延迟时间（秒） */
        double classificationDelayTime; /**< 分类延迟时间（秒） */
        double identificationDelayTime; /**< 识别延迟时间（秒） */

        double coastTime;               /**< 航迹保持超时（秒） */
        double detectionCoastTime;      /**< 检测态 coast 时间 */
        double classificationCoastTime; /**< 分类态 coast 时间 */
        double identificationCoastTime; /**< 识别态 coast 时间 */
        double transitionCoastTime;     /**< 过渡态 coast 时间 */

        bool targetRecognitionEnabled;  /**< 是否启用目标识别状态机 */
        bool reportsVelocity;           /**< 是否报告速度 */
        bool reportsSide;               /**< 是否报告敌我属性 */
        bool reportsType;               /**< 是否报告目标类型 */
        bool reportsBearingElevation;   /**< 是否报告方位角/俯仰角 */
        bool includeUnstableCovariance;         /**< 是否包含未稳定状态协方差 */
        bool includeUnstableResidualCovariance;  /**< 是否包含未稳定残差协方差 */
        unsigned int randomSeed;                 /**< 随机数种子（用于 Johnson 准则 draw） */
    };

    // ============================================================================
    // 图像帧中的检测目标
    // ============================================================================
    /**
     * IP_DetectedObject
     * -----------------
     * 图像中单个检测目标的属性。
     */
    struct IP_DetectedObject
    {
        double pixelCount;     /**< 目标像素数 */
        double pixelIntensity; /**< 目标像素强度 */
        double signalLevel;    /**< 信号电平 */
        double locationWCS[3]; /**< 目标位置（WCS 坐标，米） */
        int truthIndex;        /**< 真值索引 */
        int truthType;         /**< 真值类型 ID */
        int truthSide;         /**< 真值敌我属性 */
        const char *truthName; /**< 真值目标名称 */
    };

    // ============================================================================
    // 图像帧输入
    // ============================================================================
    /**
     * IP_ImageInput
     * -------------
     * 单帧图像输入，包含传感器参数和所有检测目标。
     */
    struct IP_ImageInput
    {
        double simTime;         /**< 仿真时间（秒） */
        int sensorType;         /**< 传感器类型 */
        double sensorLocWCS[3]; /**< 传感器位置（WCS） */
        double bearingError;    /**< 方位角误差标准差（弧度） */
        double elevationError;  /**< 俯仰角误差标准差（弧度） */
        double rangeError;      /**< 距离误差标准差（米） */
        double rangeRateError;  /**< 径向速率误差标准差（米/秒） */
        double noiseLevel;      /**< 图像噪声电平 */
        double backgroundLevel; /**< 图像背景电平 */
        int imageWidth;         /**< 图像宽度（像素） */
        int imageHeight;        /**< 图像高度（像素） */

        bool isVideoStream; /**< 是否为视频流（true=持续跟踪，false=静态图像） */
        int streamNumber;   /**< 视频流编号 */
        int imageNumber;    /**< 帧编号（>0 表示视频流） */

        int objectCount;                  /**< 检测目标数量 */
        const IP_DetectedObject *objects; /**< 检测目标数组 */
    };

    // ============================================================================
    // 航迹输出
    // ============================================================================
    /**
     * IP_TrackOutput
     * --------------
     * 单条目标航迹的输出结果。
     */
    struct IP_TrackOutput
    {
        double posX, posY, posZ;         /**< 目标位置（ECEF，米） */
        double velX, velY, velZ;         /**< 目标速度（ECEF，米/秒） */
        double latitude;                 /**< 纬度（弧度） */
        double longitude;                /**< 经度（弧度） */
        double altitude;                 /**< 高度（米） */
        double stateCovariance[36];        /**< 6×6 状态协方差（行优先） */
        double measurementCovariance[9];   /**< 3×3 测量协方差（行优先） */
        double residualCovariance[9];      /**< 3×3 残差协方差（行优先），来自滤波器 innovationCovariance */

        double bearing;        /**< 方位角（弧度） */
        double elevation;      /**< 俯仰角（弧度） */
        double range;          /**< 斜距（米） */
        double signalToNoise;  /**< 信噪比 */
        double pixelCount;     /**< 目标像素数 */
        double classifiedTime; /**< 分类时间戳（-1.0=未分类） */
        double identifiedTime; /**< 识别时间戳（-1.0=未识别） */

        int targetIndex;      /**< 目标索引 */
        int trackId;          /**< 航迹 ID */
        int recognitionState; /**< 当前识别状态，IP_STATE_* */
        int filterState;      /**< 滤波器状态 */
        int valid;            /**< 输出有效标志 */
        double updateTime;    /**< 更新时刻 */
    };

    // ============================================================================
    // 图像处理汇总输出
    // ============================================================================
    /**
     * IP_ProcessOutput
     * ----------------
     * 单帧图像处理完成后汇总输出，包含所有航迹。
     */
    struct IP_ProcessOutput
    {
        int trackCount;            /**< 有效航迹数量 */
        IP_TrackOutput tracks[32]; /**< 航迹数组（最大 32 条） */
        int frameState;            /**< 帧处理状态 */
    };

    // ============================================================================
    // 对外 C API
    // ============================================================================

    /**
     * 初始化图像处理器模块。
     */
    IP_EXPORT bool IP_Initialize(const IP_Config *cfg);

    /**
     * 释放模块资源。
     */
    IP_EXPORT bool IP_Finalize();

    /**
     * 处理一帧图像，提取目标并输出航迹。
     * @param input  图像帧输入
     * @param output 处理输出（航迹列表）
     * @return 0=成功，负值=失败
     */
    IP_EXPORT int IP_ProcessImage(const IP_ImageInput *input,
                                  IP_ProcessOutput *output);

    /**
     * 无探测更新（目标丢失时外推）。
     */
    IP_EXPORT int IP_NoDetectUpdate(double simTime,
                                    IP_TrackOutput *output);

    /**
     * 重置所有状态。
     */
    IP_EXPORT int IP_Reset();

    /**
     * 获取当前滤波器状态。
     */
    IP_EXPORT int IP_GetFilterState();

    /**
     * 运行时修改配置。
     */
    IP_EXPORT int IP_SetConfig(const IP_Config *cfg);

    /**
     * 获取当前配置。
     */
    IP_EXPORT int IP_GetConfig(IP_Config *cfg);

#ifdef __cplusplus
}
#endif
