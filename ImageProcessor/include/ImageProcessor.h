#pragma once

#include "IP_Interface.h"
#include <filter/FilterBase.h> // 复用 TargetMeasurementDataProcessor 的滤波器接口
#include <filter/KalmanFilterND.h> // 6D Kalman 滤波器（CV 模型）
#include <memory>
#include <vector>
#include <map>
#include <string>
#include <random>

namespace ImageProcessor
{

    /**
     * 目标识别状态枚举
     * -----------------
     *
     * 状态机转换路径（→ 表示帧达标过渡，─→ 表示延迟期满确认，⋯→ 表示 coast 降级）：
     *
     *   UNDETECTED → WAITING_DETECTION ─→ DETECTED → WAITING_CLASSIFICATION
     *       ↑                                    ⋮                    │
     *       └──────────────────────────────────── ⋯────────────────────┘
     *                                                   ↓
     *                                          CLASSIFIED → WAITING_IDENTIFICATION
     *                                               ⋮                    │
     *                                               └────────────────────┘
     *                                                                     ↓
     *                                                            IDENTIFIED
     *                                                                 ⋮
     *                                                          (coast 降级)
     */
    enum TargetRecognitionState
    {
        cST_UNDETECTED = 0,             //!< 未检测到（初始状态或 coast 超时降级）
        cST_WAITING_DETECTION = 1,      //!< 检测确认等待中（帧达标，等待延迟期满）
        cST_DETECTED = 2,               //!< 已检测到（确认目标存在）
        cST_WAITING_CLASSIFICATION = 3, //!< 分类确认等待中
        cST_CLASSIFIED = 4,             //!< 已分类（确认目标类别，如：坦克/卡车）
        cST_WAITING_IDENTIFICATION = 5, //!< 识别确认等待中
        cST_IDENTIFIED = 6              //!< 已识别（确认目标具体型号，如：T-72/M1A2）
    };

    /**
     * RecognitionStatus
     * -----------------
     * 目标识别状态机辅助类。
     *
     * 职责：
     *   - 管理单个目标跨帧的识别状态
     *   - 记录状态进入时间与最早退出时间（延迟确认）
     *   - 记录 coast 开始时间与 coast 期间最佳帧状态
     *   - 每帧标记 stale，帧处理完毕后未更新的 stale 状态执行 coast 降级
     *
     * 典型生命周期：
     *   Create  → 每帧 evaluateObjectState → 可能状态迁移
     *          → 帧末 processCoastDowngrade → 超时降级或保持
     *          → 航迹回收时 erase
     */
    class RecognitionStatus
    {
    public:
        RecognitionStatus();
        explicit RecognitionStatus(const char *truthName);
        ~RecognitionStatus() = default;

        TargetRecognitionState CurrentState() const { return m_state; }
        double StateEntryTime() const { return m_entryTime; }
        double EarliestStateExitTime() const { return m_earliestExitTime; }
        bool IsStale() const { return m_isStale; }
        void SetStale() { m_isStale = true; }

        /**
         * 进入新状态。
         * @param state   目标状态
         * @param simTime 进入时刻
         * @param delay   最小停留时间（延迟确认），earliestExitTime = simTime + delay
         */
        void EnterState(TargetRecognitionState state,
                        double simTime,
                        double delay);

        /**
         * 检查 coast 是否超时。
         * @param aSimTime  当前仿真时间
         * @param frameState 当前帧状态（用于记录 coast 期间最佳状态）
         * @param coastTime  允许的最大 coast 时间
         * @return true=已超时，调用方应当触发状态降级
         *
         * 首次调用时记录 coastStartTime = aSimTime 和初始 coastingState = frameState。
         * 后续调用若 frameState 更高则更新 coastingState（记录 coast 期间最佳帧状态）。
         */
        bool CoastTimeExceeded(double aSimTime,
                               TargetRecognitionState frameState,
                               double coastTime);

        TargetRecognitionState CoastingState() const { return m_coastingState; }
        void SetLastGoodUpdateTime(double t) { m_lastGoodUpdateTime = t; }
        const char *TruthName() const { return m_truthName.c_str(); }

    private:
        TargetRecognitionState m_state;         //!< 当前状态
        double m_entryTime;                     //!< 进入当前状态的仿真时间
        double m_earliestExitTime;              //!< 允许退出的最早时间（= m_entryTime + 延迟）
        double m_lastGoodUpdateTime;            //!< 末次满足等级要求的帧时间
        double m_coastingStartTime;             //!< 开始 coast 的时间（<0 表示未 coast）
        TargetRecognitionState m_coastingState; //!< coast 期间记录的最佳帧状态
        bool m_isStale;                         //!< 本帧是否未收到更新（帧首置 true，处理到则置 false）
        std::string m_truthName;                //!< 目标真值名称（调试用）
    };

    /**
     * TrackState
     * ----------
     * 单目标视频流航迹持久状态。
     *
     * 视频流场景下，每个目标（按 truthIndex 索引）在首帧创建 TrackState，
     * 后续帧复用其中的 trackId 和 filter。
     *
     * 分类/识别时间戳（classifiedTime/identifiedTime）跨帧持久化，
     * 确保"状态不后退"——一旦目标达到 IDENTIFIED，即使后续帧概率下降
     * 也始终报告 IDENTIFIED。
     */
    struct TrackState
    {
        int trackId;           //!< 航迹 ID（首帧分配，跨帧不变）
        int streamNumber;      //!< 视频流编号（流切换时重置滤波器）
        double classifiedTime; //!< 最佳分类时间戳（-1=未分类），跨帧持久化
        double identifiedTime; //!< 最佳识别时间戳（-1=未识别），跨帧持久化
        std::string truthName; //!< 真值目标名
        // 滤波器（复用 TargetMeasurementDataProcessor 的滤波器接口）
        std::unique_ptr<TargetMeasurement::Filter::FilterBase> filter; //!< 可选滤波器（接口预留）
        int filterUpdateCount;                                          //!< 滤波器更新次数（用于判断稳定性）
        IP_TrackOutput output;                                          //!< 最近帧输出（用于 purgeOldTracks 判定）

        TrackState()
            : trackId(-1), streamNumber(-1), classifiedTime(-1.0), identifiedTime(-1.0)
            , filterUpdateCount(0)
        {
        }
    };

    /**
     * ImageProcessor
     * ---------------
     * 图像目标探测、跟踪与识别核心处理器。
     *
     * 核心功能：
     *   1. ProcessImage — 处理单帧图像中的全部检测目标
     *       - 目标识别状态评估（Johnson 准则 + 7 状态 FSM）
     *       - 航迹创建/更新（视频流持续跟踪 / 静态图像一次性报告）
     *       - 可选滤波器平滑（Alpha-Beta / Kalman，接口预留）
     *       - 航迹属性填充（位置/速度/SNR/方位俯仰/像素数）
     *       - 分类/识别时间戳持久化（状态不后退）
     *
     *   2. 航迹生命周期管理
     *       - 创建（首帧）→ 更新（后续帧）→ Coast（目标丢失）→ 回收（超时）
     *       - purgeOldTracks：超 coastTime 的视频流航迹自动清理
     *
     *   3. 目标识别状态机
     *       - Johnson 准则概率计算（检测/分类/识别三级）
     *       - 7 状态 FSM 带延迟确认和 coast 降级
     *       - 跨帧持久化的 RecognitionStatus
     *
     *   4. 滤波器接口（⚠️ 预留，待扩展）
     *       - FilterBase 抽象基类
     *       - 通过 initializeFilter() 工厂方法注入
     *
     * 典型用法：
     * @code
     *   ImageProcessor proc;
     *   proc.Initialize(&config);
     *   proc.ProcessImage(&input, &output);
     *   // output.tracks[0..trackCount-1] 包含航迹结果
     * @endcode
     */
    class ImageProcessor
    {
    public:
        ImageProcessor();
        ~ImageProcessor();

        /**
         * 初始化处理器。
         * @param cfg 配置参数（若 cfg=nullptr 则使用全零默认值）
         * @return true=成功
         */
        bool Initialize(const IP_Config *cfg);

        /**
         * 处理一帧图像。
         * @param input  图像帧输入（传感器参数 + 检测目标列表）
         * @param output 处理输出（航迹数组，最大 32 条）
         * @return 0=成功，-1=参数无效或未初始化
         *
         * 处理步骤：
         *   1. 标记所有 RecognitionStatus 为 stale（帧首）
         *   2. 遍历每个 IP_DetectedObject：
         *      a. evaluateObjectState() — Johnson 准则 + FSM
         *      b. 若 UNDETECTED，跳过
         *      c. 视频流：查找/创建 TrackState
         *      d. 静态图像：创建一次性航迹
         *      e. 填充位置、SNR、方位俯仰、速度
         *      f. updateTrackAuxData() — 分类/识别时间戳
         *   3. purgeOldTracks() — 清理超时航迹（仅视频流）
         *   4. processCoastDowngrade() — stale 状态 coast 降级
         */
        int ProcessImage(const IP_ImageInput *input,
                         IP_ProcessOutput *output);

        /**
         * 无探测更新（目标丢失时外推）。
         * 当前为占位实现，滤波器注入后补充 coast 外推逻辑。
         */
        int NoDetectUpdate(double simTime,
                           IP_TrackOutput *output);

        /** 重置所有航迹和状态机状态。 */
        int Reset();

        /** 获取当前滤波器生命周期状态。 */
        int GetFilterState() const;

        /** 运行时修改配置。 */
        int SetConfig(const IP_Config *cfg);

        /** 获取当前配置。 */
        int GetConfig(IP_Config *cfg) const;

    private:
        // ======================================================================
        // 滤波器工厂
        // ======================================================================
        /**
         * 初始化滤波器原型。
         * 根据 m_config.filterType 创建对应 FilterBase 子类实例，
         * 存入 m_filterPrototype。视频流新航迹通过 createFilterForTrack()
         * 获取独立实例。
         */
        bool initializeFilter();

        /**
         * 为视频流新航迹创建配置好的 KalmanFilterND 实例。
         * 6 维状态：[x, y, z, vx, vy, vz]，匀速模型，3 维位置测量。
         */
        TargetMeasurement::Filter::KalmanFilterND* createKalmanFilter() const;

        // ======================================================================
        // Johnson 准则概率计算
        // ======================================================================
        /**
         * 计算检测概率。
         * Johnson 准则公式：P(N) = N^(2.7+0.7N) / (1 + N^(2.7+0.7N))
         * 其中 N = objectSize / SAF。
         * @param objectSize 目标尺寸（线对数或像素数，由 averageAspectRatio 决定）
         * @return 概率 [0, 1]
         */
        double computeProbabilityOfDetection(double objectSize) const;

        /** 计算分类概率（公式同上，使用 classificationSAF）。 */
        double computeProbabilityOfClassification(double objectSize) const;

        /** 计算识别概率（公式同上，使用 identificationSAF）。 */
        double computeProbabilityOfIdentification(double objectSize) const;

        // ======================================================================
        // 目标识别状态评估
        // ======================================================================
        /**
         * 评估单个目标的识别状态。
         * @param simTime  当前仿真时间
         * @param image    图像帧输入（含噪声电平）
         * @param object   检测目标
         * @param outRecognitionState [out] 经过 FSM 后的最终状态
         *
         * 步骤如下：
         *   1. 像素数 → 线对数（若 averageAspectRatio > 0）
         *   2. 计算三级 Johnson 概率（检测/分类/识别）
         *   3. 均匀随机 draw 确定帧状态
         *   4. 帧状态 × 像素数硬门槛 → 最终帧状态
         *   5. 获取/创建 RecognitionStatus
         *   6. 执行 7 状态 FSM 过渡（含 coast 检查）
         *   7. 保存状态到 m_statusList
         */
        void evaluateObjectState(double simTime,
                                 const IP_ImageInput &image,
                                 const IP_DetectedObject &object,
                                 int &outRecognitionState);

        // ======================================================================
        // 航迹管理
        // ======================================================================
        /**
         * 清理超时航迹。
         * 遍历 m_trackList，若 imageTime - lastUpdateTime > coastTime，
         * 则删除该 TrackState 及其对应的 RecognitionStatus。
         */
        void purgeOldTracks(double simTime,
                            double imageTime);

        // ======================================================================
        // 协方差传播
        // ======================================================================
        /**
         * 计算测量协方差矩阵（3×3，WCS 坐标）。
         * 将传感器 RBE 误差（方位/俯仰/距离）通过 Jacobian
         * 变换到 WCS 坐标系。
         * @param range   斜距（米）
         * @param az      方位角（弧度）
         * @param el      俯仰角（弧度）
         * @param input   图像帧输入（含传感器误差参数）
         * @param cov[9]  输出 3×3 协方差（行优先）
         */
        void computeMeasurementCovariance(double range, double az, double el,
                                          const IP_ImageInput &input,
                                          double cov[9]) const;

        // ======================================================================
        // 状态机辅助
        // ======================================================================
        /**
         * 将 FSM 状态映射为结果字符串。
         * 映射规则：
         *   IDENTIFIED                 → "IDENTIFIED"
         *   WAITING_IDENTIFICATION / CLASSIFIED → "CLASSIFIED"
         *   WAITING_CLASSIFICATION / DETECTED   → "DETECTED"
         *   其他                       → "UNDETECTED"
         */
        std::string stateToResultString(int state) const;

        // ======================================================================
        // Coast 降级
        // ======================================================================
        /**
         * 帧处理完毕后执行 coast 降级。
         * 未被本帧更新的 RecognitionStatus（stale）检查 coast 超时，
         *   超时则降级到 UNDETECTED。
         *
         * 各状态的 coast 时间：
         *   DETECTED   → detectionCoastTime
         *   CLASSIFIED → classificationCoastTime
         *   IDENTIFIED → identificationCoastTime
         *   其他       → transitionCoastTime
         */
        void processCoastDowngrade(double simTime);

        // ======================================================================
        // UpdateTrack — Aux Data
        // ======================================================================
        /**
         * 更新航迹的辅助数据（分类/识别时间戳）。
         * 状态不后退——一旦达到 IDENTIFIED，永不回退。
         *   - 分类/识别时间戳在视频流中通过 TrackState 跨帧持久化
         *   - 静态图像（无 TrackState）仅用当前帧结果
         *
         * 决策逻辑：
         *   1. 若持久化已识别 OR 帧结果 == "IDENTIFIED"
         *      → classifiedTime = identifiedTime = simTime
         *   2. 否则若持久化已分类 OR 帧结果 == "CLASSIFIED"
         *      → classifiedTime = simTime, identifiedTime = -1
         *   3. 否则
         *      → classifiedTime = identifiedTime = -1
         */
        void updateTrackAuxData(int targetIndex,
                                bool isVideoStream,
                                double simTime,
                                const std::string &frameResult,
                                IP_TrackOutput &track);

        // ======================================================================
        // 成员变量
        // ======================================================================
        IP_Config m_config; //!< 运行配置
        bool m_initialized; //!< 初始化标志

        std::unique_ptr<TargetMeasurement::Filter::FilterBase> m_filterPrototype; //!< 滤波器原型（接口预留）
        std::map<int, TrackState> m_trackList;                                    //!< 视频流航迹表 key=targetIndex
        std::map<int, RecognitionStatus> m_statusList;                            //!< 目标识别状态表 key=targetIndex
        int m_nextTrackId;                                                        //!< 下一个可用的航迹 ID

        std::mt19937 m_rng; //!< Johnson 准则均匀随机数生成器

        static const char *sStateNames[7]; //!< 状态名数组（调试输出）
    };

} // namespace ImageProcessor
