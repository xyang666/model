#include "ImageProcessor.h"
#include <cstring>   // memset
#include <cmath>     // sqrt, pow, atan2, asin, sin, cos
#include <cassert>

namespace ImageProcessor
{

// ======================================================================
// 状态名称映射（调试输出用）
// ======================================================================
// 索引与 TargetRecognitionState 枚举值对应：
//   0=UNDETECTED, 1=WAITING_DETECTION, 2=DETECTED,
//   3=WAITING_CLASSIFICATION, 4=CLASSIFIED,
//   5=WAITING_IDENTIFICATION, 6=IDENTIFIED
const char* ImageProcessor::sStateNames[] = {
    "UNDETECTED",
    "WAITING_DETECTION",
    "DETECTED",
    "WAITING_CLASSIFICATION",
    "CLASSIFIED",
    "WAITING_IDENTIFICATION",
    "IDENTIFIED"
};

// ======================================================================
// RecognitionStatus 实现
// ======================================================================

RecognitionStatus::RecognitionStatus()
    : m_state(cST_UNDETECTED)
    , m_entryTime(0.0)
    , m_earliestExitTime(0.0)
    , m_lastGoodUpdateTime(-1.0e30)
    , m_coastingStartTime(-1.0)
    , m_coastingState(cST_UNDETECTED)
    , m_isStale(false)
    , m_truthName("")
{
}

RecognitionStatus::RecognitionStatus(const char* truthName)
    : m_state(cST_UNDETECTED)
    , m_entryTime(0.0)
    , m_earliestExitTime(0.0)
    , m_lastGoodUpdateTime(-1.0e30)
    , m_coastingStartTime(-1.0)
    , m_coastingState(cST_UNDETECTED)
    , m_isStale(false)
    , m_truthName(truthName ? truthName : "")
{
}

// ----------------------------------------------------------------------
// EnterState — 进入新状态
// ----------------------------------------------------------------------
// 设置状态、进入时刻和最早退出时间（= 进入时间 + 确认延迟）。
// 重置 coast 计时器，清除 stale 标记。
void RecognitionStatus::EnterState(TargetRecognitionState state,
                                    double simTime,
                                    double delay)
{
    m_state = state;
    m_entryTime = simTime;
    m_earliestExitTime = simTime + delay;  // 延迟确认：必须等到此时间之后才能升级
    m_coastingStartTime = -1.0;             // 重置 coast
    m_coastingState = cST_UNDETECTED;
    m_isStale = false;
}

// ----------------------------------------------------------------------
// CoastTimeExceeded — 检查 coast 是否超时
// ----------------------------------------------------------------------
// Coast 机制：当帧状态不足以维持当前等级时进入 coast。
// - 首次调用（coastingStartTime < 0）：记录开始时间，记录当前帧状态
// - 后续调用：若帧状态比已记录的状态更高则更新 coastingState
// - 返回 true 表示 coast 时间已耗尽，调用方应执行降级
//
// 注意：coastingState 记录的是 coast 期间遇到的最佳帧状态，
//       而非 coast 开始时的状态。这意味着降级时可能回到 DETECTED
//       而非直接回到 UNDETECTED。
//
// 例如：在 IDENTIFIED 态 coast，期间帧状态达到 CLASSIFIED，
//       超时后降级到 CLASSIFIED 而非 UNDETECTED。
bool RecognitionStatus::CoastTimeExceeded(double aSimTime,
                                           TargetRecognitionState frameState,
                                           double coastTime)
{
    if (m_coastingStartTime < 0.0)
    {
        // 首次 coast，记录起始时间和当前帧状态
        m_coastingStartTime = aSimTime;
        m_coastingState = frameState;
    }
    else if (frameState > m_coastingState)
    {
        // 更新 coast 期间的最佳帧状态
        m_coastingState = frameState;
    }
    m_isStale = false;  // 有检测机会就不算 stale
    return ((aSimTime - m_coastingStartTime) >= coastTime);
}

// ======================================================================
// ImageProcessor 构造/析构
// ======================================================================

ImageProcessor::ImageProcessor()
    : m_initialized(false)
    , m_nextTrackId(1)
    , m_rng(42u)  // 默认种子 42
{
    std::memset(&m_config, 0, sizeof(m_config));
}

ImageProcessor::~ImageProcessor()
{
}

// ======================================================================
// Initialize — 初始化处理器
// ======================================================================
// 执行步骤：
//   1. 复制配置
//   2. 清空航迹表和状态表
//   3. 重置航迹 ID 计数器
//   4. 用配置中的随机种子初始化 RNG
//   5. 调用滤波器工厂（当前返回 true，接口预留）
bool ImageProcessor::Initialize(const IP_Config* cfg)
{
    if (!cfg)
        return false;

    m_config = *cfg;
    m_trackList.clear();
    m_statusList.clear();
    m_nextTrackId = 1;

    // Johnson 准则使用均匀随机数决定帧状态，
    // 固定种子可确保仿真结果可复现
    m_rng.seed(m_config.randomSeed);

    if (!initializeFilter())
        return false;

    m_initialized = true;
    return true;
}

// ======================================================================
// ProcessImage — 核心方法：处理一帧图像
// ======================================================================
// 完整处理流水线：
//
//   [帧首]
//   ├── ImageProcessingInitiated：所有 RecognitionStatus 标记 stale
//   │
//   ├── [循环：每个检测目标]
//   │   ├── 1. evaluateObjectState — Johnson 准则 + 7 状态 FSM
//   │   ├── 2. 结果 UNDETECTED → 跳过（不可检测）
//   │   ├── 3. 视频流 → 查找/创建 TrackState（含 trackId 持久化）
//   │   │     静态图像 → 创建一次性航迹
//   │   ├── 4. 填充基础属性（位置、像素数、信噪比）
//   │   ├── 5. 速度（静态图像报 0，视频流待滤波器补充）
//   │   ├── 6. 方位角/俯仰角（可选报告）
//   │   ├── 7. 传感器误差传播
//   │   └── 8. updateTrackAuxData — 分类/识别时间戳持久化
//   │
//   ├── purgeOldTracks — 清理超时视频流航迹
//   │
//   └── [帧尾] processCoastDowngrade — stale 状态 coast 降级
//
// 视频流与静态图像的差异：
//   - 视频流（imageNumber > 0）：TrackState 跨帧持久化，航迹 ID 不变
//   - 静态图像（imageNumber == 0）：每帧新航迹，不保留状态
//
// @param input  图像帧输入（传感器位置/噪声/检测目标列表）
// @param output 航迹输出（最大 32 条/IP_TrackOutput）
// @return 0=成功, -1=参数无效
int ImageProcessor::ProcessImage(const IP_ImageInput* input,
                                  IP_ProcessOutput* output)
{
    if (!m_initialized || !input || !output)
        return -1;

    std::memset(output, 0, sizeof(*output));

    // ---- ImageProcessingInitiated：标记所有 status 为 stale ----
    // 后续遍历目标时，evaluateObjectState 会调用 SetLastGoodUpdateTime()
    // 清除对应 status 的 stale 标记。
    // 帧尾 processCoastDowngrade 检查仍为 stale 的 status 执行降级。
    if (m_config.targetRecognitionEnabled)
    {
        for (auto& sli : m_statusList)
            sli.second.SetStale();
    }

    // 判断是否为视频流：imageNumber > 0 表示视频流
    bool isVideoStream = (input->imageNumber > 0);

    // ---- 遍历每个检测目标 ----
    for (int i = 0; i < input->objectCount && output->trackCount < 32; ++i)
    {
        const IP_DetectedObject& obj = input->objects[i];

        int  frameState = cST_UNDETECTED;
        std::string resultStr = "UNDETECTED";

        // ----------------------------------------------------------------
        // 步骤 1：目标识别状态评估（Johnson 准则 + FSM）
        // ----------------------------------------------------------------
        if (m_config.targetRecognitionEnabled)
        {
            evaluateObjectState(input->simTime, *input, obj, frameState);
            resultStr = stateToResultString(frameState);
            if (resultStr == "UNDETECTED")
                continue; // 本帧不可检测，跳过
        }
        else
        {
            // 识别关闭时：全部视为检测到（目标识别状态机旁路）
            frameState = cST_DETECTED;
            resultStr = "DETECTED";
        }

        // ----------------------------------------------------------------
        // 步骤 2：创建/查找航迹
        // ----------------------------------------------------------------
        IP_TrackOutput& track = output->tracks[output->trackCount];

        // 清空输出行（避免残余数据）
        std::memset(&track, 0, sizeof(track));
        track.targetIndex = obj.truthIndex;
        track.recognitionState = frameState;
        track.pixelCount = obj.pixelCount;
        track.updateTime = input->simTime;
        track.filterState = IP_FILTER_STATE_TRACKING;
        track.classifiedTime = -1.0;
        track.identifiedTime = -1.0;

        // 位置（直接来自上游检测结果）
        track.posX = obj.locationWCS[0];
        track.posY = obj.locationWCS[1];
        track.posZ = obj.locationWCS[2];

        if (isVideoStream)
        {
            // ---- 视频流分支：持续跟踪 ----
            auto it = m_trackList.find(obj.truthIndex);
            if (it != m_trackList.end())
            {
                // 已有航迹：复用 trackId，继承分类/识别时间戳
                TrackState& ts = it->second;
                track.trackId = ts.trackId;

                // 流编号变化表示视频源切换，重置滤波器状态
                if (ts.streamNumber != input->streamNumber)
                {
                    ts.streamNumber = input->streamNumber;
                    if (ts.filter)
                        ts.filter->reset();
                }

                // 继承跨帧持久化的最佳识别状态（状态不后退）
                track.classifiedTime = ts.classifiedTime;
                track.identifiedTime = ts.identifiedTime;

                // ── 滤波器更新 ──
                // 有滤波器时：predict(时间差) → update(测量位置)
                // 输出平滑后的位置和速度 + 状态协方差
                if (ts.filter && ts.filter->isInitialized())
                {
                    double dt = input->simTime - ts.output.updateTime;
                    if (dt > 0.0)
                        ts.filter->predict(dt);

                    Eigen::VectorXd z(3);
                    z << obj.locationWCS[0], obj.locationWCS[1], obj.locationWCS[2];
                    ts.filter->update(z);
                    ts.filterUpdateCount++;

                    Eigen::VectorXd s = ts.filter->state();
                    track.posX = s(0);
                    track.posY = s(1);
                    track.posZ = s(2);
                    track.velX = s(3);
                    track.velY = s(4);
                    track.velZ = s(5);

                    // ── 状态协方差传播 ──
                    // 前 2 帧认为滤波器未稳定，受 includeUnstableCovariance 控制
                    bool stable = (ts.filterUpdateCount > 2);
                    if (stable || m_config.includeUnstableCovariance)
                    {
                        Eigen::MatrixXd P = ts.filter->covariance();
                        for (int r = 0; r < 6; r++)
                            for (int c = 0; c < 6; c++)
                                track.stateCovariance[r * 6 + c] = P(r, c);
                    }

                    // ── 残差协方差传播（innovationCovariance）──
                    if (stable || m_config.includeUnstableResidualCovariance)
                    {
                        Eigen::MatrixXd S = ts.filter->innovationCovariance();
                        int dim = static_cast<int>(S.rows());
                        for (int r = 0; r < dim && r < 3; r++)
                            for (int c = 0; c < dim && c < 3; c++)
                                track.residualCovariance[r * 3 + c] = S(r, c);
                    }
                }

                // 更新航迹时间戳（purgeOldTracks 依赖此字段）
                ts.output.updateTime = input->simTime;
            }
            else
            {
                // 新目标：创建 TrackState，分配新 trackId
                TrackState ts;
                ts.trackId = m_nextTrackId++;
                ts.streamNumber = input->streamNumber;
                ts.truthName  = obj.truthName ? obj.truthName : "";
                track.trackId = ts.trackId;

                // 有滤波器时，创建独立实例给该航迹
                if (m_filterPrototype)
                {
                    ts.filter.reset(createKalmanFilter());
                    // 首帧初始化：用测量值初始化状态
                    Eigen::VectorXd z(3);
                    z << obj.locationWCS[0], obj.locationWCS[1], obj.locationWCS[2];
                    ts.filter->update(z);
                    ts.filterUpdateCount = 1;
                }

                m_trackList[obj.truthIndex] = std::move(ts);
            }
        }
        else
        {
            // ---- 静态图像分支：一次性报告 ----
            // 每帧新 trackId，不持久化 TrackState
            // 对应 spot SAR 或静态照片场景
            track.trackId = m_nextTrackId++;
        }

        // ----------------------------------------------------------------
        // 步骤 3：信噪比
        // ----------------------------------------------------------------
        if (obj.signalLevel > 0.0 && input->noiseLevel > 0.0)
        {
            track.signalToNoise = obj.signalLevel / input->noiseLevel;
        }

        // ----------------------------------------------------------------
        // 步骤 4：速度报告
        // ----------------------------------------------------------------
        // 视频流：有滤波器时速度来自 Kalman 状态估计（已在步骤 2 中填充），
        //         无滤波器时不报速度。
        // 静态图像：报零速度（约定 velocityValid=true）。
        if (!isVideoStream)
        {
            track.velX = 0.0;
            track.velY = 0.0;
            track.velZ = 0.0;
        }

        // ----------------------------------------------------------------
        // 步骤 5：方位角/俯仰角报告
        // ----------------------------------------------------------------
        // 当 reportsBearingElevation=true 时，基于传感器和目标位置
        // 计算几何方位和俯仰角（WCS 坐标系的简化计算）。
        if (m_config.reportsBearingElevation)
        {
            double dx = obj.locationWCS[0] - input->sensorLocWCS[0];
            double dy = obj.locationWCS[1] - input->sensorLocWCS[1];
            double dz = obj.locationWCS[2] - input->sensorLocWCS[2];
            double range = std::sqrt(dx*dx + dy*dy + dz*dz);

            if (range > 1.0)
            {
                track.bearing   = std::atan2(dy, dx);
                track.elevation = std::asin(-dz / range);
                track.range     = range;
            }
        }

        // ----------------------------------------------------------------
        // 步骤 6：测量协方差传播
        // ----------------------------------------------------------------
        // 将传感器 RBE 误差变换为 WCS 测量协方差矩阵（3×3）。
        {
            double dx = obj.locationWCS[0] - input->sensorLocWCS[0];
            double dy = obj.locationWCS[1] - input->sensorLocWCS[1];
            double dz = obj.locationWCS[2] - input->sensorLocWCS[2];
            double rng = std::sqrt(dx*dx + dy*dy + dz*dz);
            double az  = std::atan2(dy, dx);
            double el  = std::asin(-dz / (rng > 1.0 ? rng : 1.0));
            computeMeasurementCovariance(rng, az, el, *input, track.measurementCovariance);
        }

        // ----------------------------------------------------------------
        // 步骤 7：传感器误差传播
        // ----------------------------------------------------------------
        track.bearing   = std::atan2(
            obj.locationWCS[1] - input->sensorLocWCS[1],
            obj.locationWCS[0] - input->sensorLocWCS[0]);

        // ----------------------------------------------------------------
        // 步骤 7：UpdateTrack — 分类/识别 aux data
        // ----------------------------------------------------------------
        // 写入分类/识别时间戳，维护"状态不后退"语义
        if (m_config.targetRecognitionEnabled)
        {
            updateTrackAuxData(obj.truthIndex, isVideoStream,
                               input->simTime, resultStr, track);
        }

        track.valid = 1;
        output->trackCount++;
    }

    // ----------------------------------------------------------------
    // 步骤 8：清理超时航迹（仅视频流）
    // ----------------------------------------------------------------
    if (isVideoStream)
    {
        purgeOldTracks(input->simTime, input->simTime);
    }

    // ----------------------------------------------------------------
    // 步骤 9：ImageProcessingCompleted — coast 降级
    // ----------------------------------------------------------------
    // 本帧未被更新的 RecognitionStatus（stale）执行 coast 检查，
    // 超时则降级到 UNDETECTED。
    if (m_config.targetRecognitionEnabled)
    {
        processCoastDowngrade(input->simTime);
    }

    output->frameState = 0;
    return 0;
}

// ======================================================================
// NoDetectUpdate — 无探测更新（coast 外推）
// ======================================================================
// 当外部框架在没有新图像帧的情况下调用时，对有滤波器的活跃航迹
// 执行 coast 外推：filter->predict(dt)，输出第一条 coast 航迹。
int ImageProcessor::NoDetectUpdate(double simTime,
                                    IP_TrackOutput* output)
{
    if (!m_initialized || !output)
        return -1;

    std::memset(output, 0, sizeof(*output));

    // 对第一个有滤波器的活跃航迹执行 coast 外推
    for (auto& pair : m_trackList)
    {
        TrackState& ts = pair.second;
        if (!ts.filter || !ts.filter->isInitialized())
            continue;

        double dt = simTime - ts.output.updateTime;
        if (dt > 0.0)
        {
            ts.filter->predict(dt);
            ts.output.updateTime = simTime;
        }

        Eigen::VectorXd s = ts.filter->state();
        std::memset(output, 0, sizeof(*output));
        output->targetIndex = pair.first;
        output->trackId     = ts.trackId;
        output->posX        = s(0);
        output->posY        = s(1);
        output->posZ        = s(2);
        output->velX        = s(3);
        output->velY        = s(4);
        output->velZ        = s(5);
        output->updateTime  = simTime;
        output->filterState = IP_FILTER_STATE_COASTING;
        output->valid       = 1;
        break;  // 仅输出第一条活跃 coast 航迹
    }
    return 0;
}

// ======================================================================
// Reset — 重置处理器状态
// ======================================================================
int ImageProcessor::Reset()
{
    m_trackList.clear();
    m_statusList.clear();
    m_nextTrackId = 1;
    return 0;
}

// ======================================================================
// GetFilterState — 查询滤波器状态
// ======================================================================
int ImageProcessor::GetFilterState() const
{
    if (!m_initialized)
        return IP_FILTER_UNINITIALIZED;

    // 无滤波器时，有航迹表示正在跟踪，否则未初始化
    if (m_trackList.empty())
        return IP_FILTER_UNINITIALIZED;

    return IP_FILTER_TRACKING;
}

// ======================================================================
// 配置读写
// ======================================================================
int ImageProcessor::SetConfig(const IP_Config* cfg)
{
    if (!cfg)
        return -1;
    m_config = *cfg;
    return 0;
}

int ImageProcessor::GetConfig(IP_Config* cfg) const
{
    if (!cfg)
        return -1;
    *cfg = m_config;
    return 0;
}

// ======================================================================
// createKalmanFilter — 创建配置好的 Kalman 滤波器实例
// ======================================================================
// 6 维状态：[x, y, z, vx, vy, vz]
// 3 维测量：[x, y, z]
// 匀速（CV）模型
TargetMeasurement::Filter::KalmanFilterND* ImageProcessor::createKalmanFilter() const
{
    auto kf = new TargetMeasurement::Filter::KalmanFilterND(6, 3);

    // ── 状态转移 f(x, dt) — 匀速模型 ──
    kf->setStateFunc([](const Eigen::VectorXd& x, double dt) {
        Eigen::VectorXd x2(6);
        x2(0) = x(0) + x(3) * dt;  // x' = x + vx·dt
        x2(1) = x(1) + x(4) * dt;  // y' = y + vy·dt
        x2(2) = x(2) + x(5) * dt;  // z' = z + vz·dt
        x2(3) = x(3);               // vx' = vx
        x2(4) = x(4);               // vy' = vy
        x2(5) = x(5);               // vz' = vz
        return x2;
    });

    // ── Jacobian F = ∂f/∂x ──
    kf->setStateJacobian([](const Eigen::VectorXd&, double dt) {
        Eigen::MatrixXd F(6, 6);
        F << 1, 0, 0, dt, 0,  0,
             0, 1, 0, 0,  dt, 0,
             0, 0, 1, 0,  0,  dt,
             0, 0, 0, 1,  0,  0,
             0, 0, 0, 0,  1,  0,
             0, 0, 0, 0,  0,  1;
        return F;
    });

    // ── 测量模型 h(x) = 取位置分量 ──
    kf->setMeasureFunc([](const Eigen::VectorXd& x) {
        Eigen::VectorXd z(3);
        z(0) = x(0);
        z(1) = x(1);
        z(2) = x(2);
        return z;
    });

    // ── 测量 Jacobian H = ∂h/∂x ──
    kf->setMeasureJacobian([](const Eigen::VectorXd&) {
        Eigen::MatrixXd H(3, 6);
        H << 1, 0, 0, 0, 0, 0,
             0, 1, 0, 0, 0, 0,
             0, 0, 1, 0, 0, 0;
        return H;
    });

    // ── 过程噪声协方差 Q ──
    Eigen::MatrixXd Q(6, 6);
    double qx = m_config.processNoiseSigmaX * m_config.processNoiseSigmaX;
    double qy = m_config.processNoiseSigmaY * m_config.processNoiseSigmaY;
    double qz = m_config.processNoiseSigmaZ * m_config.processNoiseSigmaZ;
    Q << qx, 0,  0,  0,  0,  0,
         0,  qy, 0,  0,  0,  0,
         0,  0,  qz, 0,  0,  0,
         0,  0,  0,  qx*10, 0,  0,
         0,  0,  0,  0,  qy*10, 0,
         0,  0,  0,  0,  0,  qz*10;
    kf->setQ(Q);

    // ── 测量噪声协方差 R ──
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3) * 100.0;
    kf->setR(R);

    return kf;
}

// ======================================================================
// initializeFilter — 滤波器工厂
// ======================================================================
bool ImageProcessor::initializeFilter()
{
    if (m_config.filterType == IP_FILTER_NONE)
    {
        m_filterPrototype.reset();
        return true;
    }

    m_filterPrototype.reset(createKalmanFilter());
    return true;
}

// ======================================================================
// computeMeasurementCovariance — 计算测量协方差矩阵
// ======================================================================
// 将传感器 RBE 误差通过 Jacobian 变换到 WCS 坐标。
//
// 输入：
//   range, az, el — 目标相对传感器的球坐标
//   input.bearingError   → σ_az（方位角误差）
//   input.elevationError → σ_el（俯仰角误差）
//   input.rangeError     → σ_r（距离误差）
//
// 公式：
//   P_wcs = J * diag(σ_r², σ_az², σ_el²) * J^T
//   其中 J = ∂(x,y,z)/∂(r,az,el)
void ImageProcessor::computeMeasurementCovariance(double range, double az, double el,
                                                   const IP_ImageInput& input,
                                                   double cov[9]) const
{
    double sr = input.rangeError;
    double sa = input.bearingError;
    double se = input.elevationError;

    // 防御：误差为 0 时输出零矩阵
    if (sr <= 0.0 && sa <= 0.0 && se <= 0.0)
    {
        std::memset(cov, 0, 9 * sizeof(double));
        return;
    }

    double ce = std::cos(el);
    double se_el = std::sin(el);
    double ca = std::cos(az);
    double sa_az = std::sin(az);

    // Jacobian J = ∂(x,y,z)/∂(r,az,el)
    double J[3][3];
    J[0][0] = ce * ca;               J[0][1] = -range * ce * sa_az;  J[0][2] = -range * se_el * ca;
    J[1][0] = ce * sa_az;            J[1][1] =  range * ce * ca;    J[1][2] = -range * se_el * sa_az;
    J[2][0] = se_el;                 J[2][1] =  0.0;                 J[2][2] =  range * ce;

    // P_wcs = J * diag(sr², sa², se²) * J^T
    double sr2 = sr * sr;
    double sa2 = sa * sa;
    double se2 = se * se;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            cov[i * 3 + j] = J[i][0] * sr2 * J[j][0]
                           + J[i][1] * sa2 * J[j][1]
                           + J[i][2] * se2 * J[j][2];
        }
    }
}

// ======================================================================
// Johnson 准则概率计算
// ======================================================================
// 三个计算方法使用同一数学公式：
//
//   P(N) = N^(2.7 + 0.7N) / (1 + N^(2.7 + 0.7N))
//
// 其中 N = objectSize / SAF。
//
// 这是经典的 Johnson 准则（1958）经验公式，描述人眼/光学系统
// 对目标的识别能力与线对数之间的 S 形关系。
// 已被 MIL-STD-1500 等军用标准采纳。
//
// 各参数含义：
//   objectSize — 目标尺寸（线对数或像素数）
//   SAF        — Scene Analysis Factor，即达到 50% 概率所需尺寸
//
// 当 N < 0（SAF 为负）时直接返回 0，但调用方已保证 SAF > 0。

double ImageProcessor::computeProbabilityOfDetection(double objectSize) const
{
    if (m_config.detectionSAF <= 0.0)
        return 0.0;
    double sizeRatio  = objectSize / m_config.detectionSAF;
    double commonTerm = std::pow(sizeRatio, (2.7 + 0.7 * sizeRatio));
    return (commonTerm / (1.0 + commonTerm));
}

double ImageProcessor::computeProbabilityOfClassification(double objectSize) const
{
    if (m_config.classificationSAF <= 0.0)
        return 0.0;
    double sizeRatio  = objectSize / m_config.classificationSAF;
    double commonTerm = std::pow(sizeRatio, (2.7 + 0.7 * sizeRatio));
    return (commonTerm / (1.0 + commonTerm));
}

double ImageProcessor::computeProbabilityOfIdentification(double objectSize) const
{
    if (m_config.identificationSAF <= 0.0)
        return 0.0;
    double sizeRatio  = objectSize / m_config.identificationSAF;
    double commonTerm = std::pow(sizeRatio, (2.7 + 0.7 * sizeRatio));
    return (commonTerm / (1.0 + commonTerm));
}

// ======================================================================
// stateToResultString — 状态→结果字符串映射
// ======================================================================
// 将 FSM 内部状态映射为 UpdateTrack 使用的帧结果字符串。
// 映射规则对应 ImageProcessor::EvaluateObjectState 末尾的 switch：
//
//   IDENTIFIED                 → "IDENTIFIED"
//   WAITING_IDENTIFICATION     → "CLASSIFIED"
//   CLASSIFIED                 → "CLASSIFIED"
//   WAITING_CLASSIFICATION     → "DETECTED"
//   DETECTED                   → "DETECTED"
//   其他                       → "UNDETECTED"
//
// 注意：WAITING_* 状态映射到较低等级的结果。
//       如 WAITING_IDENTIFICATION 返回 "CLASSIFIED"，
//       表示尚未正式确认识别，只能报告为"已分类"。
std::string ImageProcessor::stateToResultString(int state) const
{
    switch (state)
    {
    case cST_IDENTIFIED:
        return "IDENTIFIED";
    case cST_WAITING_IDENTIFICATION:
    case cST_CLASSIFIED:
        return "CLASSIFIED";
    case cST_WAITING_CLASSIFICATION:
    case cST_DETECTED:
        return "DETECTED";
    default:
        return "UNDETECTED";
    }
}

// ======================================================================
// evaluateObjectState — Johnson 准则 + 完整 7 状态 FSM
// ======================================================================
// 这是整个目标识别状态机的核心方法。对每个检测目标执行：
//
// 步骤：
//   1. 目标尺寸计算（像素数 → 线对数）
//   2. 三级 Johnson 概率计算（P_d, P_c, P_i）
//   3. 均匀随机 draw → 帧状态
//   4. 像素数硬门槛校验（最小像素数约束）
//   5. 获取/创建 RecognitionStatus（跨帧持久化）
//   6. 执行 7 状态 FSM 过渡
//   7. 保存更新后的状态
//
// FSM 过渡规则摘要：
//
//   当前状态        | 帧达标条件           | 下一状态
//   ----------------+---------------------+-----------------------
//   UNDETECTED      | frameState≥DETECTED  | WAITING_DETECTION
//   WAITING_DETECTION| frameState≥DETECTED  | (延迟满→) DETECTED
//   DETECTED        | frameState≥CLASSIFIED | WAITING_CLASSIFICATION
//   WAITING_CLASSIF | frameState≥CLASSIFIED | (延迟满→) CLASSIFIED
//   CLASSIFIED      | frameState≥IDENTIFIED | WAITING_IDENTIFICATION
//   WAITING_IDENTIF | frameState≥IDENTIFIED | (延迟满→) IDENTIFIED
//   IDENTIFIED      | (持续保持)            | IDENTIFIED
//
// 各状态有对应的 coast 超时，超时后降级到 coast 期间最佳帧状态。
void ImageProcessor::evaluateObjectState(double simTime,
                                          const IP_ImageInput& image,
                                          const IP_DetectedObject& object,
                                          int& outRecognitionState)
{
    (void)image;

    // ---- 1. 计算目标尺寸 ----
    // 使用像素数或线对数作为 Johnson 准则的输入
    double pixelCount = object.pixelCount;
    double objectSize = pixelCount;

    if (m_config.averageAspectRatio > 0.0)
    {
        // 像素数 → 线对数转换：
        //   假设目标长宽比为 AR，
        //   面积 PC = 长×宽 = (AR×SD)×SD
        //   → SD = sqrt(PC / AR)   （最短边像素数）
        //   → 线对数 = SD / 2      （一线对 = 一亮 + 一暗两条线）
        //
        // 例如：PC=200, AR=4.0
        //   SD = sqrt(200/4) = 7.07
        //   线对数 = 7.07/2 = 3.54
        objectSize = 0.5 * std::sqrt(pixelCount / m_config.averageAspectRatio);
    }
    // 若 averageAspectRatio == 0，直接用像素数作为 objectSize

    // ---- 2. 计算三级 Johnson 概率 ----
    double probDetect = computeProbabilityOfDetection(objectSize);
    double probClass  = computeProbabilityOfClassification(objectSize);
    double probIdent  = computeProbabilityOfIdentification(objectSize);

    // ---- 3. 随机 draw 确定本帧理论帧状态 ----
    // 均匀分布 [0,1) 随机数与三级概率比较：
    //   draw ≤ probIdent  → IDENTIFIED
    //   draw ≤ probClass  → CLASSIFIED
    //   draw ≤ probDetect → DETECTED
    //   否则              → UNDETECTED
    //
    // 这模拟了 ATR 系统的概率性识别：同尺寸目标在不同帧可能
    // 因噪声、视角等因素得到不同的识别结果。
    double draw = std::uniform_real_distribution<double>(0.0, 1.0)(m_rng);

    TargetRecognitionState frameState = cST_UNDETECTED;
    if ((draw <= probIdent) && (pixelCount >= m_config.minIdentPixelCount))
    {
        frameState = cST_IDENTIFIED;
    }
    else if ((draw <= probClass) && (pixelCount >= m_config.minClassPixelCount))
    {
        frameState = cST_CLASSIFIED;
    }
    else if ((draw <= probDetect) && (pixelCount >= m_config.minDetectPixelCount))
    {
        frameState = cST_DETECTED;
    }
    // 注意：帧状态同时受 Johnson 概率和像素数硬门槛约束

    // ---- 4. 获取/创建该目标的持久识别状态 ----
    size_t targetIndex = static_cast<size_t>(object.truthIndex);
    RecognitionStatus status(object.truthName ? object.truthName : "");

    auto sli = m_statusList.find(targetIndex);
    if (sli != m_statusList.end())
    {
        // 已有状态：继承跨帧的 FSM 状态
        status = sli->second;
    }
    // 新目标：使用默认初始状态（UNDETECTED）

    // ---- 5. 完整 FSM 过渡 ----
    // 使用 do-while 循环确保一次可能连续过渡多个状态
    // （例如从 WAITING_IDENTIFICATION 直接跳到 IDENTIFIED）
    TargetRecognitionState currentState;
    do
    {
        currentState = status.CurrentState();

        switch (status.CurrentState())
        {
        case cST_UNDETECTED:
            // 帧达标 → 进入等待检测
            if (frameState >= cST_DETECTED)
            {
                status.EnterState(cST_WAITING_DETECTION, simTime,
                                  m_config.detectionDelayTime);
                status.SetLastGoodUpdateTime(simTime);
            }
            else
            {
                // 帧不达标，重置 lastGoodUpdateTime
                status.SetLastGoodUpdateTime(-1.0e30);
            }
            break;

        case cST_WAITING_DETECTION:
            // 持续达标且延迟期满 → 确认检测
            if (frameState >= cST_DETECTED)
            {
                if (simTime >= status.EarliestStateExitTime())
                {
                    status.EnterState(cST_DETECTED, simTime, 0.0);
                }
                status.SetLastGoodUpdateTime(simTime);
            }
            // 持续不达标 → coast 检查
            else if (status.CoastTimeExceeded(simTime, frameState,
                                               m_config.transitionCoastTime))
            {
                status.EnterState(status.CoastingState(), simTime, 0.0);
            }
            break;

        case cST_DETECTED:
            // 帧达标
            if (frameState >= cST_DETECTED)
            {
                // 若帧状态达到 CLASSIFIED，进入分类等待
                if (frameState >= cST_CLASSIFIED)
                {
                    status.EnterState(cST_WAITING_CLASSIFICATION, simTime,
                                      m_config.classificationDelayTime);
                }
                status.SetLastGoodUpdateTime(simTime);
            }
            // 持续不达标 → coast 降级
            else if (status.CoastTimeExceeded(simTime, frameState,
                                               m_config.detectionCoastTime))
            {
                status.EnterState(status.CoastingState(), simTime, 0.0);
            }
            break;

        case cST_WAITING_CLASSIFICATION:
            // 持续达标且延迟期满 → 确认分类
            if (frameState >= cST_CLASSIFIED)
            {
                if (simTime >= status.EarliestStateExitTime())
                {
                    status.EnterState(cST_CLASSIFIED, simTime, 0.0);
                }
                status.SetLastGoodUpdateTime(simTime);
            }
            else if (status.CoastTimeExceeded(simTime, frameState,
                                               m_config.transitionCoastTime))
            {
                status.EnterState(status.CoastingState(), simTime, 0.0);
            }
            break;

        case cST_CLASSIFIED:
            // 帧达标
            if (frameState >= cST_CLASSIFIED)
            {
                // 若帧状态达到 IDENTIFIED，进入识别等待
                if (frameState >= cST_IDENTIFIED)
                {
                    status.EnterState(cST_WAITING_IDENTIFICATION, simTime,
                                      m_config.identificationDelayTime);
                }
                status.SetLastGoodUpdateTime(simTime);
            }
            else if (status.CoastTimeExceeded(simTime, frameState,
                                               m_config.classificationCoastTime))
            {
                status.EnterState(status.CoastingState(), simTime, 0.0);
            }
            break;

        case cST_WAITING_IDENTIFICATION:
            // 持续达标且延迟期满 → 确认识别
            if (frameState >= cST_IDENTIFIED)
            {
                if (simTime >= status.EarliestStateExitTime())
                {
                    status.EnterState(cST_IDENTIFIED, simTime, 0.0);
                }
                status.SetLastGoodUpdateTime(simTime);
            }
            else if (status.CoastTimeExceeded(simTime, frameState,
                                               m_config.transitionCoastTime))
            {
                status.EnterState(status.CoastingState(), simTime, 0.0);
            }
            break;

        case cST_IDENTIFIED:
            // 最高状态：达标则更新 goodUpdateTime，不达标则 coast
            if (frameState >= cST_IDENTIFIED)
            {
                status.SetLastGoodUpdateTime(simTime);
            }
            else if (status.CoastTimeExceeded(simTime, frameState,
                                               m_config.identificationCoastTime))
            {
                // 注意：降级目标状态是 coast 期间的最佳帧状态
                // （由 CoastTimeExceeded 内部维护）
                status.EnterState(status.CoastingState(), simTime, 0.0);
            }
            break;
        }
    } while (currentState != status.CurrentState());
    // 循环直到状态不再变化（一次可能连续跳过多级）

    // ---- 6. 保存更新后的状态到持久存储 ----
    m_statusList[targetIndex] = status;

    // ---- 7. 设置返回值 ----
    outRecognitionState = status.CurrentState();
}

// ======================================================================
// updateTrackAuxData — 航迹辅助数据（分类/识别时间戳）
// ======================================================================
// 对应 ImageProcessor::UpdateTrack 的 aux data 逻辑。
//
// 功能：将帧结果（IDENTIFIED/CLASSIFIED/DETECTED）写入输出航迹的
//       classifiedTime / identifiedTime 字段，并维护跨帧持久化。
//
// 关键设计——"状态不后退"：
//   一旦目标在视频流中达到某识别等级（例如 IDENTIFIED），
//   即使后续帧 Johnson 概率下降，输出航迹始终报告最高已达状态。
//   这是通过 TrackState 的持久化时间戳实现的：
//
//   帧 1：目标在图像中足够清晰 → Johnson 准则 → IDENTIFIED
//         → TrackState.identifiedTime = simTime
//   帧 5：目标变远、像素减少 → Johnson 准则 → DETECTED
//         → 但 TrackState.identifiedTime >= 0 → 仍输出 IDENTIFIED
//
// 静态图像无 TrackState 持久化，仅使用当前帧结果。
void ImageProcessor::updateTrackAuxData(int targetIndex,
                                         bool isVideoStream,
                                         double simTime,
                                         const std::string& frameResult,
                                         IP_TrackOutput& track)
{
    // 获取持久化的 aux data（仅视频流有 TrackState）
    TrackState* tsPtr = nullptr;
    if (isVideoStream)
    {
        auto it = m_trackList.find(targetIndex);
        if (it != m_trackList.end())
            tsPtr = &it->second;
    }

    // 读取持久化时间戳
    double existingClassified = -1.0;
    double existingIdentified = -1.0;
    if (tsPtr)
    {
        existingClassified = tsPtr->classifiedTime;
        existingIdentified = tsPtr->identifiedTime;
    }

    // 决策：状态不后退（见上方说明）
    if ((existingIdentified >= 0.0) || (frameResult == "IDENTIFIED"))
    {
        // == 已识别 ==
        // 无论是之前已达 IDENTIFIED 还是本帧新达，都输出当前时间
        track.identifiedTime = simTime;
        track.classifiedTime = simTime;

        if (tsPtr)
        {
            tsPtr->identifiedTime = simTime;
            tsPtr->classifiedTime = simTime;
        }
    }
    else if ((existingClassified >= 0.0) || (frameResult == "CLASSIFIED"))
    {
        // == 已分类但未识别 ==
        track.identifiedTime = -1.0;
        track.classifiedTime = simTime;

        if (tsPtr)
        {
            tsPtr->identifiedTime = -1.0;
            tsPtr->classifiedTime = simTime;
        }
    }
    else
    {
        // == 仅检测（未分类也未识别）==
        track.identifiedTime = -1.0;
        track.classifiedTime = -1.0;

        if (tsPtr)
        {
            tsPtr->identifiedTime = -1.0;
            tsPtr->classifiedTime = -1.0;
        }
    }
}

// ======================================================================
// processCoastDowngrade — 帧处理完毕后的 coast 降级
// ======================================================================
// 对应 ImageProcessor::ImageProcessingCompleted。
//
// 帧首所有 RecognitionStatus 被标记为 stale。ProcessImage 循环中
// 每处理一个目标就清除其 status 的 stale 标记（通过 evaluateObjectState
// 内部的 SetLastGoodUpdateTime）。
//
// 帧末，仍为 stale 的 status 表示"该目标在本帧图像中没有出现"。
// 对这类 status 执行 coast 检查：若连续未出现时间超过对应 coast 时间，
// 则降级到 UNDETECTED。
//
// 不同状态使用不同的 coast 时间：
//   - DETECTED   → detectionCoastTime
//   - CLASSIFIED → classificationCoastTime
//   - IDENTIFIED → identificationCoastTime
//   - 其他       → transitionCoastTime
//
// 一旦降级到 UNDETECTED，目标需要重新经过整个检测→分类→识别流程。
//
// 注意：此处的 "目标不在图像中" 指的是上游传感器未提供该目标，
//       而非"图像中有目标但 Johnson 准则判定为未检测到"。
//       后一种情况在 evaluateObjectState 中处理。
void ImageProcessor::processCoastDowngrade(double simTime)
{
    for (auto& sli : m_statusList)
    {
        RecognitionStatus& status = sli.second;
        if (status.IsStale())
        {
            // 根据当前状态选择 coast 时间
            double coastTime = m_config.transitionCoastTime;
            switch (status.CurrentState())
            {
            case cST_DETECTED:
                coastTime = m_config.detectionCoastTime;
                break;
            case cST_CLASSIFIED:
                coastTime = m_config.classificationCoastTime;
                break;
            case cST_IDENTIFIED:
                coastTime = m_config.identificationCoastTime;
                break;
            default:
                break;
            }

            if (status.CoastTimeExceeded(simTime, cST_UNDETECTED, coastTime))
            {
                // Coast 超时，降级到 UNDETECTED
                // 注意：此处传入 cST_UNDETECTED 作为 frameState，
                // 因此 coastingState 也是 UNDETECTED，降级目标就是 UNDETECTED
                TargetRecognitionState oldState = status.CurrentState();
                status.EnterState(cST_UNDETECTED, simTime, 0.0);
                (void)oldState;  // 调试时可使用
            }
        }
    }
}

// ======================================================================
// purgeOldTracks — 清理超时航迹
// ======================================================================
// 对应 ImageProcessor::PurgeOldTracks。
//
// 遍历视频流航迹表 m_trackList，检查每个 TrackState 的末次更新时间。
// 若 imageTime - 末次更新时间 > coastTime，认为航迹已过期，执行清理：
//   1. 删除对应的 RecognitionStatus（状态机状态）
//   2. 删除 TrackState（航迹本身）
//
// 注意：
//   - 仅用于视频流（静态图像不产生持久化航迹）
//   - 此处使用 ts.output.updateTime 作为末次更新依据
//   - 在 ProcessImage 中，这一步在全部目标处理完毕后执行
void ImageProcessor::purgeOldTracks(double simTime, double imageTime)
{
    (void)simTime;

    auto it = m_trackList.begin();
    while (it != m_trackList.end())
    {
        TrackState& ts    = it->second;
        double timeSinceLastUpdate = imageTime - ts.output.updateTime;

        if (timeSinceLastUpdate > m_config.coastTime)
        {
            // 删除对应的识别状态机状态
            m_statusList.erase(it->first);

            // 删除航迹
            m_trackList.erase(it++);
        }
        else
        {
            ++it;
        }
    }
}

} // namespace ImageProcessor
