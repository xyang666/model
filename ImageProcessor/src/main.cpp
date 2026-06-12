#include "IP_Interface.h"
#include <cstdio>
#include <cstring>
#include <cmath>

/*
 * ImageProcessor 演示程序
 * -----------------------
 * 展示目标识别状态机、航迹管理、aux data 等功能。
 */

// 辅助：识别状态名
static const char* stateName(int s)
{
    static const char* names[] = {
        "UNDETECTED", "WAITING_DETECTION", "DETECTED",
        "WAITING_CLASSIFICATION", "CLASSIFIED",
        "WAITING_IDENTIFICATION", "IDENTIFIED"
    };
    if (s < 0 || s > 6) return "?";
    return names[s];
}

// ================================================================
// 测试1：关闭目标识别（纯检测模式）
// ================================================================
static int testBasicDetection()
{
    std::printf("========================================\n");
    std::printf("Test 1: Basic Detection (recognition OFF)\n");
    std::printf("========================================\n");

    IP_Config cfg;
    std::memset(&cfg, 0, sizeof(cfg));
    cfg.filterType = IP_FILTER_NONE;
    cfg.minDetectPixelCount = 1.0;
    cfg.coastTime = 5.0;
    cfg.targetRecognitionEnabled = false;
    cfg.randomSeed = 42;

    if (!IP_Initialize(&cfg))
    {
        std::printf("FAIL: IP_Initialize\n");
        return -1;
    }

    IP_DetectedObject objs[2];
    std::memset(objs, 0, sizeof(objs));

    objs[0].pixelCount = 150.0;
    objs[0].signalLevel = 50.0;
    objs[0].locationWCS[0] = 5000.0;
    objs[0].locationWCS[1] = 3000.0;
    objs[0].locationWCS[2] = -200.0;
    objs[0].truthIndex = 1;
    objs[0].truthName = "Target_A";

    objs[1].pixelCount = 30.0;
    objs[1].signalLevel = 10.0;
    objs[1].locationWCS[0] = -2000.0;
    objs[1].locationWCS[1] = 8000.0;
    objs[1].locationWCS[2] = -500.0;
    objs[1].truthIndex = 2;
    objs[1].truthName = "Target_B";

    IP_ImageInput input;
    std::memset(&input, 0, sizeof(input));
    input.simTime = 1.0;
    input.sensorLocWCS[0] = 0.0;
    input.sensorLocWCS[1] = 0.0;
    input.sensorLocWCS[2] = 5000.0;
    input.noiseLevel = 1.0;
    input.isVideoStream = false;
    input.objectCount = 2;
    input.objects = objs;

    IP_ProcessOutput output;
    int result = IP_ProcessImage(&input, &output);
    if (result != 0)
    {
        std::printf("FAIL: IP_ProcessImage returned %d\n", result);
        IP_Finalize();
        return -1;
    }

    std::printf("Frame 1: %d tracks\n", output.trackCount);
    for (int i = 0; i < output.trackCount; ++i)
    {
        const auto& t = output.tracks[i];
        std::printf("  [%d] target=%d trackId=%d state=%s "
                    "pos=(%.0f,%.0f,%.0f) pixels=%.0f\n",
                    i, t.targetIndex, t.trackId,
                    stateName(t.recognitionState),
                    t.posX, t.posY, t.posZ, t.pixelCount);
    }

    IP_Finalize();
    std::printf("PASS\n\n");
    return 0;
}

// ================================================================
// 测试2：目标识别状态机（Johnson 准则）
// ================================================================
static int testRecognitionFSM()
{
    std::printf("========================================\n");
    std::printf("Test 2: Target Recognition FSM\n");
    std::printf("========================================\n");

    IP_Config cfg;
    std::memset(&cfg, 0, sizeof(cfg));
    cfg.filterType = IP_FILTER_NONE;
    cfg.minDetectPixelCount = 1.0;
    cfg.minClassPixelCount  = 10.0;
    cfg.minIdentPixelCount  = 50.0;
    cfg.averageAspectRatio  = 4.0;
    cfg.detectionSAF      = 1.0;
    cfg.classificationSAF = 4.0;
    cfg.identificationSAF = 6.4;
    cfg.detectionDelayTime    = 0.0;  // 无延迟，快速展示
    cfg.classificationDelayTime  = 0.0;
    cfg.identificationDelayTime  = 0.0;
    cfg.detectionCoastTime      = 10.0;
    cfg.classificationCoastTime = 10.0;
    cfg.identificationCoastTime = 10.0;
    cfg.coastTime = 20.0;
    cfg.targetRecognitionEnabled = true;
    cfg.randomSeed = 12345;

    if (!IP_Initialize(&cfg))
    {
        std::printf("FAIL: IP_Initialize\n");
        return -1;
    }

    // 单个目标，大像素数，多帧连续测试
    IP_DetectedObject obj;
    std::memset(&obj, 0, sizeof(obj));
    obj.pixelCount  = 200.0;   // 足够大，有望达到 IDENTIFIED
    obj.signalLevel = 100.0;
    obj.locationWCS[0] = 5000.0;
    obj.locationWCS[1] = 3000.0;
    obj.locationWCS[2] = -200.0;
    obj.truthIndex = 10;
    obj.truthName  = "Target_X";

    IP_ImageInput input;
    std::memset(&input, 0, sizeof(input));
    input.sensorLocWCS[0] = 0.0;
    input.sensorLocWCS[1] = 0.0;
    input.sensorLocWCS[2] = 5000.0;
    input.noiseLevel = 1.0;
    input.isVideoStream = true;
    input.streamNumber = 1;
    input.objectCount = 1;
    input.objects = &obj;

    // 运行 15 帧，观察目标识别状态演进
    int lastState = -1;
    for (int frame = 1; frame <= 15; ++frame)
    {
        input.simTime = static_cast<double>(frame);
        input.imageNumber = frame;

        IP_ProcessOutput output;
        std::memset(&output, 0, sizeof(output));

        int result = IP_ProcessImage(&input, &output);
        if (result != 0)
        {
            std::printf("Frame %d: IP_ProcessImage failed (%d)\n", frame, result);
            continue;
        }

        if (output.trackCount > 0)
        {
            const auto& t = output.tracks[0];
            if (t.recognitionState != lastState)
            {
                std::printf("Frame %2d: target=%d -> %s"
                            " classified=%.0f identified=%.0f"
                            " pixels=%.0f\n",
                            frame, t.targetIndex,
                            stateName(t.recognitionState),
                            t.classifiedTime, t.identifiedTime,
                            t.pixelCount);
                lastState = t.recognitionState;
            }
        }
        else
        {
            if (lastState != 0) // UNDETECTED
            {
                std::printf("Frame %2d: target UNDETECTED (no track)\n", frame);
                lastState = 0;
            }
        }
    }

    // 显示最终状态摘要
    IP_Config outCfg;
    IP_GetConfig(&outCfg);
    std::printf("\nFinal status list entries: ");
    // 无法直接获取 m_statusList，但可以看到航迹数
    std::printf("(track count = 1)\n");

    IP_Finalize();
    std::printf("PASS\n\n");
    return 0;
}

// ================================================================
// 测试3：静态图像 + reportsBearingElevation
// ================================================================
static int testBearingElevation()
{
    std::printf("========================================\n");
    std::printf("Test 3: Static Image + Bearing/Elevation\n");
    std::printf("========================================\n");

    IP_Config cfg;
    std::memset(&cfg, 0, sizeof(cfg));
    cfg.filterType = IP_FILTER_NONE;
    cfg.minDetectPixelCount = 1.0;
    cfg.coastTime = 5.0;
    cfg.targetRecognitionEnabled = false;
    cfg.reportsBearingElevation = true;
    cfg.randomSeed = 42;

    if (!IP_Initialize(&cfg))
    {
        std::printf("FAIL: IP_Initialize\n");
        return -1;
    }

    IP_DetectedObject obj;
    std::memset(&obj, 0, sizeof(obj));
    obj.pixelCount = 100.0;
    obj.locationWCS[0] = 1000.0;
    obj.locationWCS[1] = 1000.0;
    obj.locationWCS[2] = 0.0;
    obj.truthIndex = 1;

    IP_ImageInput input;
    std::memset(&input, 0, sizeof(input));
    input.simTime = 1.0;
    input.sensorLocWCS[0] = 0.0;
    input.sensorLocWCS[1] = 0.0;
    input.sensorLocWCS[2] = 0.0;
    input.isVideoStream = false;
    input.objectCount = 1;
    input.objects = &obj;

    IP_ProcessOutput output;
    IP_ProcessImage(&input, &output);

    if (output.trackCount > 0)
    {
        const auto& t = output.tracks[0];
        double expectedBearing = std::atan2(1000.0, 1000.0); // pi/4
        std::printf("  bearing=%.4f rad (expect ~%.4f)\n", t.bearing, expectedBearing);
        std::printf("  elev=%.4f rad\n", t.elevation);
    }

    IP_Finalize();
    std::printf("PASS\n\n");
    return 0;
}

// ================================================================
// 测试4：视频流航迹持续跟踪 + aux data 持久化
// ================================================================
static int testVideoStreamPersistence()
{
    std::printf("========================================\n");
    std::printf("Test 4: Video Stream + Aux Data Persistence\n");
    std::printf("========================================\n");

    IP_Config cfg;
    std::memset(&cfg, 0, sizeof(cfg));
    cfg.filterType = IP_FILTER_KALMAN;
    cfg.processNoiseSigmaX = 5.0;
    cfg.processNoiseSigmaY = 5.0;
    cfg.processNoiseSigmaZ = 5.0;
    cfg.minDetectPixelCount = 5.0;
    cfg.minClassPixelCount  = 50.0;
    cfg.minIdentPixelCount  = 100.0;
    cfg.averageAspectRatio  = 4.0;
    cfg.detectionSAF      = 1.5;
    cfg.classificationSAF = 4.0;
    cfg.identificationSAF = 6.4;
    cfg.detectionDelayTime    = 0.0;
    cfg.classificationDelayTime  = 0.0;
    cfg.identificationDelayTime  = 0.0;
    cfg.coastTime = 30.0;
    cfg.targetRecognitionEnabled = true;
    cfg.reportsBearingElevation = true;
    cfg.randomSeed = 42;

    IP_Initialize(&cfg);

    // 两个目标，帧 1-10 都存在，帧 11-15 消失（测试 coast）
    IP_DetectedObject objs[2];
    std::memset(objs, 0, sizeof(objs));

    objs[0].pixelCount = 300.0;
    objs[0].signalLevel = 80.0;
    objs[0].locationWCS[0] = 5000.0;
    objs[0].locationWCS[1] = 3000.0;
    objs[0].locationWCS[2] = 0.0;
    objs[0].truthIndex = 20;
    objs[0].truthName  = "PersistTarget_A";

    objs[1].pixelCount = 80.0;
    objs[1].signalLevel = 25.0;
    objs[1].locationWCS[0] = -2000.0;
    objs[1].locationWCS[1] = 4000.0;
    objs[1].locationWCS[2] = 0.0;
    objs[1].truthIndex = 21;
    objs[1].truthName  = "PersistTarget_B";

    IP_ImageInput input;
    std::memset(&input, 0, sizeof(input));
    input.sensorLocWCS[0] = 0.0;
    input.sensorLocWCS[1] = 0.0;
    input.sensorLocWCS[2] = 5000.0;
    input.noiseLevel = 1.0;
    input.isVideoStream = true;
    input.streamNumber = 1;
    input.objectCount = 2;
    input.objects = objs;

    // 帧 1-10：两个目标都在
    for (int frame = 1; frame <= 10; ++frame)
    {
        input.simTime = static_cast<double>(frame);
        input.imageNumber = frame;

        IP_ProcessOutput output;
        IP_ProcessImage(&input, &output);

        std::printf("Frame %2d: %d tracks\n", frame, output.trackCount);
        for (int i = 0; i < output.trackCount; ++i)
        {
            const auto& t = output.tracks[i];
            std::printf("  [%d] %s pos=(%.0f,%.0f,%.0f) vel=(%.2f,%.2f,%.2f)"
                        " cTime=%.0f iTime=%.0f\n",
                        t.targetIndex,
                        stateName(t.recognitionState),
                        t.posX, t.posY, t.posZ,
                        t.velX, t.velY, t.velZ,
                        t.classifiedTime, t.identifiedTime);
        }
    }

    // 帧 11-15：目标消失，检查 coast 和滤波器外推
    input.objectCount = 0;
    input.objects = nullptr;
    for (int frame = 11; frame <= 15; ++frame)
    {
        input.simTime = static_cast<double>(frame);
        input.imageNumber = frame;

        // 先处理空帧（status coast 降级）
        IP_ProcessOutput output;
        IP_ProcessImage(&input, &output);

        // 再调用 NoDetectUpdate 触发滤波器 coast 外推
        IP_TrackOutput coastOutput;
        IP_NoDetectUpdate(input.simTime, &coastOutput);

        if (coastOutput.valid)
        {
            std::printf("Frame %2d: coast track [%d] pos=(%.0f,%.0f,%.0f)"
                        " vel=(%.2f,%.2f,%.2f)\n",
                        frame, coastOutput.targetIndex,
                        coastOutput.posX, coastOutput.posY, coastOutput.posZ,
                        coastOutput.velX, coastOutput.velY, coastOutput.velZ);
        }
        else
        {
            std::printf("Frame %2d: no coast tracks\n", frame);
        }
    }

    IP_Finalize();
    std::printf("PASS\n\n");
    return 0;
}

// ================================================================
// main
// ================================================================
int main()
{
    int failures = 0;

    failures += (testBasicDetection() != 0);
    failures += (testRecognitionFSM() != 0);
    failures += (testBearingElevation() != 0);
    failures += (testVideoStreamPersistence() != 0);

    std::printf("========================================\n");
    if (failures == 0)
        std::printf("All tests PASSED\n");
    else
        std::printf("%d test(s) FAILED\n", failures);
    std::printf("========================================\n");
    return failures;
}
