#include "TM_Interface.h"
#include <cstdio>
#include <cstring>
#ifdef _WIN32
#include <windows.h>
#endif

static void fillRadarMeas(TM_RadarMeasurement& meas, int targetId,
                          double simTime, double range, double az, double el)
{
    std::memset(&meas, 0, sizeof(meas));
    meas.input.simTime = simTime;
    meas.input.targetId = targetId;
    meas.input.sensorType = TM_SENSOR_RADAR;
    meas.input.referenceFrame = TM_FRAME_LLA;
    meas.input.position[0] = 31.23 * 3.14159265358979323846 / 180.0;
    meas.input.position[1] = 121.47 * 3.14159265358979323846 / 180.0;
    meas.input.position[2] = 100.0;
    meas.input.attitude[0] = 1; meas.input.attitude[1] = 0; meas.input.attitude[2] = 0;
    meas.input.attitude[3] = 0; meas.input.attitude[4] = 1; meas.input.attitude[5] = 0;
    meas.input.attitude[6] = 0; meas.input.attitude[7] = 0; meas.input.attitude[8] = 1;

    meas.range = range;
    meas.azimuth = az;
    meas.elevation = el;
    meas.rangeRate = 300.0;
    meas.rangeError = 50.0;
    meas.azimuthError = 0.1 * 3.14159265358979323846 / 180.0;
    meas.elevationError = 0.1 * 3.14159265358979323846 / 180.0;
    meas.rangeRateError = 2.0;
    meas.validFlags = TM_MEAS_RANGE | TM_MEAS_AZIMUTH | TM_MEAS_ELEVATION | TM_MEAS_RANGE_RATE;
}

int main()
{
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif

    TM_Config cfg;
    std::memset(&cfg, 0, sizeof(cfg));
    cfg.sensorType = TM_SENSOR_RADAR;
    cfg.filterType = TM_FILTER_KALMAN;
    cfg.track_init_m = 2;
    cfg.track_init_n = 3;
    cfg.track_maint_m = 1;
    cfg.track_maint_n = 1;
    cfg.track_coast_s = 3.0;

    if (!TM_Initialize(&cfg))
    {
        std::printf("TM_Initialize failed\n");
        return 1;
    }

    TM_RadarMeasurement meas0, meas1;
    TM_LocationOutput out0, out1;

    for (int frame = 0; frame < 5; ++frame)
    {
        double t = frame * 0.1;
        double halfPi = 3.14159265358979323846 / 180.0;

        fillRadarMeas(meas0, 0, t, 80000.0 + frame * 50,
                      20.0 * halfPi, 5.0 * halfPi);
        TM_ProcessMeasurement(&meas0, sizeof(meas0), &out0);

        fillRadarMeas(meas1, 1, t, 40000.0 + frame * 30,
                      30.0 * halfPi, 3.0 * halfPi);
        TM_ProcessMeasurement(&meas1, sizeof(meas1), &out1);

        std::printf("Frame %d: t0=(%.2f,%.2f,%.2f) t1=(%.2f,%.2f,%.2f) cnt=%d\n",
                    frame,
                    out0.position[0], out0.position[1], out0.position[2],
                    out1.position[0], out1.position[1], out1.position[2],
                    TM_GetTrackCount());
    }

    std::printf("\n--- Coast ---\n");
    std::memset(&out0, 0, sizeof(out0));
    TM_NoDetectUpdate(1.0, &out0);
    std::printf("After coast 1s: cnt=%d\n", TM_GetTrackCount());

    std::memset(&out0, 0, sizeof(out0));
    TM_NoDetectUpdate(5.0, &out0);
    std::printf("After coast 5s: cnt=%d\n", TM_GetTrackCount());

    TM_Finalize();
    return 0;
}
