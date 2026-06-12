#pragma once

#include "TM_Interface.h"
#include <Eigen/Dense>

namespace TargetMeasurement
{
    namespace Coord
    {
        using M33 = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>;
        using V3 = Eigen::Matrix<double, 3, 1>;
        // ======================================================================
        // WGS84 大地坐标
        // ======================================================================

        void llaToEcef(double lat, double lon, double alt, double ecef[3]);
        void ecefToLla(const double ecef[3], double &lat, double &lon, double &alt);

        // ======================================================================
        // ECEF ↔ NED（origin 为 NED 原点在 ECEF 中的坐标）
        // ======================================================================
        void ecefToNed(const double ecef[3], const double origin[3], double ned[3]);
        void nedToEcef(const double ned[3], const double origin[3], double ecef[3]);

        // ======================================================================
        // RBE 球坐标 → 本体系笛卡尔
        // ======================================================================

        void rbeToBody(double range, double azimuth, double elevation, double body[3]);
        void rbeJacobian(double range, double azimuth, double elevation, double J[3][3]);

        // ======================================================================
        // CoordTransform — 笛卡尔参考系间 SE3 变换
        // ======================================================================
        // 支持 TM_FRAME_ECEF / NED / ENU 等笛卡尔坐标系，不支持 LLA（大地坐标）。
        // LLA → ECEF / ECEF → LLA 请使用 llaToEcef / ecefToLla。
        //
        //   CoordTransform xf(TM_FRAME_NED, TM_FRAME_ECEF, origin);
        //   xf.applyPos(p_ned, p_ecef);   // p_ecef = R * p_ned + t
        //   xf.applyVel(v_ned, v_ecef);   // v_ecef = R * v_ned
        // ======================================================================

        class CoordTransform
        {
        public:
            CoordTransform(int srcType, int dstType, const double *t, bool tIsSrcInDst = true);

            void applyPos(const double src[3], double dst[3]) const;
            void applyVel(const double src[3], double dst[3]) const;

            const M33& getR() const { return R_; }
            const V3& getT() const { return t_; }

        private:
            M33 R_;
            V3 t_;
        };

    } // namespace Coord
} // namespace TargetMeasurement
