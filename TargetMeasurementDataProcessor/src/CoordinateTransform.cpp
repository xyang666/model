#include "utils/CoordinateTransform.h"
#include <cstring>
#include <cmath>

namespace TargetMeasurement
{
    namespace Coord
    {
        static constexpr double kWgs84A = 6378137.0;
        static constexpr double kWgs84F = 1.0 / 298.257223563;
        static constexpr double kWgs84E2 = kWgs84F * (2.0 - kWgs84F);

        static M33 ecefToNedMatrix(const double origin[3])
        {
            double lat, lon, alt;
            ecefToLla(origin, lat, lon, alt);
            double slat = std::sin(lat), clat = std::cos(lat);
            double slon = std::sin(lon), clon = std::cos(lon);
            M33 R;
            R << -slat * clon, -slat * slon, clat,
                -slon, clon, 0.0,
                -clat * clon, -clat * slon, -slat;
            return R;
        }

        // ======================================================================
        // LLA ↔ ECEF
        // ======================================================================

        void llaToEcef(double lat, double lon, double alt, double ecef[3])
        {
            double sinLat = std::sin(lat), cosLat = std::cos(lat);
            double sinLon = std::sin(lon), cosLon = std::cos(lon);
            double N = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sinLat * sinLat);
            ecef[0] = (N + alt) * cosLat * cosLon;
            ecef[1] = (N + alt) * cosLat * sinLon;
            ecef[2] = ((1.0 - kWgs84E2) * N + alt) * sinLat;
        }

        void ecefToLla(const double ecef[3], double &lat, double &lon, double &alt)
        {
            double x = ecef[0], y = ecef[1], z = ecef[2];
            lon = std::atan2(y, x);
            double r = std::sqrt(x * x + y * y);
            double latCurr = std::atan2(z, r);
            for (int i = 0; i < 10; ++i)
            {
                double sinLat = std::sin(latCurr);
                double N = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sinLat * sinLat);
                double latPrev = latCurr;
                latCurr = std::atan2(z + kWgs84E2 * N * sinLat, r);
                if (std::abs(latCurr - latPrev) < 1e-12)
                    break;
            }
            lat = latCurr;
            double sinLat = std::sin(lat);
            double N = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sinLat * sinLat);
            alt = r / std::cos(lat) - N;
        }

        // ======================================================================
        // ECEF ↔ NED
        // ======================================================================

        void ecefToNed(const double ecef[3], const double origin[3], double ned[3])
        {
            M33 R = ecefToNedMatrix(origin);
            V3 delta(ecef[0] - origin[0], ecef[1] - origin[1], ecef[2] - origin[2]);
            V3 out = R * delta;
            ned[0] = out(0);
            ned[1] = out(1);
            ned[2] = out(2);
        }

        void nedToEcef(const double ned[3], const double origin[3], double ecef[3])
        {
            M33 R = ecefToNedMatrix(origin);
            V3 out = R.transpose() * V3(ned[0], ned[1], ned[2]);
            ecef[0] = out(0) + origin[0];
            ecef[1] = out(1) + origin[1];
            ecef[2] = out(2) + origin[2];
        }

        // ======================================================================
        // RBE
        // ======================================================================

        void rbeToBody(double range, double azimuth, double elevation, double body[3])
        {
            double ce = std::cos(elevation), se = std::sin(elevation);
            double ca = std::cos(azimuth), sa = std::sin(azimuth);
            body[0] = range * ce * ca;
            body[1] = range * ce * sa;
            body[2] = -range * se;
        }

        void rbeJacobian(double range, double azimuth, double elevation, double J[3][3])
        {
            double ce = std::cos(elevation), se = std::sin(elevation);
            double ca = std::cos(azimuth), sa = std::sin(azimuth);
            J[0][0] = ce * ca;
            J[0][1] = -range * ce * sa;
            J[0][2] = -range * se * ca;
            J[1][0] = ce * sa;
            J[1][1] = range * ce * ca;
            J[1][2] = -range * se * sa;
            J[2][0] = -se;
            J[2][1] = 0.0;
            J[2][2] = -range * ce;
        }

        // ======================================================================
        // CoordTransform
        // ======================================================================
        CoordTransform::CoordTransform(int srcType, int dstType, const double *t, bool tIsSrcInDst)
        {
            if (srcType == TM_FRAME_NED && dstType == TM_FRAME_ECEF)
                R_ = ecefToNedMatrix(t).transpose();
            else if (srcType == TM_FRAME_ECEF && dstType == TM_FRAME_NED)
                R_ = ecefToNedMatrix(t);
            else
                R_.setIdentity();

            // 设置 t_
            if (tIsSrcInDst)
                t_ = V3(t[0], t[1], t[2]);
            else
                t_ = -R_ * V3(t[0], t[1], t[2]);
        }

        void CoordTransform::applyPos(const double src[3], double dst[3]) const
        {
            V3 out = R_ * V3(src[0], src[1], src[2]) + t_;
            dst[0] = out(0);
            dst[1] = out(1);
            dst[2] = out(2);
        }

        void CoordTransform::applyVel(const double src[3], double dst[3]) const
        {
            V3 out = R_ * V3(src[0], src[1], src[2]);
            dst[0] = out(0);
            dst[1] = out(1);
            dst[2] = out(2);
        }
    } // namespace Coord
} // namespace TargetMeasurement
