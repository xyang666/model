#ifndef __UTILS_H__
#define __UTILS_H__

/*
 * TEME <-> J2000 (GCRF) position/velocity transformation
 *
 * SGP4 outputs are in TEME (True Equator, Mean Equinox).
 * This header converts to/from J2000 (GCRF) using:
 *   - IAU 2006/2000A precession-nutation via ERFA eraPnm06a()
 *   - Equation of equinoxes via ERFA eraEe06a()
 *
 * Transform chain:
 *   TEME --[Rz(Ee)]--> TOD --[PN^T]--> J2000
 *
 * Dependencies: Eigen3, ERFA
 */

#include <Eigen/Dense>
#include <erfa.h>
#include <cmath>

namespace CoordTransform
{

    /**
     * Build the orthogonal rotation matrix  M  such that
     *   r_J2000 = M * r_TEME
     *   v_J2000 = M * v_TEME
     *
     * @param jd_tt  Julian Date in Terrestrial Time (TT)
     */
    inline Eigen::Matrix3d teme2j2000_matrix(double jd_tt)
    {
        // Two-part JD for ERFA (split at J2000.0 for numerical precision)
        const double tt1 = 2451545.0;
        const double tt2 = jd_tt - 2451545.0;

        // eraPnm06a: combined precession-nutation matrix (IAU 2006/2000A)
        // Transforms J2000 -> True-of-Date:  r_TOD = rmatpn * r_J2000
        double rmatpn[3][3];
        eraPnm06a(tt1, tt2, rmatpn);

        const Eigen::Matrix3d PN =
            Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(&rmatpn[0][0]);

        // eraEe06a: equation of equinoxes (IAU 2006/2000A)
        // Accounts for the difference between mean and apparent equinox.
        // Rz(Ee) maps TEME -> TOD
        const double ee = eraEe06a(tt1, tt2);

        // R_z(-Ee): TEME (mean equinox) -> TOD (true equinox)
        const Eigen::Matrix3d Rz =
            Eigen::AngleAxisd(-ee, Eigen::Vector3d::UnitZ()).toRotationMatrix();

        // TEME -> TOD -> J2000:
        //   r_TOD   = Rz * r_TEME
        //   r_J2000 = PN^T * r_TOD  (PN is orthogonal, so inverse = transpose)
        return PN.transpose() * Rz;
    }

    /**
     * Convert position and velocity from TEME to J2000 (GCRF).
     *
     * TEME is a quasi-inertial frame, so velocity transforms identically
     * to position (no omega x r term).
     *
     * @param jd_tt    Julian Date in Terrestrial Time
     * @param r_teme   Position in TEME (km)
     * @param v_teme   Velocity in TEME (km/s)
     * @param r_j2000  Output position in J2000 (km)
     * @param v_j2000  Output velocity in J2000 (km/s)
     */
    inline void teme2j2000(double jd_tt,
                           const Eigen::Vector3d &r_teme,
                           const Eigen::Vector3d &v_teme,
                           Eigen::Vector3d &r_j2000,
                           Eigen::Vector3d &v_j2000)
    {
        const Eigen::Matrix3d M = teme2j2000_matrix(jd_tt);

        // dM/dt via central differences: δt = 0.5 s
        // Mdot [1/s],  Mdot * r [km] -> [km/s] correction
        const double dt_jd = 0.5 / 86400.0;
        const Eigen::Matrix3d Mdot =
            (teme2j2000_matrix(jd_tt + dt_jd) - teme2j2000_matrix(jd_tt - dt_jd)) / (2.0 * 0.5); // divide by 2*dt_s = 1.0 s

        r_j2000 = M * r_teme;
        v_j2000 = M * v_teme + Mdot * r_teme; // includes frame-drag term Ṁ·r
    }

    /**
     * Convert position and velocity from J2000 (GCRF) to TEME.
     *
     * @param jd_tt    Julian Date in Terrestrial Time
     * @param r_j2000  Position in J2000 (km)
     * @param v_j2000  Velocity in J2000 (km/s)
     * @param r_teme   Output position in TEME (km)
     * @param v_teme   Output velocity in TEME (km/s)
     */
    inline void j20002teme(double jd_tt,
                           const Eigen::Vector3d &r_j2000,
                           const Eigen::Vector3d &v_j2000,
                           Eigen::Vector3d &r_teme,
                           Eigen::Vector3d &v_teme)
    {
        const Eigen::Matrix3d M = teme2j2000_matrix(jd_tt);
        const Eigen::Matrix3d MT = M.transpose(); // M orthogonal => M^-1 = M^T

        const double dt_jd = 0.5 / 86400.0;
        const Eigen::Matrix3d Mdot =
            (teme2j2000_matrix(jd_tt + dt_jd) - teme2j2000_matrix(jd_tt - dt_jd)) / (2.0 * 0.5);

        // Inverse of  v_j2000 = M·v_teme + Mdot·r_teme
        //   v_teme = M^T·(v_j2000 - Mdot·r_teme)
        r_teme = MT * r_j2000;
        v_teme = MT * (v_j2000 - Mdot * r_teme);
    }

} // namespace CoordTransform

#endif // __UTILS_H__