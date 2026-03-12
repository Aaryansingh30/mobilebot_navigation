#pragma once
// Purpose: Trapezoidal time-parameterization for arc-length samples.

#include <vector>
#include <algorithm>
#include <cmath>

// Trapezoidal time-parameterization for arc-length samples.

// Compute timestamps for a monotonic arc-length vector `s`.
// Returns a vector t (seconds) with the same size as s.
inline std::vector<double> compute_time_stamps(
    const std::vector<double> &s,
    double v_max,
    double a_max)
{
    std::vector<double> t;
    if (s.empty()) {
        return t;
    }

    v_max = std::max(0.05, v_max);
    a_max = std::max(0.05, a_max);

    // Total distance to traverse.
    const double s_total = s.back();
    const double t_accel = v_max / a_max;
    const double d_accel = 0.5 * a_max * t_accel * t_accel;

    // If the path is too short to reach v_max, use a triangular profile.
    const bool triangular = (2.0 * d_accel >= s_total);
    double t_peak = t_accel;
    double v_peak = v_max;
    double d_peak = d_accel;

    if (triangular) {
        t_peak = std::sqrt(s_total / a_max);
        v_peak = a_max * t_peak;
        d_peak = 0.5 * a_max * t_peak * t_peak;
    }

    t.reserve(s.size());
    for (double si : s) {
        // Compute time at distance si by inverting the profile.
        double ti = 0.0;
        if (si <= d_peak) {
            ti = std::sqrt(2.0 * si / a_max);
        } else if (!triangular && si <= (s_total - d_peak)) {
            ti = t_peak + (si - d_peak) / v_peak;
        } else {
            const double s_dec = s_total - si;
            ti = (triangular ? t_peak : (t_peak + (s_total - 2.0 * d_peak) / v_peak))
                 + (t_peak - std::sqrt(2.0 * s_dec / a_max));
        }
        t.push_back(ti);
    }
    return t;
}
