// Purpose: Cubic spline implementation for path smoothing.
#include "trajectory_follower_ros2/cubic_spline.hpp"

#include <cmath>
#include <algorithm>

// 1D natural cubic spline implementation.
CubicSpline1D::CubicSpline1D(
    const std::vector<double>& x,
    const std::vector<double>& y)
{
    x_ = x;
    int n = x.size();

    a_ = y;

    b_.resize(n);
    c_.resize(n);
    d_.resize(n);

    // Segment widths between knot points.
    std::vector<double> h(n - 1);

    for (int i = 0; i < n - 1; i++)
        h[i] = x[i + 1] - x[i];

    // RHS of the tridiagonal system.
    std::vector<double> alpha(n);

    for (int i = 1; i < n - 1; i++)
    {
        alpha[i] =
            (3.0 / h[i]) * (a_[i + 1] - a_[i]) -
            (3.0 / h[i - 1]) * (a_[i] - a_[i - 1]);
    }

    // Solve tridiagonal system for second-derivative coefficients.
    std::vector<double> l(n);
    std::vector<double> mu(n);
    std::vector<double> z(n);

    l[0] = 1.0;
    mu[0] = 0.0;
    z[0] = 0.0;

    for (int i = 1; i < n - 1; i++)
    {
        l[i] =
            2 * (x[i + 1] - x[i - 1]) -
            h[i - 1] * mu[i - 1];

        mu[i] = h[i] / l[i];

        z[i] =
            (alpha[i] - h[i - 1] * z[i - 1]) /
            l[i];
    }

    l[n - 1] = 1.0;
    z[n - 1] = 0.0;

    c_[n - 1] = 0.0;

    // Back-substitution to compute spline coefficients.
    for (int j = n - 2; j >= 0; j--)
    {
        c_[j] =
            z[j] - mu[j] * c_[j + 1];

        b_[j] =
            (a_[j + 1] - a_[j]) / h[j]
            - h[j] * (c_[j + 1] + 2 * c_[j]) / 3.0;

        d_[j] =
            (c_[j + 1] - c_[j]) /
            (3.0 * h[j]);
    }
}

int CubicSpline1D::findSegment(double t) const
{
    // Locate the spline segment for parameter t.
    auto it = std::upper_bound(
        x_.begin(), x_.end(), t);

    int idx =
        std::max(int(it - x_.begin()) - 1, 0);

    if (idx >= (int)x_.size() - 1)
        idx = x_.size() - 2;

    return idx;
}

double CubicSpline1D::calcPosition(double t) const
{
    int i = findSegment(t);

    double dx = t - x_[i];

    return
        a_[i]
        + b_[i] * dx
        + c_[i] * dx * dx
        + d_[i] * dx * dx * dx;
}

double CubicSpline1D::calcFirstDerivative(double t) const
{
    int i = findSegment(t);

    double dx = t - x_[i];

    return
        b_[i]
        + 2 * c_[i] * dx
        + 3 * d_[i] * dx * dx;
}

double CubicSpline1D::calcSecondDerivative(double t) const
{
    int i = findSegment(t);

    double dx = t - x_[i];

    return
        2 * c_[i]
        + 6 * d_[i] * dx;
}

// 2D spline constructed from independent x(s) and y(s) splines.

CubicSpline2D::CubicSpline2D(
    const std::vector<double>& x,
    const std::vector<double>& y)
    : s_(computeArcLength(x, y)),
      sx_(s_, x),
      sy_(s_, y)
{
}

std::vector<double>
CubicSpline2D::computeArcLength(
    const std::vector<double>& x,
    const std::vector<double>& y)
{
    std::vector<double> s;

    s.push_back(0.0);

    // Accumulate arc length along the piecewise-linear path.
    for (size_t i = 1; i < x.size(); i++)
    {
        double dx = x[i] - x[i - 1];
        double dy = y[i] - y[i - 1];

        double dist =
            std::sqrt(dx * dx + dy * dy);

        s.push_back(s.back() + dist);
    }

    return s;
}

std::pair<double, double>
CubicSpline2D::calcPosition(double s) const
{
    double x = sx_.calcPosition(s);
    double y = sy_.calcPosition(s);

    return {x, y};
}

double CubicSpline2D::calcYaw(double s) const
{
    double dx = sx_.calcFirstDerivative(s);
    double dy = sy_.calcFirstDerivative(s);

    return std::atan2(dy, dx);
}

double CubicSpline2D::calcCurvature(double s) const
{
    double dx = sx_.calcFirstDerivative(s);
    double ddx = sx_.calcSecondDerivative(s);

    double dy = sy_.calcFirstDerivative(s);
    double ddy = sy_.calcSecondDerivative(s);

    double numerator =
        dx * ddy - dy * ddx;

    double denominator =
        std::pow(dx * dx + dy * dy, 1.5);

    return numerator / denominator;
}

double CubicSpline2D::getMaxArcLength() const
{
    return s_.back();
}

// Interpolator: sample the spline at fixed arc-length intervals.

geometry_msgs::msg::PoseArray
CubicSplineInterpolator::interpolate(
    const geometry_msgs::msg::PoseArray& path,
    double resolution)
{
    geometry_msgs::msg::PoseArray output;

    if (path.poses.size() < 2)
        return path;

    std::vector<double> x;
    std::vector<double> y;

    for (const auto& pose : path.poses)
    {
        x.push_back(pose.position.x);
        y.push_back(pose.position.y);
    }

    CubicSpline2D spline(x, y);

    double s_max =
        spline.getMaxArcLength();

    // Sample the spline at fixed arc-length increments.
    for (double s = 0.0; s <= s_max; s += resolution)
    {
        auto pos = spline.calcPosition(s);

        geometry_msgs::msg::Pose pose;

        pose.position.x = pos.first;
        pose.position.y = pos.second;

        double yaw = spline.calcYaw(s);

        pose.orientation.z =
            std::sin(yaw * 0.5);

        pose.orientation.w =
            std::cos(yaw * 0.5);

        output.poses.push_back(pose);
    }

    return output;
}
