#pragma once
// Purpose: Declarations for cubic spline utilities used in smoothing.

#include <vector>
#include <utility>
#include <geometry_msgs/msg/pose_array.hpp>

// Natural cubic spline utilities for 2D path smoothing.

// 1D spline used to evaluate position and derivatives along a knot vector.
class CubicSpline1D{
public:
    CubicSpline1D(const std::vector<double>& x, const std::vector<double>& y);

    // Position and derivatives at parameter t (t is in the x_ knot domain).
    double calcPosition(double t) const;
    double calcFirstDerivative(double t) const;
    double calcSecondDerivative(double t) const;

private:
    int findSegment(double t) const;
    std::vector<double> x_;
    std::vector<double> a_;
    std::vector<double> b_;
    std::vector<double> c_;
    std::vector<double> d_;
};

// 2D spline built from two 1D splines parameterized by arc length.
class CubicSpline2D{
public:
    CubicSpline2D(const std::vector<double>& x, const std::vector<double>& y);

    // Returns (x, y) at arc-length s along the spline.
    std::pair<double, double> calcPosition(double s) const;
    double calcYaw(double s) const;
    double calcCurvature(double s) const;
    double getMaxArcLength() const;

private:
    std::vector<double> computeArcLength(
        const std::vector<double>& x, const std::vector<double>& y);

    std::vector<double> s_;
    CubicSpline1D sx_;
    CubicSpline1D sy_;
};

// Samples a CubicSpline2D into a PoseArray at fixed resolution.
class CubicSplineInterpolator{
public:
    geometry_msgs::msg::PoseArray interpolate(
        const geometry_msgs::msg::PoseArray& path, double resolution);
};
