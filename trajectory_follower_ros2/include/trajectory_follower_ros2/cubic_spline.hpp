#pragma once
// Purpose: Declarations for cubic spline utilities used in smoothing.

#include <vector>
#include <utility>
#include <geometry_msgs/msg/pose_array.hpp>

// Natural cubic spline utilities for 2D path smoothing.

class CubicSpline1D{
public:
    CubicSpline1D(const std::vector<double>& x, const std::vector<double>& y);

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

class CubicSpline2D{
public:
    CubicSpline2D(const std::vector<double>& x, const std::vector<double>& y);

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

class CubicSplineInterpolator{
public:
    geometry_msgs::msg::PoseArray interpolate(
        const geometry_msgs::msg::PoseArray& path, double resolution);
};
