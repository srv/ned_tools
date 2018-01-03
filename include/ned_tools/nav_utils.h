#ifndef NAV_UTILS_H
#define NAV_UTILS_H

#include <eigen3/Eigen/Geometry>
#include <math.h>

#include <iostream>

Eigen::Vector3d getRPY(const Eigen::Matrix3d& rotation);

Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw);

double dms2Deg(const double degree_minutes, const char hemisphere);

double dms2DegInt(const double degree_minutes, const int hemisphere);

double rad2Deg(const double radians);

double deg2Rad(const double degrees);

double normalizeAngle(const double angle);

#endif // NAV_UTILS_H
