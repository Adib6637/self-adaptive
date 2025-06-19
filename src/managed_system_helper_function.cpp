#include <iostream>
#include <cmath>

double quaternionToYaw360(double x, double y, double z, double w) {
    // Convert quaternion to yaw (rotation around Z-axis)
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp); // in radians

    // Convert to degrees
    double yaw_degrees = yaw * 180.0 / M_PI;

    // Normalize to 0–360 degrees
    if (yaw_degrees < 0) {
        yaw_degrees += 360.0;
    }

    return yaw_degrees; // heading relative to north
}

// Function to compute the resultant speed of the drone
double computeResultantSpeed(double vx, double vy, double vz) {
    return std::sqrt(vx * vx + vy * vy + vz * vz);
}


