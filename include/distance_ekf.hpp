#pragma once

// Starts the distance-sensor correction task after chassis.initialize().
void distance_ekf_start();

// Call after setting EZ-Template's initial odometry pose.
void distance_ekf_reset(double x, double y, double theta);
