#include "main.h"

#include "distance_ekf.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cmath>

namespace {

// Smart ports for the forward-, left-, and right-facing V5 Distance Sensors.
// Change these and the mounting offsets to match the robot before enabling
// odometry on the field. Offsets use EZ-Template coordinates: +x is right and
// +y is forward, measured from the robot's rotation center in inches.
constexpr std::uint8_t FRONT_DISTANCE_PORT = 8;
constexpr std::uint8_t LEFT_DISTANCE_PORT = 9;
constexpr std::uint8_t RIGHT_DISTANCE_PORT = 10;
constexpr double FRONT_SENSOR_X = 0.0;
constexpr double FRONT_SENSOR_Y = 0.0;
constexpr double LEFT_SENSOR_X = 0.0;
constexpr double LEFT_SENSOR_Y = 0.0;
constexpr double RIGHT_SENSOR_X = 0.0;
constexpr double RIGHT_SENSOR_Y = 0.0;

// Override's metal field is 140.5 in wall-to-wall. Use 140.4 for the portable
// perimeter, and set the autonomous starting pose in this wall-based frame.
constexpr double FIELD_SIZE_IN = 140.5;
constexpr double MM_PER_IN = 25.4;
constexpr double MIN_DISTANCE_IN = 1.0;
constexpr double MAX_DISTANCE_IN = 78.0;
constexpr double MIN_CONFIDENCE = 30.0;
constexpr double RANGE_VARIANCE = 9.0;  // 3 in standard deviation.
constexpr double MAX_INNOVATION_IN = 10.0;
constexpr double PI = 3.14159265358979323846;

struct State {
  double x;
  double y;
  double theta;
};

struct Sensor {
  pros::Distance *device;
  double x;
  double y;
  double bearing;
};

pros::Distance front_distance(FRONT_DISTANCE_PORT);
pros::Distance left_distance(LEFT_DISTANCE_PORT);
pros::Distance right_distance(RIGHT_DISTANCE_PORT);

const std::array<Sensor, 3> sensors{{
    {&front_distance, FRONT_SENSOR_X, FRONT_SENSOR_Y, 0.0},
    {&left_distance, LEFT_SENSOR_X, LEFT_SENSOR_Y, -90.0},
    {&right_distance, RIGHT_SENSOR_X, RIGHT_SENSOR_Y, 90.0},
}};

pros::Mutex state_mutex;
State state{0.0, 0.0, 0.0};
State last_applied_pose{0.0, 0.0, 0.0};
std::array<std::array<double, 3>, 3> covariance{};
bool initialized = false;

double radians(double degrees) {
  return degrees * PI / 180.0;
}

double wrapped_delta(double degrees) {
  while (degrees > 180.0) degrees -= 360.0;
  while (degrees <= -180.0) degrees += 360.0;
  return degrees;
}

void reset_covariance() {
  covariance = {{{9.0, 0.0, 0.0},
                 {0.0, 9.0, 0.0},
                 {0.0, 0.0, 4.0}}};
}

bool expected_wall_range(const State &pose, const Sensor &sensor, double *range) {
  const double theta = radians(pose.theta);
  const double body_x = sensor.x;
  const double body_y = sensor.y;
  const double sensor_x = pose.x + std::cos(theta) * body_x + std::sin(theta) * body_y;
  const double sensor_y = pose.y - std::sin(theta) * body_x + std::cos(theta) * body_y;
  const double beam = radians(pose.theta + sensor.bearing);
  const double ray_x = std::sin(beam);
  const double ray_y = std::cos(beam);

  // Distance sensors are only trusted when they point mostly at one wall.
  if (std::max(std::abs(ray_x), std::abs(ray_y)) < 0.85) return false;

  double nearest = MAX_DISTANCE_IN + 1.0;
  if (std::abs(ray_x) > 0.001) {
    const double wall_x = ray_x > 0.0 ? FIELD_SIZE_IN : 0.0;
    const double candidate = (wall_x - sensor_x) / ray_x;
    if (candidate > 0.0) nearest = std::min(nearest, candidate);
  }
  if (std::abs(ray_y) > 0.001) {
    const double wall_y = ray_y > 0.0 ? FIELD_SIZE_IN : 0.0;
    const double candidate = (wall_y - sensor_y) / ray_y;
    if (candidate > 0.0) nearest = std::min(nearest, candidate);
  }

  if (nearest < MIN_DISTANCE_IN || nearest > MAX_DISTANCE_IN) return false;
  *range = nearest;
  return true;
}

bool read_range(const Sensor &sensor, double *range) {
  const std::int32_t millimeters = sensor.device->get();
  if (millimeters <= 0) return false;

  *range = millimeters / MM_PER_IN;
  if (*range < MIN_DISTANCE_IN || *range > MAX_DISTANCE_IN) return false;
  if (millimeters > 200 && sensor.device->get_confidence() < MIN_CONFIDENCE) return false;
  return true;
}

void correct_with_range(const Sensor &sensor, double measurement) {
  double predicted = 0.0;
  if (!expected_wall_range(state, sensor, &predicted)) return;

  const double residual = measurement - predicted;
  if (std::abs(residual) > MAX_INNOVATION_IN) return;

  constexpr std::array<double, 3> step{{0.25, 0.25, 1.0}};
  std::array<double, 3> h{};
  for (std::size_t i = 0; i < h.size(); i++) {
    State perturbed = state;
    if (i == 0) perturbed.x += step[i];
    if (i == 1) perturbed.y += step[i];
    if (i == 2) perturbed.theta += step[i];

    double changed = 0.0;
    if (!expected_wall_range(perturbed, sensor, &changed)) return;
    h[i] = (changed - predicted) / step[i];
  }

  std::array<double, 3> hp{};
  for (std::size_t column = 0; column < hp.size(); column++) {
    for (std::size_t row = 0; row < h.size(); row++) hp[column] += h[row] * covariance[row][column];
  }

  double innovation_variance = RANGE_VARIANCE;
  for (std::size_t i = 0; i < h.size(); i++) innovation_variance += hp[i] * h[i];
  if (innovation_variance <= 0.0 || std::abs(residual) > 3.0 * std::sqrt(innovation_variance)) return;

  std::array<double, 3> gain{};
  for (std::size_t row = 0; row < gain.size(); row++) {
    for (std::size_t column = 0; column < h.size(); column++) gain[row] += covariance[row][column] * h[column];
    gain[row] /= innovation_variance;
  }

  state.x += gain[0] * residual;
  state.y += gain[1] * residual;
  state.theta += gain[2] * residual;

  for (std::size_t row = 0; row < covariance.size(); row++) {
    for (std::size_t column = 0; column < covariance[row].size(); column++) {
      covariance[row][column] -= gain[row] * hp[column];
    }
  }
}

void distance_ekf_update() {
  if (!chassis.odom_enabled()) return;

  const ez::pose odom = chassis.odom_pose_get();
  state_mutex.take();

  const double dx = odom.x - last_applied_pose.x;
  const double dy = odom.y - last_applied_pose.y;
  const double dtheta = wrapped_delta(odom.theta - last_applied_pose.theta);
  if (!initialized || std::abs(dx) > 24.0 || std::abs(dy) > 24.0 || std::abs(dtheta) > 45.0) {
    state = {odom.x, odom.y, odom.theta};
    last_applied_pose = state;
    reset_covariance();
    initialized = true;
    state_mutex.give();
    return;
  }

  // EZ-Template supplies the prediction from its drivetrain encoders and IMU.
  state.x += dx;
  state.y += dy;
  state.theta += dtheta;
  covariance[0][0] += 0.10;
  covariance[1][1] += 0.10;
  covariance[2][2] += 0.02;

  for (const Sensor &sensor : sensors) {
    double measurement = 0.0;
    if (read_range(sensor, &measurement)) correct_with_range(sensor, measurement);
  }

  chassis.odom_xyt_set(state.x, state.y, state.theta);
  last_applied_pose = state;
  state_mutex.give();
}

void distance_ekf_task() {
  while (true) {
    distance_ekf_update();
    pros::delay(20);
  }
}

}  // namespace

void distance_ekf_start() {
  static pros::Task task(distance_ekf_task, "Distance EKF");
}

void distance_ekf_reset(double x, double y, double theta) {
  state_mutex.take();
  state = {x, y, theta};
  last_applied_pose = state;
  reset_covariance();
  initialized = true;
  state_mutex.give();
}
