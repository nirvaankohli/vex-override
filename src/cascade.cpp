#include "main.h"
#include "cascade.hpp"

pros::Distance claw_distance_sensor(7);

enum class GoalHeight { // fake values change later bc i am lazy
    ALLIANCE = 30,
    NEUTRAL = 60,
    CENTER = 90, 
};

std::unordered_map<std::string, int> GoalHeightMap = {
    {"ALLIANCE", static_cast<int>(GoalHeight::ALLIANCE)},
    {"NEUTRAL", static_cast<int>(GoalHeight::NEUTRAL)},
    {"CENTER", static_cast<int>(GoalHeight::CENTER)},
};

const int perConeIncrement = 10; // fake value change later bc i am lazy

float claw_distance_sensor_height_calculation() {
    float raw_distance = claw_distance_sensor.get_distance();  // Get the raw distance from the sensor
    float base_height = 10.0;  // Base height of the sensor from the ground in cm
    float height = base_height + raw_distance; 

    return height;  // Return the calculated height in cm
}

float get_lift_height() {
    float claw_distance_sensor_estimated_height = claw_distance_sensor_height_calculation();

    estimated_height = claw_distance_sensor_estimated_height 
    return estimated_height;
}