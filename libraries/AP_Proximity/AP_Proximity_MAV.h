#pragma once

#include "AP_Proximity.h"
#include "AP_Proximity_Backend.h"

class AP_Proximity_MAV : public AP_Proximity_Backend
{

public:
    // constructor
    using AP_Proximity_Backend::AP_Proximity_Backend;

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override { return _distance_max; }
    float distance_min() const override { return _distance_min; };

    // get distance upwards in meters. returns true on success
    bool get_upward_distance(float &distance) const override;

    // handle mavlink DISTANCE_SENSOR messages
    void handle_msg(const mavlink_message_t &msg) override;

    uint8_t get_object_count() const override;
    bool get_object_angle_and_distance(uint8_t object_number, float& angle_deg, float &distance) const override;
    bool get_closest_object(float& angle_deg, float &distance) const override;

private:

    // initialise sensor (returns true if sensor is succesfully initialised)
    bool initialise();

    // horizontal distance support
    uint32_t _last_update_ms;   // system time of last DISTANCE_SENSOR message received
    float _distance_max;        // max range of sensor in meters
    float _distance_min;        // min range of sensor in meters

    // upward distance support
    uint32_t _last_upward_update_ms;    // system time of last update distance
    float _distance_upward;             // upward distance in meters

    float _dist;
    int _angle_deg;
};
