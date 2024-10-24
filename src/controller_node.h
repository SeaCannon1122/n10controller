#pragma once

#ifndef CONTROLLER_NODE_H
#define CONTROLLER_NODE_H

struct controller_node_feedback_data {

    float lin_x = 0.f;
    float lin_y = 0.f;
    float ang_z = 0.f;

    float wheel_angle_0 = 0.f;
    float wheel_angle_1 = 0.f;
    float wheel_angle_2 = 0.f;
    float wheel_angle_3 = 0.f;
    float wheel_angle_4 = 0.f;
    float wheel_angle_5 = 0.f;

    float wheel_speed_0 = 0.f;
    float wheel_speed_1 = 0.f;
    float wheel_speed_2 = 0.f;
    float wheel_speed_3 = 0.f;
    float wheel_speed_4 = 0.f;
    float wheel_speed_5 = 0.f;
};

int controller_node_main(int argc, const char* const* argv);

#endif