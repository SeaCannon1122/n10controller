#pragma once

#ifndef CONTROLLER_NODE_H
#define CONTROLLER_NODE_H

struct controller_node_feedback_data {

    double lin_factor = 0.2;
    double ang_factor = 0.5;

    double lin_x = 0.;
    double lin_y = 0.;
    double ang_z = 0.;

    double wheel_angle_0 = 0.;
    double wheel_angle_1 = 0.;
    double wheel_angle_2 = 0.;
    double wheel_angle_3 = 0.;
    double wheel_angle_4 = 0.;
    double wheel_angle_5 = 0.;

    double wheel_speed_0 = 0.;
    double wheel_speed_1 = 0.;
    double wheel_speed_2 = 0.;
    double wheel_speed_3 = 0.;
    double wheel_speed_4 = 0.;
    double wheel_speed_5 = 0.;

    double arm_x = 0.04;
    double arm_y = 0.09;
    double ground_angle = -1.5;
    double gripper = 0.;

    double arm_angle_0 = 0.;
    double arm_angle_1 = 0.;
    double arm_angle_2 = 0.;

    int arm_mode = 0;
};

int controller_node_main(int argc, const char* const* argv);

#endif