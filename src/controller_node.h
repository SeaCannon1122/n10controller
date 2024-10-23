#pragma once

#ifndef CONTROLLER_NODE_H
#define CONTROLLER_NODE_H

struct controller_node_data {
    double lin_x = 0.f;
    double lin_y = 0.f;
    double ang_z = 0.f;
};

int controller_node_main(int argc, const char* const* argv);

#endif