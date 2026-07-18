#include "config_param.h"


configParam_t configParam = {
    .pidRate = {
        .roll  = {250.0f, 0.0f, 2.0f},   // Kp, Ki, Kd
        .pitch = {250.0f, 0.0f, 2.0f},
        .yaw   = {120.0f, 0.0f, 0.0f},
    },
    .pidAngle = {
        .roll  = {30.0f, 0.0f, 0.5f},
        .pitch = {10.0f, 0.0f, 0.5f},
        .yaw   = {30.0f, 0.0f, 0.5f},
    },
    .pidPos = {
        .vx = {0}, .vy = {0}, .vz = {0},
        .x  = {0}, .y  = {0}, .z  = {0},
    },
    .trimP = 0.0f,
};

