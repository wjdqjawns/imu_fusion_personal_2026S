#include "global_var.h"

ImuData g_imuRaw = {};
ImuData g_imuCal = {};

// Paste bias values from MODE_CALIBRATION output here.
// All zeros = no calibration applied.
ImuBias g_bias = {
    .ax = 0.0f, .ay = 0.0f, .az = 0.0f,
    .gx = 0.0f, .gy = 0.0f, .gz = 0.0f
};
