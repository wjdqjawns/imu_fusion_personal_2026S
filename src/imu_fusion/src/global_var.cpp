#include "global_var.h"

sMsgPacket g_msg_packet = {};
sImuData   g_imuRaw     = {};

sImuBias g_bias = {
    .ax = 0.0f, .ay = 0.0f, .az = 0.0f,
    .gx = 0.0f, .gy = 0.0f, .gz = 0.0f,
    .mx = 0.0f, .my = 0.0f, .mz = 0.0f
};