#pragma once

struct ImuData
{
    // raw sensor accelerometer [g]
    float ax;
    float ay; 
    float az;

    // raw sensor gyroscope [deg/s]
    float gx;
    float gy;
    float gz;
    
    // raw sensor magnetometer [μT]
    float mx;
    float my;
    float mz;
};

struct ImuBias
{
    // biases for accelerometer [g]
    float ax_bias = 0.0f;
    float ay_bias = 0.0f;
    float az_bias = 0.0f;

    // biases for gyroscope [deg/s]
    float gx_bias = 0.0f;
    float gy_bias = 0.0f;
    float gz_bias = 0.0f;

    // biases for magnetometer [μT]
    float mx_bias = 0.0f;
    float my_bias = 0.0f;
    float mz_bias = 0.0f;
};

struct EulerAngle
{
    float phi   = 0.0f;
    float theta = 0.0f;
    float psi   = 0.0f;

    EulerAngle() = default;
    EulerAngle(float phi, float theta, float psi) : phi(phi), theta(theta), psi(psi) {}
};

struct Quaternion
{
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Quaternion() = default;
    Quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}
};