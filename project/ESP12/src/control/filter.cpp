#include "filter.h"

void Complementary::filter(float_t attitude[], const float_t acc[], const float_t rate[], int16_t dt) {
    float_t acc_angles[] = {0.0, 0.0, 0.0};

    // convert acc to angles in degrees, so that acc is degrees and rate is degrees per time
    acc_angles[0] = atan2f(acc[0], sqrt(acc[1] * acc[1] + acc[2] * acc[2])) * RAD_TO_DEG;
    acc_angles[1] = atan2f(acc[1], sqrt(acc[0] * acc[0] + acc[2] * acc[2])) * RAD_TO_DEG;

    if (!isAngleSet) {
        angle_p[0] = acc_angles[0];
        angle_p[1] = acc_angles[1];
        isAngleSet = true;
    }

    float_t dt_secs = (float_t) dt / 1000;

    attitude[0] = FILTER_HPASS_CUTOFF * (angle_p[0] + rate[0] * dt_secs) + FILTER_LPASS_CUTOFF * acc_angles[0];
    attitude[1] = FILTER_HPASS_CUTOFF * (angle_p[1] + rate[1] * dt_secs) + FILTER_LPASS_CUTOFF * acc_angles[1];
    attitude[2] = angle_p[2] + rate[2] * dt_secs;

    angle_p[0] = attitude[0];
    angle_p[1] = attitude[1];
    angle_p[2] = attitude[2];
}