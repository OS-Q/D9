#ifndef Filter_H
#define Filter_H

#include <Arduino.h>

#define FILTER_HPASS_CUTOFF       0.98
#define FILTER_LPASS_CUTOFF       (1.0 - FILTER_HPASS_CUTOFF)

// Serial Debug
#define FILTER_SERIAL_DEBUG

#ifdef FILTER_SERIAL_DEBUG
    #include <Streaming.h>
#endif

class Complementary {
    public:
        Complementary() {
            angle_p[0] = 0.0;
            angle_p[1] = 0.0;
            angle_p[2] = 0.0;
        }
        void filter(float_t attitude[], const float_t acc[], const float_t rate[], int16_t dt);
    private:
        float_t angle_p[3];
        bool isAngleSet;
};

#endif
