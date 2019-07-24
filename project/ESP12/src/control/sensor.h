#ifndef Sensor_H
#define Sensor_H

#include <Arduino.h>
#include "MPU6050.h"

// Calibration 
#define CALIBRATION_SAMPLES         512U
#define CALIBRATION_STEP            1024

// Gyro
#define GYRO_SENSITIVITY_FS_250     131 // per degree / second
#define GYRO_SENSITIVITY            GYRO_SENSITIVITY_FS_250

// Accelerometer
#define ACCEL_SENSITIVITY_FS_8      4096      
#define ACCEL_SENSITIVITY_FS_4      8192
#define ACCEL_SENSITIVITY_FS_2      16384

#define ACCEL_SENSITIVITY           ACCEL_SENSITIVITY_FS_4
#define ACCEL_FS_RANGE              MPU6050_ACCEL_FS_4

// See '4.18 Registers 65 and 66 – Temperature Measurement' from MPU 6000 Register Map
#define TEMP_CONSTANT               340
#define TEMP_BIAS                   36.53

// Serial Debug
#ifdef IMU6D_SERIAL_DEBUG
    #include <Streaming.h>
#endif

class IMU6D {
    public:
        IMU6D() {
        }

        bool initialize();
        bool isInitialized();
        void calibrate();

        void getMotion(float_t acc[], float_t rot[]);
        float_t getTemperatureCelcius();
    private:
        // class default I2C address is 0x68
        // specific I2C addresses may be passed as a parameter here
        // AD0 low = 0x68 (default for InvenSense evaluation board)
        // AD0 high = 0x69
        MPU6050 mpu;
        //MPU6050 mpu(0x69); // <-- use for AD0 high

        // calibration helpers
        void setOffsets(int16_t offsets[]);
        void sampleAverage(int16_t means[], uint16_t samples);
        
        int16_t buffer[6];
        bool initialized = false;
};

#endif
