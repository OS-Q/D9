#include "sensor.h"

bool IMU6D::initialize() {
    mpu.initialize();

    mpu.setFullScaleAccelRange(ACCEL_FS_RANGE);

    initialized = mpu.testConnection();
    return initialized;
}

bool IMU6D::isInitialized() {
    return initialized;
}

void IMU6D::setOffsets(int16_t offsets[]) {
    mpu.setXAccelOffset(offsets[0]);
    mpu.setYAccelOffset(offsets[1]);
    mpu.setZAccelOffset(offsets[2]);

    mpu.setXGyroOffset(offsets[3]);
    mpu.setYGyroOffset(offsets[4]);
    mpu.setZGyroOffset(offsets[5]);
}

void IMU6D::calibrate() {
    #ifdef IMU6D_SERIAL_DEBUG
    Serial << "ao:" << mpu.getXAccelOffset() << ", " << mpu.getYAccelOffset() << ", " << mpu.getZAccelOffset() << endl;
    Serial << "go:" << mpu.getXGyroOffset() << ", " << mpu.getYGyroOffset() << ", " << mpu.getZGyroOffset() << endl;  
    #endif

    int16_t offsets[] = {0, 0, 0, 0, 0, 0};
    int16_t targets[] = {0, 0, ACCEL_SENSITIVITY, 0, 0, 0};
    
    setOffsets(offsets);

    int16_t meansFirst[] = {0, 0, 0, 0, 0, 0};
    sampleAverage(meansFirst, CALIBRATION_SAMPLES);

    // increase offsets by a specific value
    for (uint8_t s = 0; s < 6; s++) {
        offsets[s] += CALIBRATION_STEP;
    }
    setOffsets(offsets);

    int16_t meansSecond[] = {0, 0, 0, 0, 0, 0};
    sampleAverage(meansSecond, CALIBRATION_SAMPLES);

    for (uint8_t s = 0; s < 6; s++) {
        offsets[s] -= (meansSecond[s] - targets[s]) * CALIBRATION_STEP / (meansSecond[s] - meansFirst[s]);
    }

    setOffsets(offsets);

    #ifdef IMU6D_SERIAL_DEBUG
    Serial << "ao:" << mpu.getXAccelOffset() << ", " << mpu.getYAccelOffset() << ", " << mpu.getZAccelOffset() << endl;
    Serial << "go:" << mpu.getXGyroOffset() << ", " << mpu.getYGyroOffset() << ", " << mpu.getZGyroOffset() << endl;  
    #endif
}

void IMU6D::getMotion(float_t rot[], float_t rot_d[]) {
    mpu.getMotion6(&buffer[0], &buffer[1], &buffer[2], &buffer[3], &buffer[4], &buffer[5]);

    rot[0] = (float_t) buffer[0] / ACCEL_SENSITIVITY;
    rot[1] = (float_t) buffer[1] / ACCEL_SENSITIVITY;
    rot[2] = (float_t) buffer[2] / ACCEL_SENSITIVITY;

    rot_d[0] = (float_t) buffer[3] / GYRO_SENSITIVITY;
    rot_d[1] = (float_t) buffer[4] / GYRO_SENSITIVITY;
    rot_d[2] = (float_t) buffer[5] / GYRO_SENSITIVITY;
}

float_t IMU6D::getTemperatureCelcius() {
    return (float) mpu.getTemperature() / TEMP_CONSTANT + TEMP_BIAS;
}

void IMU6D::sampleAverage(int16_t means[], uint16_t samples) {
    long totals[] = {0L, 0L, 0L, 0L, 0L, 0L}; 
    for (uint16_t n = 0; n < samples; n++) {
        mpu.getMotion6(&buffer[0], &buffer[1], &buffer[2], &buffer[3], &buffer[4], &buffer[5]);

        for (uint8_t s = 0; s < 6; s++) {
            totals[s] += buffer[s]; 
        }
        delay(5);
    }

    for (uint8_t s = 0; s < 6; s++) {
        means[s] = totals[s] / samples; 
    }
}
