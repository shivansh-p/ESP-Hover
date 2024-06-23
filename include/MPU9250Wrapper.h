//
// Created by ubuntu on 4/8/24.
//

#ifndef ESP_HOVER_MPU9250WRAPPER_H
#define ESP_HOVER_MPU9250WRAPPER_H

#include "MPU9250.h"
#include "Log.h"
#include "Filter.h"


class MPU9250Wrapper {
private:
    MPU9250 mpu;
    ExponentialFilter<float> pitch_filter = ExponentialFilter<float>(80, 0);
    ExponentialFilter<float> roll_filter = ExponentialFilter<float>(80, 0);
    ExponentialFilter<float> yaw_filter = ExponentialFilter<float>(80, 0);
    ExponentialFilter<float> gyroZ_filter = ExponentialFilter<float>(80, 0);
    ExponentialFilter<float> gyroX_filter = ExponentialFilter<float>(80, 0);
    ExponentialFilter<float> gyroY_filter = ExponentialFilter<float>(80, 0);

public:
    MPU9250Wrapper();

    bool update();

    float pitch();

    float roll();

    float yaw();

    float gyroX();

    float gyroY();

    float gyroZ();

    void calibrateAccAndGyro();

    void calibrateMag();
};


#endif //ESP_HOVER_MPU9250WRAPPER_H
