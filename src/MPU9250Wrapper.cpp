//
// Created by ubuntu on 4/8/24.
//

#include "MPU9250Wrapper.h"

MPU9250Wrapper::MPU9250Wrapper() {
    Wire.begin();
    Wire.setClock(40000);
    delay(2000);
    MPU9250Setting setting;
    setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
    setting.gyro_fs_sel = GYRO_FS_SEL::G2000DPS;
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    setting.gyro_fchoice = 0x03;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    setting.accel_fchoice = 0x01;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

    if (!mpu.setup(0x68, setting)) {  // change to your own address
        while (true) {
            Log::println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
}

bool MPU9250Wrapper::update() {
    return mpu.update();
}

float MPU9250Wrapper::pitch() {
    pitch_filter.Filter(mpu.getPitch());
    return pitch_filter.Current();
}

float MPU9250Wrapper::roll() {
    roll_filter.Filter(mpu.getRoll());
    return roll_filter.Current();
}

float MPU9250Wrapper::yaw() {
    yaw_filter.Filter(mpu.getYaw());
    return yaw_filter.Current();
}

float MPU9250Wrapper::gyroX() {
    gyroX_filter.Filter(mpu.getGyroX());
    return gyroX_filter.Current();
}

float MPU9250Wrapper::gyroY() {
    gyroY_filter.Filter(mpu.getGyroY() * -1);
    return gyroY_filter.Current();
}

float MPU9250Wrapper::gyroZ() {
    gyroZ_filter.Filter(mpu.getGyroZ());
    return gyroZ_filter.Current();
}

void MPU9250Wrapper::calibrateAccAndGyro() {
    mpu.calibrateAccelGyro();
}

void MPU9250Wrapper::calibrateMag() {
    mpu.calibrateMag();
}
