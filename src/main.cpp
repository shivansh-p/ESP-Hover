#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <drone_controller/AttitudeArray.h>
#include <drone_controller/PID_Gains.h>
#include <drone_controller/FlightRecord.h>
#include <drone_controller/LightFlightRecord.h>
#include "../include/model.h"
#include "../include/PID.h"
#include "../include/FlightRecorder.h"
#include "../include/Log.h"
#include "../include/XQuadThrustManager.h"
#include "../include/MPU9250Wrapper.h"
#include "IMUManager.cpp"
#include <memory>
#include <sstream>

/*
 *  x shape motors diagram
 * Drone Head
 * B    C
 * D    A
 */
#define X_Shape

#define MOTOR_HIGH_BOUNDARY 255.0
#define MOTOR_LOW_BOUNDARY (-255.0)

#define LED_PIN 2

#ifdef X_Shape
#define MOTOR_A_PIN 32
#define MOTOR_A_CHANNEL 2

#define MOTOR_B_PIN 4
#define MOTOR_B_CHANNEL 0

#define MOTOR_C_PIN 18
#define MOTOR_C_CHANNEL 3

#define MOTOR_D_PIN 19
#define MOTOR_D_CHANNEL 1

XQuadThrustManager thrustManager(MOTOR_HIGH_BOUNDARY, MOTOR_LOW_BOUNDARY, 80);

#else

#define MOTOR_A_PIN 32
#define MOTOR_A_CHANNEL 2

#define MOTOR_B_PIN 18
#define MOTOR_B_CHANNEL 0

#define MOTOR_C_PIN 19
#define MOTOR_C_CHANNEL 3

#define MOTOR_D_PIN 4
#define MOTOR_D_CHANNEL 1

PlusQuadThrustManager thrustManager(MOTOR_HIGH_BOUNDARY, MOTOR_LOW_BOUNDARY, 80);

#endif

SemaphoreHandle_t xMutex{xSemaphoreCreateMutex()};
QueueHandle_t attitudeQueueHandle = xQueueCreate(50, sizeof(AttitudeModel));
QueueHandle_t recorderQueueHandle = xQueueCreate(50, sizeof(drone_controller::LightFlightRecord));
QueueHandle_t gainsQueueHandle = xQueueCreate(50, sizeof(GainModel));
TickType_t sendWaitTime = pdMS_TO_TICKS(5);
TickType_t receiveWaitTime = pdMS_TO_TICKS(1);


//std::unique_ptr<MPU9250Wrapper> mpu9250 = nullptr;
IMUManager mpu9250;

PID pitchPID(MOTOR_HIGH_BOUNDARY, MOTOR_LOW_BOUNDARY);
PID rollPID(MOTOR_HIGH_BOUNDARY, MOTOR_LOW_BOUNDARY);
PID yawPID(MOTOR_HIGH_BOUNDARY, MOTOR_LOW_BOUNDARY);

GainModel gain_model{};
AttitudeModel attitude_model{};

unsigned long previous_millis{0};
volatile bool isArmed{false};
float current_attitude[5] = {0, 0, 0, 0, 0}; // pitch, roll, yaw_rate, pitch_rate, roll_rate
float offsets[5] = {0, 0, 0, 0, 0};

long long msg_pub_count = 0;

const char *ssid = "WE_E80297";
const char *password = "m8m33955";
IPAddress server(192, 168, 93, 82);
const uint16_t serverPort = 11411;

std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>();

void attitude_callback(const drone_controller::AttitudeArray &msg) {
    isArmed = msg.data[0] == 1;
    Log::println(std::to_string(isArmed));

        AttitudeModel model = AttitudeModel{
                .thrust = msg.data[1],
                .pitchSetPoint = msg.data[2],
                .rollSetPoint = msg.data[3],
                .yawSetPoint = msg.data[4]
        };

        if (xQueueSend(attitudeQueueHandle, &model, sendWaitTime) == pdPASS) {
            Log::println("Enqueue new attitude success");
        } else {
            Log::println("Enqueue new attitude failed");
        }

//        Log::println("thrust = " + std::to_string(model.thrust));
//        Log::println("pitch = " + std::to_string(model.pitchSetPoint));
//        Log::println("roll = " + std::to_string(model.rollSetPoint));
//        Log::println("yaw = " + std::to_string(model.yawSetPoint));
}

void gains_callback(const drone_controller::PID_Gains &msg) {
    gain_model = GainModel{
            .pitch_kp = msg.p_kp,
            .pitch_ki = msg.p_ki,
            .pitch_kd = msg.p_kd,

            .roll_kp = msg.r_kp,
            .roll_ki = msg.r_ki,
            .roll_kd = msg.r_kd,

            .yaw_kp = msg.y_kp,
            .yaw_ki = msg.y_ki,
            .yaw_kd = msg.y_kd
    };

    if (xQueueSend(gainsQueueHandle, &gain_model, sendWaitTime) == pdPASS) {
        Log::println("Enqueue new gains success");
    } else {
        Log::println("Enqueue new gains failed");
    }
}

//drone_controller::FlightRecord record;
drone_controller::LightFlightRecord record;

ros::Subscriber<drone_controller::AttitudeArray> attitude_sub("/cmd_attitude", attitude_callback);
ros::Subscriber<drone_controller::PID_Gains> pid_sub("/pid_gains", gains_callback);
ros::Publisher flight_record_pub("/recorder", &record);

FlightRecorder flightRecorder([](std::vector<drone_controller::FlightRecord> &&buffer) {
    for (auto &rec: buffer) {
        //record = std::move(rec);
        //xQueueSend(recorderQueueHandle, &rec, sendWaitTime);
    }
});

void measureOffsets() {
    int counter{0};
    double sumPitch{0};
    double sumRoll{0};
    double sumYaw{0};
    double sumPitchRate{0};
    double sumRollRate{0};

    while (counter <= 1049) {
        if (mpu9250.imu_loop()) {
            if (counter >= 999) {
                sumPitch += mpu9250.getPitch();
                sumRoll += mpu9250.getRoll();
                sumYaw += mpu9250.getGyroZ();
                sumPitchRate += mpu9250.getGyroY();
                sumRollRate += mpu9250.getGyroX();
            }
            counter++;
            delay(5);
        }
    }

    offsets[0] = static_cast<float >(sumPitch / 50.0);
    offsets[1] = static_cast<float>(sumRoll / 50.0);
    offsets[2] = static_cast<float>(sumYaw / 50.0);
    offsets[3] = static_cast<float>(sumPitchRate / 50.0);
    offsets[4] = static_cast<float>(sumRollRate / 50.0);
}

void setupPins() {
    pinMode(LED_PIN, OUTPUT);
    pinMode(MOTOR_A_PIN, OUTPUT);
    pinMode(MOTOR_B_PIN, OUTPUT);
    pinMode(MOTOR_C_PIN, OUTPUT);
    pinMode(MOTOR_D_PIN, OUTPUT);

    ledcSetup(MOTOR_A_CHANNEL, 500, 8); // channel 0, 500 hz PWM, 8-bit resolution
    ledcSetup(MOTOR_B_CHANNEL, 500, 8); // channel 1, 500 hz PWM, 8-bit resolution
    ledcSetup(MOTOR_C_CHANNEL, 500, 8); // channel 2, 500 hz PWM, 8-bit resolution
    ledcSetup(MOTOR_D_CHANNEL, 500, 8); // channel 3, 500 hz PWM, 8-bit resolution

    ledcAttachPin(MOTOR_A_PIN, MOTOR_A_CHANNEL);
    ledcAttachPin(MOTOR_B_PIN, MOTOR_B_CHANNEL);
    ledcAttachPin(MOTOR_C_PIN, MOTOR_C_CHANNEL);
    ledcAttachPin(MOTOR_D_PIN, MOTOR_D_CHANNEL);
}

void calculateAttitude() {
    mpu9250.imu_loop();
    current_attitude[0] = mpu9250.getPitch() - offsets[0];
    current_attitude[1] = mpu9250.getRoll() - offsets[1];
    current_attitude[2] = mpu9250.getGyroZ() - offsets[2];
    current_attitude[3] = mpu9250.getGyroY() - offsets[3];
    current_attitude[4] = mpu9250.getGyroX() - offsets[4];

//    current_attitude[0] = mpu9250->pitch() - offsets[0];
//    current_attitude[1] = mpu9250->roll() - offsets[1];
//    current_attitude[2] = mpu9250->gyroZ()- offsets[2];
//    current_attitude[3] = mpu9250->gyroY() - offsets[3];
//    current_attitude[4] = mpu9250->gyroX() - offsets[4];

//    Log::print(std::to_string(current_attitude[0]) + ",");
//    Log::print(std::to_string(current_attitude[1]) + ",");
//    Log::print(std::to_string(current_attitude[2]) + ",");
//    Log::print(std::to_string(current_attitude[3]) + ",");
//    Log::print(std::to_string(current_attitude[4]) + ",");
}

void stopAllMotors() {
    ledcWrite(MOTOR_D_CHANNEL, 0);
    ledcWrite(MOTOR_C_CHANNEL, 0);
    ledcWrite(MOTOR_B_CHANNEL, 0);
    ledcWrite(MOTOR_A_CHANNEL, 0);
}

void blink(unsigned long interval, unsigned long total_duration) {
    unsigned long startTime = millis();
    bool ledState = false;

    while (millis() - startTime < total_duration) {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState ? HIGH : LOW); // Set LED state
        delay(interval);
    }

    digitalWrite(LED_PIN, LOW);
}

// Task function for ROS logic (runs on core 0)
[[noreturn]] void rosTask(void *pvParameters) {
    while (true) {
        drone_controller::LightFlightRecord rec;
        if (xQueueReceive(recorderQueueHandle, &rec, receiveWaitTime) == pdPASS) {
            flight_record_pub.publish(&rec);
        }
        nh->spinOnce();
        const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
        vTaskDelay(xDelay);
    }
}

void setup() {
    nh.reset(new ros::NodeHandle);
    Log::setup(115200, nh);

    setupPins();
    stopAllMotors();

    // Wifi connection
    Log::print("Connecting to ");
    Log::print(ssid);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Log::print(".");
    }

    Log::println("");
    Log::println("WiFi connected");
    Log::println("IP address: ");
    Log::println(WiFi.localIP());

    // Initialize ROS node handle
    nh->getHardware()->setConnection(server, serverPort);
    nh->initNode();
    nh->subscribe(attitude_sub);
    nh->subscribe(pid_sub);
    nh->advertise(flight_record_pub);

    xTaskCreatePinnedToCore(
            rosTask,          // Task function
            "rosTask",        // Task name
            10000,            // Stack size
            nullptr,             // Task parameters
            1,                // Priority
            nullptr,             // Task handle
            0                 // Core number (0 for core 0)
    );

    // MPU setup
   // mpu9250.reset(new MPU9250Wrapper());

    // Acc&Gyro Calibration
//    Log::println("Start Acc and Gyro Calibration");
//    blink(500, 4000);
//    mpu9250->calibrateAccAndGyro();
//    blink(200, 2000);
//    Log::println("Acc and Gyro Calibration done");
//
//    delay(3000);
//
//    // Mag Calibration
//    Log::println("Start Mag Calibration");
//    blink(500, 4000);
//    mpu9250->calibrateMag();
//    blink(200, 4000);
//    Log::println("Mag Calibration done");
//
//    delay(3000);

    // Setup IMU
    Log::println("MPU Setup");
    blink(500, 4000);
    mpu9250.imuSetup();
    blink(200, 4000);
    Log::println("MPU Setup done");

    delay(3000);

    // Offsets Calc
    Log::println("Start Offset Calculation");
    blink(500, 4000);
    measureOffsets();
    blink(200, 4000);
    Log::println("Offset Calculation done");

    std::string msg = "Offsets: Pitch = " + std::to_string(offsets[0]) + ", Roll = " + std::to_string(offsets[1]) +
                      ", Yaw = " + std::to_string(offsets[1]);
    Log::println(msg);
    Log::println("All set!");
}

void emergencyStop() {
    stopAllMotors();
    thrustManager.resetMotorsFilters();
}

void resetSetpointsAndBaseThrust() {
    pitchPID.SetPoint(0);
    rollPID.SetPoint(0);
    yawPID.SetPoint(0);
    thrustManager.updateBaseThrust(0);
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        isArmed = false;
        GPIO.out_w1tc = (1 << LED_PIN);
        emergencyStop();
        WiFi.disconnect();
        WiFi.begin(ssid, password);
        delay(500);
    } else {
        GPIO.out_w1ts = (1 << LED_PIN);
    }

    unsigned long now = millis();

    calculateAttitude();

    const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
    vTaskDelay(xDelay);

    if (isArmed) {
        float sampleTime = static_cast<float>(now - previous_millis) / (float) 1000;

        AttitudeModel attitudeModel{};
        if (xQueueReceive(attitudeQueueHandle, &attitudeModel, receiveWaitTime) == pdPASS) {
           // Log::println("************************Received an attitude model************************");
            pitchPID.SetPoint(attitudeModel.pitchSetPoint);
            rollPID.SetPoint(attitudeModel.rollSetPoint);
            yawPID.SetPoint(attitudeModel.yawSetPoint);
            thrustManager.updateBaseThrust(attitudeModel.thrust);

//            Log::println("a-thrust = " + std::to_string(attitudeModel.thrust));
//            Log::println("a-pitch = " + std::to_string(attitudeModel.pitchSetPoint));
//            Log::println("a-roll = " + std::to_string(attitudeModel.rollSetPoint));
//            Log::println("a-yaw = " + std::to_string(attitudeModel.yawSetPoint));
        } else {
            //Log::println("Couldn't receive the attitude model in time");
        }

        PIDModel pitchPIDModel = pitchPID.Compute(sampleTime, current_attitude[0], current_attitude[3]);
        PIDModel rollPIDModel = rollPID.Compute(sampleTime, current_attitude[1], current_attitude[4]);
        PIDModel yawPIDModel = yawPID.Compute(sampleTime, current_attitude[2], std::numeric_limits<double>::max());

        ThrustModel thrustModel = thrustManager.calculateMotorsThrust(pitchPIDModel.system_constrained_total,
                                                                      rollPIDModel.system_constrained_total,
                                                                      yawPIDModel.system_constrained_total);

        ledcWrite(MOTOR_D_CHANNEL, thrustModel.motor_d_thrust);
        ledcWrite(MOTOR_C_CHANNEL, thrustModel.motor_c_thrust);
        ledcWrite(MOTOR_B_CHANNEL, thrustModel.motor_b_thrust);
        ledcWrite(MOTOR_A_CHANNEL, thrustModel.motor_a_thrust);


//        Log::print(std::to_string(sampleTime) + ",");
//        Log::print(std::to_string(pitchPIDModel.proportional) + ",");
//        Log::print(std::to_string(pitchPIDModel.integral) + ",");
//        Log::print(std::to_string(pitchPIDModel.derivative) + ",");
//        Log::print(std::to_string(pitchPIDModel.system_constrained_total) + ",");
//
//        Log::print(std::to_string(rollPIDModel.proportional) + ",");
//        Log::print(std::to_string(rollPIDModel.integral) + ",");
//        Log::print(std::to_string(rollPIDModel.derivative) + ",");
//        Log::print(std::to_string(rollPIDModel.system_constrained_total) + ",");
//
//        Log::print(std::to_string(yawPIDModel.proportional) + ",");
//        Log::print(std::to_string(yawPIDModel.integral) + ",");
//        Log::print(std::to_string(yawPIDModel.system_constrained_total) + ",");
//
//        Log::print(std::to_string(thrustModel.motor_a_thrust) + ",");
//        Log::print(std::to_string(thrustModel.motor_b_thrust) + ",");
//        Log::print(std::to_string(thrustModel.motor_c_thrust) + ",");
//        Log::println(std::to_string(thrustModel.motor_d_thrust));

        if (msg_pub_count % 400 == 0) {
            record = drone_controller::LightFlightRecord();
            record.millis = static_cast<int32_t>(now);

            record.pitch = static_cast<int8_t>(current_attitude[0]);
            record.roll = static_cast<int8_t>(current_attitude[1]);
            record.yaw_rate = static_cast<int8_t>(current_attitude[3]);

//            record.pitch_p = static_cast<float>(pitchPIDModel.proportional);
//            record.pitch_i = static_cast<float>(pitchPIDModel.integral);
//            record.pitch_d = static_cast<float>(pitchPIDModel.derivative);
//
//            record.roll_p = static_cast<float>(rollPIDModel.proportional);
//            record.roll_i = static_cast<float>(rollPIDModel.integral);
//            record.roll_d = static_cast<float>(rollPIDModel.derivative);
//
//            record.yaw_p = static_cast<int16_t>(yawPIDModel.proportional);
//            record.yaw_i = static_cast<int16_t>(yawPIDModel.integral);
//            record.yaw_d = static_cast<int16_t>(yawPIDModel.derivative);
//
//            record.motor_a_thrust = (int16_t) thrustModel.motor_a_thrust;
//            record.motor_b_thrust = (int16_t) thrustModel.motor_b_thrust;
//            record.motor_c_thrust = (int16_t) thrustModel.motor_c_thrust;
//            record.motor_d_thrust = (int16_t) thrustModel.motor_d_thrust;

            record.pitch_pid = static_cast<int8_t>(pitchPIDModel.system_constrained_total);
            record.roll_pid = static_cast<int8_t>(rollPIDModel.system_constrained_total);
            record.yaw_pid = static_cast<int8_t>(yawPIDModel.system_constrained_total);

            //flightRecorder.insertAndFlushIfReady(record);
           // flight_record_pub.publish(&record);
            xQueueSend(recorderQueueHandle, &record, sendWaitTime);
        }

        previous_millis = now;
    } else {
        resetSetpointsAndBaseThrust();
        emergencyStop();

        GainModel gainModel{};
        if (xQueueReceive(gainsQueueHandle, &gainModel, receiveWaitTime) == pdPASS) {
            Log::println("************************Received new gain model************************");
            pitchPID.Gains(gainModel.pitch_kp, gainModel.pitch_ki, gainModel.pitch_kd);
            rollPID.Gains(gainModel.roll_kp, gainModel.roll_ki, gainModel.roll_kd);
            yawPID.Gains(gainModel.yaw_kp, gainModel.yaw_ki, gainModel.yaw_kd);
            thrustManager.resetMotorsFilters();
        }

        delay(100);

        previous_millis = now;
    }
}
