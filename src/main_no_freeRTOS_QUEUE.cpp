//#include <Arduino.h>
//#include <ros.h>
//#include <std_msgs/String.h>
//#include <drone_controller/AttitudeArray.h>
//#include <drone_controller/PID_Gains.h>
//#include <drone_controller/FlightRecord.h>
//#include "../include/model.h"
//#include "../include/PID.h"
//#include "../include/FlightRecorder.h"
//#include "../include/Log.h"
//#include "../include/XQuadThrustManager.h"
//#include "../include/MPU9250Wrapper.h"
//#include <memory>
//
///*
// *  x shape motors diagram
// * Drone Head
// * B    C
// * D    A
// */
//#define X_Shape
//
//#define MOTOR_HIGH_BOUNDARY 70.0
//#define MOTOR_LOW_BOUNDARY (-70.0)
//
//#define LED_PIN 2
//
//#ifdef X_Shape
//#define MOTOR_A_PIN 32
//#define MOTOR_A_CHANNEL 2
//
//#define MOTOR_B_PIN 4
//#define MOTOR_B_CHANNEL 0
//
//#define MOTOR_C_PIN 18
//#define MOTOR_C_CHANNEL 3
//
//#define MOTOR_D_PIN 19
//#define MOTOR_D_CHANNEL 1
//
//XQuadThrustManager thrustManager(MOTOR_HIGH_BOUNDARY, MOTOR_LOW_BOUNDARY, 80);
//
//#else
//
//#define MOTOR_A_PIN 32
//#define MOTOR_A_CHANNEL 2
//
//#define MOTOR_B_PIN 18
//#define MOTOR_B_CHANNEL 0
//
//#define MOTOR_C_PIN 19
//#define MOTOR_C_CHANNEL 3
//
//#define MOTOR_D_PIN 4
//#define MOTOR_D_CHANNEL 1
//
//PlusQuadThrustManager thrustManager(MOTOR_HIGH_BOUNDARY, MOTOR_LOW_BOUNDARY, 80);
//
//#endif
//
//std::unique_ptr<MPU9250Wrapper> mpu9250 = nullptr;
//
//PID pitchPID(MOTOR_HIGH_BOUNDARY, MOTOR_LOW_BOUNDARY);
//PID rollPID(MOTOR_HIGH_BOUNDARY, MOTOR_LOW_BOUNDARY);
//PID yawPID(MOTOR_HIGH_BOUNDARY, MOTOR_LOW_BOUNDARY);
//
//unsigned long previous_millis{0};
//bool isArmed{false};
//float current_attitude[5] = {0, 0, 0, 0, 0}; // pitch, roll, yaw_rate, pitch_rate, roll_rate
//float offsets[5] = {0, 0, 0, 0, 0};
//AttitudeModel attitudeModel;
//
//const char *ssid = "WE_E80297";
//const char *password = "m8m33955";
//IPAddress server(192, 168, 1, 5);
//const uint16_t serverPort = 11411;
//
//std::shared_ptr<ros::NodeHandle> nh = std::make_shared<ros::NodeHandle>();
//
//void blink(unsigned long interval, unsigned long total_duration) {
//    unsigned long startTime = millis();
//    bool ledState = false;
//
//    while (millis() - startTime < total_duration) {
//        ledState = !ledState;
//        digitalWrite(LED_PIN, ledState ? HIGH : LOW); // Set LED state
//        delay(interval);
//    }
//
//    digitalWrite(LED_PIN, LOW);
//}
//
//
//void attitude_callback(const drone_controller::AttitudeArray &msg) {
//    isArmed = msg.data[0] == 1;
//    Serial.print("data 0 = ");
//    Serial.println(msg.data[0]);
//    Log::println(std::to_string(isArmed));
//    Serial.println(isArmed);
//    if (isArmed) {
//        Serial.println("in isArmed");
//        attitudeModel = AttitudeModel{
//                .thrust = msg.data[1],
//                .pitchSetPoint = msg.data[2],
//                .rollSetPoint = msg.data[3],
//                .yawSetPoint = msg.data[4]
//        };
//
//        Log::println("thrust = " + std::to_string(attitudeModel.thrust));
//        Log::println("pitch = " + std::to_string(attitudeModel.pitchSetPoint));
//        Log::println("roll = " + std::to_string(attitudeModel.rollSetPoint));
//        Log::println("yaw = " + std::to_string(attitudeModel.yawSetPoint));
//
//        Serial.println("before blink");
//        blink(200, 4000);
//    }
//}
//
//void gains_callback(const drone_controller::PID_Gains &msg) {
//    Log::println("Received new gain model");
//    GainModel gainModel = GainModel{
//            .pitch_kp = msg.kp,
//            .pitch_ki = msg.ki,
//            .pitch_kd = msg.kd,
//
//            .roll_kp = msg.kp,
//            .roll_ki = msg.ki,
//            .roll_kd = msg.kd,
//
//            .yaw_kp = msg.y_kp,
//            .yaw_ki = msg.y_ki,
//            .yaw_kd = msg.y_kd
//    };
//
//    pitchPID.Gains(gainModel.pitch_kp, gainModel.pitch_ki, gainModel.pitch_kd);
//    rollPID.Gains(gainModel.roll_kp, gainModel.roll_ki, gainModel.roll_kd);
//    yawPID.Gains(gainModel.yaw_kp, gainModel.yaw_ki, gainModel.yaw_kd);
//    thrustManager.resetMotorsFilters();
//}
//
//drone_controller::FlightRecord record;
//
//ros::Subscriber<drone_controller::AttitudeArray> attitude_sub("/cmd_attitude", attitude_callback);
//ros::Subscriber<drone_controller::PID_Gains> pid_sub("/pid_gains", gains_callback);
//ros::Publisher flight_record_pub("/recorder", &record);
//
//FlightRecorder flightRecorder([](std::vector<drone_controller::FlightRecord> &&buffer) {
//    for (auto &rec: buffer) {
//        flight_record_pub.publish(&rec);
//    }
//});
//
//long long count = 0;
//
//
//void measureOffsets() {
//    int counter{0};
//    double sumPitch{0};
//    double sumRoll{0};
//    double sumYaw{0};
//    double sumPitchRate{0};
//    double sumRollRate{0};
//
//    while (counter <= 1049) {
//        if (mpu9250->update()) {
//            if (counter >= 999) {
//                sumPitch += mpu9250->pitch();
//                sumRoll += mpu9250->roll();
//                sumYaw += mpu9250->gyroZ();
//                sumPitchRate += mpu9250->gyroY();
//                sumRollRate += mpu9250->gyroX();
//            }
//            counter++;
//            delay(5);
//        }
//    }
//
//    offsets[0] = static_cast<float >(sumPitch / 50.0);
//    offsets[1] = static_cast<float>(sumRoll / 50.0);
//    offsets[2] = static_cast<float>(sumYaw / 50.0);
//    offsets[3] = static_cast<float>(sumPitchRate / 50.0);
//    offsets[4] = static_cast<float>(sumRollRate / 50.0);
//}
//
//void setupPins() {
//    pinMode(LED_PIN, OUTPUT);
//    pinMode(MOTOR_A_PIN, OUTPUT);
//    pinMode(MOTOR_B_PIN, OUTPUT);
//    pinMode(MOTOR_C_PIN, OUTPUT);
//    pinMode(MOTOR_D_PIN, OUTPUT);
//
//    ledcSetup(MOTOR_A_CHANNEL, 500, 8); // channel 0, 500 hz PWM, 8-bit resolution
//    ledcSetup(MOTOR_B_CHANNEL, 500, 8); // channel 1, 500 hz PWM, 8-bit resolution
//    ledcSetup(MOTOR_C_CHANNEL, 500, 8); // channel 2, 500 hz PWM, 8-bit resolution
//    ledcSetup(MOTOR_D_CHANNEL, 500, 8); // channel 3, 500 hz PWM, 8-bit resolution
//
//    ledcAttachPin(MOTOR_A_PIN, MOTOR_A_CHANNEL);
//    ledcAttachPin(MOTOR_B_PIN, MOTOR_B_CHANNEL);
//    ledcAttachPin(MOTOR_C_PIN, MOTOR_C_CHANNEL);
//    ledcAttachPin(MOTOR_D_PIN, MOTOR_D_CHANNEL);
//}
//
//void calculateAttitude() {
//    mpu9250->update();
//    current_attitude[0] = mpu9250->pitch() - offsets[0];
//    current_attitude[1] = mpu9250->roll() - offsets[1];
//    current_attitude[2] = mpu9250->gyroZ() - offsets[2];
//    current_attitude[3] = mpu9250->gyroY() - offsets[3];
//    current_attitude[4] = mpu9250->gyroX() - offsets[4];
//
////    Log::print(std::to_string(current_attitude[0]) + ",");
////    Log::print(std::to_string(current_attitude[1]) + ",");
////    Log::print(std::to_string(current_attitude[2]) + ",");
////    Log::print(std::to_string(current_attitude[3]) + ",");
////    Log::print(std::to_string(current_attitude[4]) + ",");
//}
//
//void stopAllMotors() {
//    ledcWrite(MOTOR_D_CHANNEL, 0);
//    ledcWrite(MOTOR_C_CHANNEL, 0);
//    ledcWrite(MOTOR_B_CHANNEL, 0);
//    ledcWrite(MOTOR_A_CHANNEL, 0);
//}
//
//// Task function for ROS logic (runs on core 0)
//[[noreturn]] void rosTask(void *pvParameters) {
//    while (true) {
//        drone_controller::FlightRecord rec;
////        if (xQueueReceive(recorderQueueHandle, &rec, receiveWaitTime) == pdPASS) {
////            flight_record_pub.publish(&rec);
////        }
//        nh->spinOnce();
//        const TickType_t xDelay = 1 / portTICK_PERIOD_MS;
//        vTaskDelay(xDelay);
//
//    }
//}
//
//void setup() {
//    Serial.begin(115200);
//    nh.reset(new ros::NodeHandle);
//    Log::setup(115200, nh);
//
//    setupPins();
//
//    // Wifi connection
//    Log::print("Connecting to ");
//    Log::print(ssid);
//
//    WiFi.begin(ssid, password);
//    while (WiFi.status() != WL_CONNECTED) {
//        delay(500);
//        Log::print(".");
//    }
//
//    Log::println("");
//    Log::println("WiFi connected");
//    Log::println("IP address: ");
//    Log::println(WiFi.localIP());
//
//    // Initialize ROS node handle
//    nh->getHardware()->setConnection(server, serverPort);
//    nh->initNode();
//    nh->subscribe(attitude_sub);
//    nh->subscribe(pid_sub);
//    nh->advertise(flight_record_pub);
//
//    xTaskCreatePinnedToCore(
//            rosTask,          // Task function
//            "rosTask",        // Task name
//            10000,            // Stack size
//            nullptr,             // Task parameters
//            1,                // Priority
//            nullptr,             // Task handle
//            0                 // Core number (0 for core 0)
//    );
//
//    // MPU setup
//    //mpu9250.reset(new MPU9250Wrapper());
//
//    // Acc&Gyro Calibration
//    blink(500, 4000);
//    //mpu9250->calibrateAccAndGyro();
//    Log::println("mpu ...1");
//    blink(200, 2000);
//
//    delay(3000);
//
//    // Mag Calibration
//    blink(500, 4000);
//    //mpu9250->calibrateMag();
//    Log::println("mpu ...2");
//    blink(200, 4000);
//
//    delay(3000);
//
//    // Offsets Calc
//    blink(500, 4000);
//    //measureOffsets();
//    Log::println("mpu ...3");
//    blink(200, 4000);
//
//    Log::println("Offset calc done!");
//    std::string msg = "Offsets: Pitch = " + std::to_string(offsets[0]) + ", Roll = " + std::to_string(offsets[1]) +
//                      ", Yaw = " + std::to_string(offsets[1]);
//    Log::println(msg);
//    Log::println("All set!");
//}
//
//void emergencyStop() {
//    stopAllMotors();
//    thrustManager.resetMotorsFilters();
//}
//
//void loop() {
//    if (WiFi.status() != WL_CONNECTED) {
//        GPIO.out_w1tc = (1 << LED_PIN);
//        emergencyStop();
//        isArmed = false;
//        WiFi.disconnect();
//        WiFi.begin(ssid, password);
//        delay(500);
//    } else {
//      //  GPIO.out_w1ts = (1 << LED_PIN);
//    }
//
//    unsigned long now = millis();
//
//    if (isArmed) {
//        float sampleTime = static_cast<float>(now - previous_millis) / (float) 1000;
//
//        pitchPID.SetPoint(attitudeModel.pitchSetPoint);
//        rollPID.SetPoint(attitudeModel.rollSetPoint);
//        yawPID.SetPoint(attitudeModel.yawSetPoint);
//        thrustManager.updateBaseThrust(attitudeModel.thrust);
//
//       // calculateAttitude();
//
//        PIDModel pitchPIDModel = pitchPID.Compute(sampleTime, current_attitude[0], current_attitude[3]);
//        PIDModel rollPIDModel = rollPID.Compute(sampleTime, current_attitude[1], current_attitude[4]);
//        PIDModel yawPIDModel = yawPID.Compute(sampleTime, current_attitude[2], 0);
//
//        ThrustModel thrustModel = thrustManager.calculateMotorsThrust(pitchPIDModel.system_constrained_total,
//                                                                      rollPIDModel.system_constrained_total,
//                                                                      yawPIDModel.system_constrained_total);
//
////        ledcWrite(MOTOR_D_CHANNEL, thrustModel.motor_d_thrust);
////        ledcWrite(MOTOR_C_CHANNEL, thrustModel.motor_c_thrust);
////        ledcWrite(MOTOR_B_CHANNEL, thrustModel.motor_b_thrust);
////        ledcWrite(MOTOR_A_CHANNEL, thrustModel.motor_a_thrust);
//
//
////        Log::print(std::to_string(sampleTime) + ",");
////        Log::print(std::to_string(pitchPIDModel.proportional) + ",");
////        Log::print(std::to_string(pitchPIDModel.integral) + ",");
////        Log::print(std::to_string(pitchPIDModel.derivative) + ",");
////        Log::print(std::to_string(pitchPIDModel.system_constrained_total) + ",");
////
////        Log::print(std::to_string(rollPIDModel.proportional) + ",");
////        Log::print(std::to_string(rollPIDModel.integral) + ",");
////        Log::print(std::to_string(rollPIDModel.derivative) + ",");
////        Log::print(std::to_string(rollPIDModel.system_constrained_total) + ",");
////
////        Log::print(std::to_string(yawPIDModel.proportional) + ",");
////        Log::print(std::to_string(yawPIDModel.integral) + ",");
////        Log::print(std::to_string(yawPIDModel.system_constrained_total) + ",");
////
////        Log::print(std::to_string(thrustModel.motor_a_thrust) + ",");
////        Log::print(std::to_string(thrustModel.motor_b_thrust) + ",");
////        Log::print(std::to_string(thrustModel.motor_c_thrust) + ",");
////        Log::println(static_cast<double>(thrustModel.motor_d_thrust));
//
//        if (count % 50 == 0) {
//            record = drone_controller::FlightRecord();
//            record.millis = static_cast<int32_t>(now);
//
//            record.pitch_setpoint = static_cast<int16_t>(pitchPID.SetPoint());
//            record.roll_setpoint = static_cast<int16_t>(rollPID.SetPoint());
//            record.yaw_setpoint = static_cast<int16_t>(yawPID.SetPoint());
//
//            record.altitude = static_cast<float>(thrustManager.baseThrust());
//
//            record.pitch = current_attitude[0];
//            record.roll = current_attitude[1];
//            record.yaw_rate = current_attitude[3];
//
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
//
//            flightRecorder.insertAndFlushIfReady(record);
//
//            count++;
//        }
//
//        previous_millis = now;
//    } else {
//        emergencyStop();
//        previous_millis = now;
//    }
//}