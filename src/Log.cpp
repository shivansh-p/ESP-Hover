//
// Created by ahmed on 4/30/2023.
//

#include "Log.h"
#include <utility>

//#define Serial Serial2

//#define SerialDebugMode
#define ROSDebugMode

std::shared_ptr<ros::NodeHandle> Log::nh = nullptr;

void Log::setup(int baud_rate, const std::shared_ptr<ros::NodeHandle>& handle) {
#if defined(SerialDebugMode)
    Serial.begin(baud_rate);
#elif defined(ROSDebugMode)
    Log::nh = handle;
#endif
}

void Log::print(const std::string &msg) {
#if defined(SerialDebugMode)
    Serial.print(msg.c_str());
#elif defined(ROSDebugMode)
    Log::nh->loginfo(msg.c_str());
#endif
}

void Log::println(const std::string &msg) {
#if defined(SerialDebugMode)
    Serial.println(msg.c_str());
#elif defined(ROSDebugMode)
    Log::nh->loginfo(msg.c_str());
#endif
}

void Log::println(const double &value) {
#if defined(SerialDebugMode)
    Serial.println(value);
#elif defined(ROSDebugMode)
    Log::nh->loginfo(std::to_string(value).c_str());
#endif
}

