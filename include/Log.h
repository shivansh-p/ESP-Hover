//
// Created by ahmed on 4/30/2023.
//

#ifndef ESP_HOVER_LOG_H
#define ESP_HOVER_LOG_H

#include <Arduino.h>
#include <ros.h>

class Log {
public:
    static void setup(int baud_rate, const std::shared_ptr<ros::NodeHandle>& handle);

    static void println(const std::string &msg);

    static void println(const double &msg);

    static void print(const std::string &msg);

    static std::shared_ptr<ros::NodeHandle> nh;
};

#endif //ESP_HOVER_LOG_H
