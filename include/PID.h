//
// Created by ahmed on 4/20/2023.
//

#ifndef ESP_HOVER_PID_H
#define ESP_HOVER_PID_H

#include <Arduino.h>
#include <utility>
#include "model.h"

class PID {

public:
    PID(double highBoundary, double lowBoundary, double maxNeglectedError = 0.01,
                 double minNeglectedError = -0.01);

    void SetPoint(double setPoint);

    double SetPoint() const;

    void Gains(double kp, double ki, double kd);

    PIDModel Compute(double sampleTime, double measurement, double rate_of_change);

private:
    double _highBoundary{0}, _lowBoundary{0}, _maxNeglectedError{0}, _minNeglectedError{0};
    double _setPoint{0};
    double _kp{0}, _ki{0}, _kd{0};
    double _accumError{0};
    double _previous_m{0};
};

#endif //ESP_HOVER_PID_H
