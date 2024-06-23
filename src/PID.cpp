//
// Created by ahmed on 4/20/2023.
//

#include "PID.h"
#include "Log.h"

PID::PID(double highBoundary, double lowBoundary, double maxNeglectedError, double minNeglectedError)
        : _highBoundary{highBoundary},
          _lowBoundary{lowBoundary},
          _maxNeglectedError{maxNeglectedError},
          _minNeglectedError{minNeglectedError} {}

// Will be updated from BLE core
void PID::Gains(double kp, double ki, double kd) {
    _accumError = 0; // Reset the accumulated error
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

// Will be updated from BLE core
void PID::SetPoint(const double setPoint) {
    _setPoint = setPoint;
}

double PID::SetPoint() const {
    return _setPoint;
}

PIDModel PID::Compute(double sampleTime, double measurement, double rate_of_change) {
    PIDModel model;

    double error = _setPoint - measurement;

    // Proportional
    model.proportional = _kp * error;

    // Integral
    if (error >= _maxNeglectedError || error <= _minNeglectedError) {
        double error_area = error * sampleTime;
        _accumError += error_area;

        double integral = _ki * _accumError;

        if (integral > _highBoundary) {
            integral = _highBoundary;
            _accumError -= error_area;
        } else if (integral < _lowBoundary) {
            integral = _lowBoundary;
            _accumError -= error_area;
        }

        model.integral = integral;
    } else {
        model.integral = _ki * _accumError;
    }

    // Derivative
    if (rate_of_change == std::numeric_limits<double>::max()) {
        double yaw_measurement_rate = (measurement - _previous_m) / sampleTime;
        model.derivative = _kd * yaw_measurement_rate;
        _previous_m = measurement;
    } else {
        model.derivative = _kd * rate_of_change;
    }

    double total = model.proportional + model.integral - model.derivative;
    model.system_constrained_total = constrain(total, _lowBoundary, _highBoundary);

    return model;
}


