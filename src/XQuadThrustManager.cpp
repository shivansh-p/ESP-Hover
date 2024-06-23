//
// Created by ahmed on 4/24/2023.
//

#include "../include/XQuadThrustManager.h"

XQuadThrustManager::XQuadThrustManager(double highBoundary, double lowBoundary, int filterWeight)
        : ThrustManager(), _motorAFilter(filterWeight, 0),
          _motorBFilter(filterWeight, 0),
          _motorCFilter(filterWeight, 0),
          _motorDFilter(filterWeight, 0),
          _highBoundary(highBoundary),
          _lowBoundary(lowBoundary) {}

ThrustModel XQuadThrustManager::calculateMotorsThrust(double pitchPID, double rollPID, double yawPID) {
    ThrustModel model{};
    model.motor_a_thrust = calculateSingleMotorThrust(_motorAFilter, (_baseThrust - pitchPID - rollPID + yawPID));
    model.motor_b_thrust = calculateSingleMotorThrust(_motorBFilter, (_baseThrust + pitchPID + rollPID + yawPID));
    model.motor_c_thrust = calculateSingleMotorThrust(_motorCFilter, (_baseThrust - pitchPID + rollPID - yawPID));
    model.motor_d_thrust = calculateSingleMotorThrust(_motorDFilter, (_baseThrust + pitchPID - rollPID - yawPID));

    return model;
}

int XQuadThrustManager::calculateSingleMotorThrust(ExponentialFilter<double> &motorFilter, double speed) {
    double constrainedSpeed = constrain(speed, _lowBoundary, _highBoundary);
    motorFilter.Filter(constrainedSpeed);
    return static_cast<int>(motorFilter.Current());
}

void XQuadThrustManager::resetMotorsFilters() {
    _motorAFilter.SetCurrent(0);
    _motorBFilter.SetCurrent(0);
    _motorCFilter.SetCurrent(0);
    _motorDFilter.SetCurrent(0);
}
