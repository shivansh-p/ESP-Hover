//
// Created by ahmed on 4/24/2023.
//

#ifndef ESP_HOVER_XQUADTHRUSTMANAGER_H
#define ESP_HOVER_XQUADTHRUSTMANAGER_H

#include "ThrustManager.h"
#include "Filter.h"

class XQuadThrustManager : public ThrustManager {
public:
    XQuadThrustManager(double highBoundary, double lowBoundary, int filterWeight);

    ThrustModel calculateMotorsThrust(double pitchPID, double rollPID, double yawPID) override;

    void resetMotorsFilters();

private:
    double _highBoundary{0}, _lowBoundary{0};
    ExponentialFilter<double> _motorAFilter;
    ExponentialFilter<double> _motorBFilter;
    ExponentialFilter<double> _motorCFilter;
    ExponentialFilter<double> _motorDFilter;

    int calculateSingleMotorThrust(ExponentialFilter<double> &motorFilter, double speed);
};

#endif //ESP_HOVER_XQUADTHRUSTMANAGER_H
