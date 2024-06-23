//
// Created by ahmed on 5/2/2023.
//

#include "../include/ThrustManager.h"

ThrustManager::ThrustManager() = default;

void ThrustManager::updateBaseThrust(int thrust) {
    _baseThrust = thrust;
}

int ThrustManager::baseThrust() const {
    return _baseThrust;
}