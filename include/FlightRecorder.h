//
// Created by ahmed on 4/29/2023.
//

#ifndef ESP_HOVER_FLIGHTRECORDER_H
#define ESP_HOVER_FLIGHTRECORDER_H

#define MAX_RECORDS_CAPACITY 15

#include "model.h"
#include <vector>
#include <functional>
#include <drone_controller/FlightRecord.h>

class FlightRecorder {
public:
    explicit FlightRecorder(const std::function<void(std::vector<drone_controller::FlightRecord> &&)>& onReadyToFlush);

    void insertAndFlushIfReady(const drone_controller::FlightRecord& record);

    void flushNow();

private:
    std::vector<drone_controller::FlightRecord> _records;
    std::function<void(std::vector<drone_controller::FlightRecord> &&)> _onReadyToFlush;

    void flushRecords();
};

#endif //ESP_HOVER_FLIGHTRECORDER_H
