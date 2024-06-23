//
// Created by ahmed on 4/25/2023.
//

#include"FlightRecorder.h"
#include "Log.h"

FlightRecorder::FlightRecorder(const std::function<void(std::vector<drone_controller::FlightRecord> &&)> &onReadyToFlush) : _onReadyToFlush(
        onReadyToFlush) {}

void FlightRecorder::insertAndFlushIfReady(const drone_controller::FlightRecord& record) {
    _records.emplace_back(record);
    if (_records.size() >= MAX_RECORDS_CAPACITY) flushRecords();
}

void FlightRecorder::flushNow() {
    if (!_records.empty()) flushRecords();
}

void FlightRecorder::flushRecords() {
    _onReadyToFlush(std::move(_records));
    _records.clear();
}

