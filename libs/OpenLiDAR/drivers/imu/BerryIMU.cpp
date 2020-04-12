#include "BerryIMU.h"

BerryIMU::BerryIMU() {

}

BerryIMU::~BerryIMU() {

}

bool BerryIMU::connect(const char* _portName, bool _verbose) {
    return true;
}

void BerryIMU::disconnect() {
}

bool BerryIMU::calibrate() {
    return true;
}

bool BerryIMU::printFirmware() {
    return true;
}

bool BerryIMU::start(bool _verbose) {
    return true;
}

bool BerryIMU::stop(bool _verbose) {
    return true;
}