#include "Gpsd.h"
#include <iostream>

#ifndef GPS_WAITING_TIME
#define GPS_WAITING_TIME 1000000
#endif

Gpsd::Gpsd(): m_gps(NULL) {
}

Gpsd::~Gpsd() {
    disconnect();
}

bool Gpsd::connect(const char* _portName, bool _verbose) {
    if (!m_gps) {
        m_gps = new gpsmm(_portName, DEFAULT_GPSD_PORT);

        if (m_gps->stream(WATCH_ENABLE | WATCH_JSON) == nullptr) {
            std::cerr << "No GPSD running." << std::endl;
            return false;
        }

        m_connected = m_gps->is_open();
    }

    return m_connected;
}

void Gpsd::disconnect() {
    if (m_connected)
        m_connected = false;

    if (m_gps) 
        delete m_gps;

    m_gps = NULL;
}

bool Gpsd::printFirmware() {
    if (m_connected)
        std::cout << "GPSD" << std::endl;
    
    return true;
}

bool Gpsd::start() {
    m_thread = std::thread([this]{
        std::cout << "Starting Thread " << std::endl; 
        while (m_connected) {
            
            if (!m_gps->waiting(GPS_WAITING_TIME)) continue;
            struct gps_data_t* gpsd_data;
            if ((gpsd_data = m_gps->read()) == nullptr) {
                std::cerr << "GPSD read error.\n";
                m_connected = false;
            }

            while (((gpsd_data = m_gps->read()) == nullptr) || (gpsd_data->fix.mode < MODE_2D)) {
                // Do nothing until fix, block execution for 1 second (busy wait mitigation)
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }

            m_mutex.lock();
            m_lat = gpsd_data->fix.latitude;
            m_lats.push_back(m_lat);

            m_lng = gpsd_data->fix.longitude;
            m_lngs.push_back(m_lng);

            if (gpsd_data->fix.mode == MODE_3D) {
                m_alt = gpsd_data->fix.altitude;
                m_alts.push_back(m_alt);
            }
            m_mutex.unlock();
        }
    });
    // m_thread.detach();

    return true;
}

bool Gpsd::stop() {
    m_connected = false;
    m_thread.join();

    return true;
}

double Gpsd::getLat() {
    double lat = 0.0;
    int total = m_lats.size();

    if (total == 0)
        return 0.0;

    m_mutex.lock();
    for(int i = 0; i < total; i++) {
        lat += m_lats[i];
    }
    m_mutex.unlock();
    return lat / (double)total;
}

double Gpsd::getLng() {
    double lng = 0.0;
    int total = m_lngs.size();

    if (total == 0)
        return 0.0;

    m_mutex.lock();
    for(int i = 0; i < total; i++) {
        lng += m_lngs[i];
    }
    m_mutex.unlock();
    return lng / (double)total;
}

double Gpsd::getAlt() {
    double alt = 0.0;
    int total = m_alts.size();

    if (total == 0)
        return 0.0;

    m_mutex.lock();
    for(int i = 0; i < total; i++) {
        alt += m_alts[i];
    }
    m_mutex.unlock();
    return alt / (double)total;
};