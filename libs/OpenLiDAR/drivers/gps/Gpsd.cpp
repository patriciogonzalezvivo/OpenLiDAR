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

    m_lats.clear();
    m_lngs.clear();
    m_alts.clear();

    return m_connected;
}

void Gpsd::disconnect() {
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

void Gpsd::update() {
    if (!m_connected || m_gps == NULL)
        return;
    
    if (!m_gps->waiting(GPS_WAITING_TIME))
        return;

    struct gps_data_t* gpsd_data;
    if ((gpsd_data = m_gps->read()) == NULL) {
        std::cerr << "GPSD read error.\n";
        m_connected = false;
    }

    if (((gpsd_data = m_gps->read()) == NULL) || (gpsd_data->fix.mode < MODE_2D)) {
        // Do nothing until fix, block execution for 1 second (busy wait mitigation)
        return;
    }

    m_lat = gpsd_data->fix.latitude;
    m_lats.push_back(m_lat);

    m_lng = gpsd_data->fix.longitude;
    m_lngs.push_back(m_lng);

    if (gpsd_data->fix.mode == MODE_3D) {
        m_alt = gpsd_data->fix.altitude;
        m_alts.push_back(m_alt);
    }
}

double Gpsd::getLat() {
    double lat = 0.0;
    int total = m_lats.size();

    if (total == 0)
        return 0.0;

    for(int i = 0; i < total; i++) {
        lat += m_lats[i];
    }
    return lat / (double)total;
}

double Gpsd::getLng() {
    double lng = 0.0;
    int total = m_lngs.size();

    if (total == 0)
        return 0.0;

    for(int i = 0; i < total; i++)
        lng += m_lngs[i];
    
    return lng / (double)total;
}

double Gpsd::getAlt() {
    double alt = 0.0;
    int total = m_alts.size();

    if (total == 0)
        return 0.0;

    for(int i = 0; i < total; i++)
        alt += m_alts[i];
    
    return alt / (double)total;
};