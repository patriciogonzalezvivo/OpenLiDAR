#pragma once

#include "MountDriver.h"

//              4º /sec, 2º /sec,  1º/sec,  0.5º/sec,   32x,    16x,    8x,     4x,     2x   
typedef enum {  SR_1,    SR_2,     SR_3,    SR_4,       SR_5,   SR_6,   SR_7,   SR_8,   SR_9 } CELESTRON_SLEW_RATE;
typedef enum {  TRACKING_OFF, TRACK_ALTAZ, TRACK_EQN, TRACK_EQS } CELESTRON_TRACK_MODE;
typedef enum {  RA_AXIS, DEC_AXIS } CELESTRON_AXIS;
typedef enum {  CELESTRON_N, CELESTRON_S, CELESTRON_W, CELESTRON_E } CELESTRON_DIRECTION;

class Celestron : public MountDriver {
public:
    Celestron();
    virtual ~Celestron();

    // Driver Abstract Methods 
    bool        connect(const char* _portName, bool _verbose);
    void        disconnect();
    bool        printFirmware();

    // Mount Abstract Methods
    bool        pan(double _targetAngle, float _speed, std::function<bool(double&, double&)> _callback);
    bool        reset(bool _verbose);

    double      getAz();
    double      getAlt();

    // Celestron Only Methods
    bool        echo();
    bool        checkConnection();
    bool        checkAligned();
    bool        hibernate();
    bool        wakeup();
    bool        sync(double _ra, double _dec);

    bool        move(CELESTRON_DIRECTION _dir, CELESTRON_SLEW_RATE _rate);
    bool        stop(CELESTRON_DIRECTION _dir);
    bool        abort();

    bool        gotoRADec(double _ra, double _dec);
    bool        gotoAzAlt(double _az, double _alt);

    bool        getVersion(char* _version, int _size);
    bool        getVariant(char* _variant);
    bool        getModel(char* _model, int _size, bool* _isGem);
    bool        getDevFirmware(int _dev, char* _version, int _size);

    // Time & Location
    bool        setLocation(double _longitude, double _latitude);
    // bool        setDatetime(struct ln_date* _utc, double _utc_offset);

    bool        getRADec(double* _ra, double* _dec);
    bool        getAzAlt(double* _az, double* _alt);

    bool        slewRADec(double _ra, double _dec);
    bool        slewAzAlt(double _az, double _alt);
    bool        isSlewing();

    // Track Mode
    bool        getTrackMode(CELESTRON_TRACK_MODE *_mode);
    bool        setTrackMode(CELESTRON_TRACK_MODE _mode);

    // Pulse Guide (experimental)
    int         sendPulse(CELESTRON_DIRECTION _dir, signed char _rate, unsigned char _duration_msec);
    int         getPulseStatus(CELESTRON_DIRECTION _dir, bool &_pulse_state);

private:
    int         send_command(const char *_cmd, int _cmd_len, char *_resp, int _resp_len);
    int         send_passthrough(int _dest, int _cmd_id, const char *_payload, int _payload_len, char *_response, int _response_len);

    int         m_fd = 0;
};
