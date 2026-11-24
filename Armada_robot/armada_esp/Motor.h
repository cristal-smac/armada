#pragma once
#include <SCServo.h>

class Motor
{
public:
    Motor(int id, int rxd = 16, int txd = 17, long baudrate = 1000000); // STS3215 servo spec 1M baudrate

    bool init();
    bool set_rpm_speed(float rpm);
    float get_speed_rpm();
    bool stop();

    /**
     * Position cumulée de l'arbre en tours (peut dépasser 1 tour, signée).
     * Le servo ne compte qu'un tour (0..4095) : on déroule le compteur ici,
     * donc cette méthode doit être appelée assez souvent (< 1/2 tour entre 2 appels).
     * @return false si le servo n'a pas répondu (rev est alors inchangé)
     */
    bool get_position_rev(float &rev);

private:
    static constexpr int POS_RANGE = 4096; // une position servo = 0..4095
    static constexpr float TICKS_PER_REV = 4095.0f; // STS3215 servo spec 4095 tic for 1 revolution
    static constexpr float MAX_RPM = 60.0f;
    static constexpr int SPEED_ATTEMPTS = 2; // essais pour une consigne de vitesse
    static constexpr int STOP_ATTEMPTS = 3;  // essais pour un arrêt (plus important)

    int rpm_to_ticks(float rpm) const;
    float ticks_to_rpm(int ticks) const;

    SMS_STS _sms_sts;
    HardwareSerial _serial;
    int _id;
    int _rxd;
    int _txd;
    long _baudrate;

    int _last_pos = -1;      // dernière position lue (-1 = aucune)
    long _total_ticks = 0;   // position cumulée déroulée
};