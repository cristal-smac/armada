#include <SCServo.h>

SMS_STS sms_sts;
// UART configuration
#define S_RXD 16
#define S_TXD 17

int TEST_ID = 1;
int SPEED = 30; // vitesse en rpm

class Motor
{
public:
  Motor(int id, int rxd = 16, int txd = 17, long baudrate = 1000000);

  bool init();
  bool set_rpm_speed(float rpm);
  float get_speed_rpm();
  bool stop();

private:
  static constexpr float TICKS_PER_REV = 4095.0f;
  static constexpr float MAX_RPM = 60.0f;

  int rpm_to_ticks(float rpm) const;
  float ticks_to_rpm(int ticks) const;

  SMS_STS _sms_sts;
  HardwareSerial _serial;
  int _id;
  int _rxd;
  int _txd;
  long _baudrate;
};

// ─── Private helpers ─────────────────────────────────────────────────────────

int Motor::rpm_to_ticks(float rpm) const
{
  return static_cast<int>(rpm / 60.0f * TICKS_PER_REV);
}

float Motor::ticks_to_rpm(int ticks) const
{
  return static_cast<float>(ticks) * 60.0f / TICKS_PER_REV;
}

// ─── Public ──────────────────────────────────────────────────────────────────

Motor::Motor(int id, int rxd, int txd, long baudrate)
    : _serial(1),
      _id(id),
      _rxd(rxd),
      _txd(txd),
      _baudrate(baudrate)
{
  _serial.begin(_baudrate, SERIAL_8N1, _rxd, _txd);
  _sms_sts.pSerial = &_serial;
  delay(1000);
}

bool Motor::init()
{
  int result = _sms_sts.WritePosEx(_id, 0, 0, 0);
  if (result == -1)
  {
    Serial.printf("[Motor %d] Erreur init\n", _id);
    return false;
  }
  return true;
}

bool Motor::set_rpm_speed(float rpm)
{
  if (rpm < 0.0f)
    rpm = 0.0f;
  if (rpm > MAX_RPM)
    rpm = MAX_RPM;

  int ticks = rpm_to_ticks(rpm);
  Serial.printf("[Motor %d] set_rpm_speed(%.2f RPM) → %d ticks/s\n", _id, rpm, ticks);

  int result = _sms_sts.WritePosEx(_id, 4095, ticks, 0);
  if (result == -1)
  {
    Serial.printf("[Motor %d] Erreur : impossible de régler la vitesse\n", _id);
    return false;
  }
  return true;
}

float Motor::get_speed_rpm()
{
  int ticks = _sms_sts.ReadSpeed(_id);
  if (ticks == -1)
  {
    Serial.printf("[Motor %d] Erreur : impossible de lire la vitesse\n", _id);
    return -1.0f;
  }
  float rpm = ticks_to_rpm(ticks);
  Serial.printf("[Motor %d] get_speed_rpm() → %d ticks/s = %.2f RPM\n", _id, ticks, rpm);
  return rpm;
}

bool Motor::stop()
{
  int result = _sms_sts.WritePosEx(_id, 0, 0, 0);
  if (result == -1)
  {
    Serial.printf("[Motor %d] Erreur : impossible d'arrêter le moteur\n", _id);
    return false;
  }
  return true;
}

Motor motor(TEST_ID, S_RXD, S_TXD, 1000000);
void setup()
{
  Serial.begin(115200);
  // Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  // sms_sts.pSerial = &Serial1;

  delay(1000);
  Serial.println("Début du test de rotation continue");
}

void loop()
{
  motor.set_rpm_speed(SPEED);
  delay(3000);

  motor.stop();
  delay(2000);
}
