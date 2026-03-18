#include <SCServo.h>

SMS_STS sms_sts;
// UART configuration
#define S_RXD 16
#define S_TXD 17

int TEST_ID = 1;
int SPEED = 1500; 

void setup()
{
  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  sms_sts.pSerial = &Serial1;
  delay(1000);
  Serial.println("Début du test de rotation continue");
}

void loop()
{
  //set speed
  int result = sms_sts.WritePosEx(TEST_ID, 4095, SPEED, 0); // 4095 pour position, inutile???
  if (result == -1)
  {
    Serial.println("Erreur : Impossible de démarrer la rotation");
  }
  else
  {
    Serial.print("Rotation à vitesse fixe pendant 3 secondes...");
    delay(3000); 
  }

  // Arrêter le servo pendant 2 secondes
  result = sms_sts.WritePosEx(TEST_ID, 0, 0, 0); // Vitesse = 0 pour arrêter
  if (result == -1)
  {
    Serial.println("Erreur : Impossible d'arrêter le servo");
  }
  else
  {
    Serial.println("Servo arrêté pendant 2 secondes...");
    delay(2000); 
  }
}
