#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

//initialiser le MPU6050 acceleromètre
MPU6050 mpu;

// Déclaration des servos
Servo servoAileronLeft;
Servo servoAileronRight;

// Pins pour les servos
const int aileronLeftPin = 3;
const int aileronRightPin = 5;

// Fonction pour mapper les angles aux positions des servos
int mapAngleToServo(float angle, float minAngle, float maxAngle, int minServo, int maxServo) {
  return map(angle, minAngle, maxAngle, minServo, maxServo);
}

void setup() {
  Wire.begin();
  Serial.begin(9600);

  // Initialisation du MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  // Initialisation des servos
  servoAileronLeft.attach(aileronLeftPin);
  servoAileronRight.attach(aileronRightPin);

  // Positionner les servos au centre
  servoAileronLeft.write(90);
  servoAileronRight.write(90);
}

void loop() {
  // Lire les valeurs de l'accéléromètre et du gyroscope
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convertir les valeurs en angles
  float ax_g = ax / 16384.0;
  float ay_g = ay / 16384.0;
  float az_g = az / 16384.0;

  float roll = atan(ay_g / sqrt(pow(ax_g, 2) + pow(az_g, 2))) * 180 / PI;
  float pitch = atan(-ax_g / sqrt(pow(ay_g, 2) + pow(az_g, 2))) * 180 / PI;

  // Déterminer les positions des servos pour les ailerons
  int servoAileronLeftPos = mapAngleToServo(roll + pitch, -45, 45, 0, 180);
  int servoAileronRightPos = mapAngleToServo(roll - pitch, -45, 45, 0, 180);

  // Positionner les servos
  servoAileronLeft.write(servoAileronLeftPos);
  servoAileronRight.write(servoAileronRightPos);

  // Afficher les angles et les positions des servos sur le moniteur série
  Serial.print("Roll: "); Serial.print(roll);
  Serial.print(" Pitch: "); Serial.print(pitch);
  Serial.print(" Ail Left: "); Serial.print(servoAileronLeftPos);
  Serial.print(" Ail Right: "); Serial.println(servoAileronRightPos);

  delay(50);
}

