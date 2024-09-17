#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <Arduino.h>
#include <ServoEasing.hpp>

// ===================================
//            Defines
// ===================================

ServoEasing servo_A, servo_B;
#define servo_Pino_A 32
#define servo_Pino_B 33
#define servo_Pino_C 25
#define servo_Pino_D 26

// ===================================
//            Constantes
// ===================================

const byte velocityTurnMotorA = 0x32;
const byte velocityTurnMotorB = 0x32;
const byte motor_A_IN = 27;
const byte motor_A_IP = 13;
const byte motor_B_IN = 4;
const byte motor_B_IP = 2;
const byte velocityMotorA = 0x5f;
const byte velocityMotorB = 0x5f;

// Variáveis de configuração dos sensores de cor
Adafruit_TCS34725 tcsF = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 tcsR = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 tcsL = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X);

// ===================================
//            Funções
// ===================================

void setServos(){
  pinMode(servo_Pino_A, OUTPUT);
  pinMode(servo_Pino_B, OUTPUT);
  servo_A.attach(servo_Pino_A);
  servo_B.attach(servo_Pino_B);
  servo_A.setEasingType(EASE_CUBIC_IN_OUT);
  servo_B.setEasingType(EASE_CUBIC_IN_OUT);
  servo_A.setSpeed(70);
  servo_B.setSpeed(70);
  servo_A.write(90);
  servo_B.write(90);
}

void checkSensorColor() {
  if (!tcsF.begin()) {
    Serial.println("Não foi possível encontrar o sensor ColorFrente.");
    while (1);
  }
  
  if (!tcsL.begin()) {
    Serial.println("Não foi possível encontrar o sensor ColorLeft.");
    while (1);
  }
  
  if (!tcsR.begin()) {
    Serial.println("Não foi possível encontrar o sensor ColorRight.");
    while (1);
  }
  
  Serial.println("Todos os sensores foram inicializados com sucesso.");
}

String identifyColor(uint16_t red, uint16_t green, uint16_t blue) {
  const uint16_t blackThreshold = 100;
  const uint16_t blueThreshold = 1000;
  
  if (red < blackThreshold && green < blackThreshold && blue < blackThreshold) {
    return "Preto";
  }
  
  if (blue > red && blue > green && blue > blueThreshold) {
    return "Azul";
  }
  
  return "Desconhecida";
}

void setMotors(){
  pinMode(motor_A_IN, OUTPUT);
  pinMode(motor_A_IP, OUTPUT);
  pinMode(motor_B_IN, OUTPUT);
  pinMode(motor_B_IP, OUTPUT);
}

void stopMotors() {
  analogWrite(motor_A_IN, 0);
  analogWrite(motor_A_IP, 0);
  analogWrite(motor_B_IN, 0);
  analogWrite(motor_B_IP, 0);
}

void stopCar(unsigned int timeMotor) {
  stopMotors();
  delay(timeMotor);
}

void moveBackward(unsigned int timeMotor) {
  analogWrite(motor_A_IN, velocityMotorA);
  analogWrite(motor_A_IP, 0);
  analogWrite(motor_B_IN, velocityMotorB);
  analogWrite(motor_B_IP, 0);
  delay(timeMotor);
}

void moveForward(unsigned int timeMotor) {
  analogWrite(motor_A_IN, 0);
  analogWrite(motor_A_IP, velocityMotorA);
  analogWrite(motor_B_IN, 0);
  analogWrite(motor_B_IP, velocityMotorB);
  delay(timeMotor);
}

void moveFrente() {
  analogWrite(motor_A_IN, 0);
  analogWrite(motor_A_IP, velocityMotorA);
  analogWrite(motor_B_IN, 0);
  analogWrite(motor_B_IP, velocityMotorB);
}

void turnRight(unsigned int timeMotor) {
  analogWrite(motor_A_IN, velocityMotorA);
  analogWrite(motor_A_IP, 0);
  analogWrite(motor_B_IN, velocityTurnMotorB);
  analogWrite(motor_B_IP, 0);
  delay(timeMotor);
}

void turnLeft(unsigned int timeMotor) {
  analogWrite(motor_A_IN, velocityTurnMotorA);
  analogWrite(motor_A_IP, 0);
  analogWrite(motor_B_IN, velocityMotorB);
  analogWrite(motor_B_IP, 0);
  delay(timeMotor);
}

void setup() {
  Serial.begin(9600);
  checkSensorColor();
  setMotors();
  setServos();
}

void loop() {
  uint16_t clearF, redF, greenF, blueF;
  uint16_t clearL, redL, greenL, blueL;
  uint16_t clearR, redR, greenR, blueR;
  
  tcsF.getRawData(&redF, &greenF, &blueF, &clearF);
  tcsL.getRawData(&redL, &greenL, &blueL, &clearL);
  tcsR.getRawData(&redR, &greenR, &blueR, &clearR);
  
  Serial.print("Frente - R: "); Serial.print(redF);
  Serial.print(" G: "); Serial.print(greenF);
  Serial.print(" B: "); Serial.print(blueF);
  Serial.print(" C: "); Serial.println(clearF);
  
  Serial.print("Esquerda - R: "); Serial.print(redL);
  Serial.print(" G: "); Serial.print(greenL);
  Serial.print(" B: "); Serial.print(blueL);
  Serial.print(" C: "); Serial.println(clearL);
  
  Serial.print("Direita - R: "); Serial.print(redR);
  Serial.print(" G: "); Serial.print(greenR);
  Serial.print(" B: "); Serial.print(blueR);
  Serial.print(" C: "); Serial.println(clearR);
  
  String colorF = identifyColor(redF, greenF, blueF);
  String colorL = identifyColor(redL, greenL, blueL);
  String colorR = identifyColor(redR, greenR, blueR);

  Serial.print("Cor detectada na frente: ");
  Serial.println(colorF);
  
  Serial.print("Cor detectada à esquerda: ");
  Serial.println(colorL);
  
  Serial.print("Cor detectada à direita: ");
  Serial.println(colorR);
  
  if (colorF == "Preto") {
    stopCar(1000); // Para o carro e aguarda um tempo
  } else {
    moveFrente(); // Move o carro para frente continuamente
  }
  
  delay(1000); // Ajuste o delay conforme necessário
}
