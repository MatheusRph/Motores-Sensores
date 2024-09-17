

//Criado por Thales Santos em 13/09/2024
//Controle para motores de um Robô

#include <Arduino.h>
//#include <analogWrite.h>
#include <ServoEasing.hpp>

#include "Adafruit_TCS34725.h" //Biblioteca para o sensor de cor

//M1 27 13
//M2 4 2
//M3 17 12
//M4 15 14
// variaveis para definição da velocidade dos motores
const byte velocityMotorA = 0x5f; // 127
const byte velocityMotorB = 0x5f; // 127
// variaveis para definição da velocidade dos motores
const byte velocityTurnMotorA = 0x32; // 50
const byte velocityTurnMotorB = 0x32; // 50
unsigned int timeMotor = 1000;  //Tempo de ligar o Motor
//Motor A;
const byte motor_A_IA = 27;
const byte motor_A_IB = 13;
//Motor B;
const byte motor_B_IA = 4;
const byte motor_B_IB = 2;

ServoEasing servo_A, servo_B;
#define servo_Pino_A 32
#define servo_Pino_B 33
#define servo_Pino_C 25
#define servo_Pino_D 26

Adafruit_TCS34725 tcsF = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X); //Definição do primeiro sensor (Sensor da Frete)

bool isBlack(uint16_t r, uint16_t g, uint16_t b, uint16_t threshold = 30) {
  return (r < threshold && g < threshold && b < threshold);
}

//#define modulo_Servo_Pino_D 33
//#define modulo_Servo_Pino_E 25
//#define modulo_Servo_Pino_F 26

void setServos(){
  //define os pinos dos Servos como Saída
  pinMode(servo_Pino_A, OUTPUT);
  pinMode(servo_Pino_B, OUTPUT);
  //anexa os Pinos dos Servo
  servo_A.attach(servo_Pino_A);
  servo_B.attach(servo_Pino_B);
  //setEasingType(EASE_CUBIC_IN_OUT)
  servo_A.setEasingType(EASE_CUBIC_IN_OUT);
  servo_B.setEasingType(EASE_CUBIC_IN_OUT);
  //define velocidade dos Servos
  servo_A.setSpeed(70);
  servo_B.setSpeed(70);
  //define a posição inicial dos Servos
  servo_A.write(90);
  servo_B.write(90);
}

void setMotors(){
  pinMode(motor_A_IA, OUTPUT);
  pinMode(motor_A_IB, OUTPUT);
  pinMode(motor_B_IA, OUTPUT);
  pinMode(motor_B_IB, OUTPUT);
}

void stopMotors() {
  // Parar o Carro
  analogWrite(motor_A_IA, 0);
  analogWrite(motor_A_IB, 0);
  analogWrite(motor_B_IA, 0);
  analogWrite(motor_B_IB, 0);
}

void stopCar(unsigned int timeMotor) {
  // Parar o Carro
  analogWrite(motor_A_IA, 0);
  analogWrite(motor_A_IB, 0);
  analogWrite(motor_B_IA, 0);
  analogWrite(motor_B_IB, 0);
  delay(timeMotor);
}

void moveBackward(unsigned int timeMotor) {
  // Mover carro para tras
  analogWrite(motor_A_IA, velocityMotorA);
  analogWrite(motor_A_IB, 0);
  analogWrite(motor_B_IA, velocityMotorB);
  analogWrite(motor_B_IB, 0);
  delay(timeMotor);
}

void moveForward(unsigned int timeMotor) {
  // Mover carro para frente
  analogWrite(motor_A_IA, 0);
  analogWrite(motor_A_IB, velocityMotorA);
  analogWrite(motor_B_IA, 0);
  analogWrite(motor_B_IB, velocityMotorB);
  delay(timeMotor);
}

void moveFrente() {
  // Mover carro para frente
  analogWrite(motor_A_IA, 0);
  analogWrite(motor_A_IB, velocityMotorA);
  analogWrite(motor_B_IA, 0);
  analogWrite(motor_B_IB, velocityMotorB);
}

void turnRight(unsigned int timeMotor) {
  // Mover carro para direita
  analogWrite(motor_A_IA, velocityMotorA);
  analogWrite(motor_A_IB, 0);
  analogWrite(motor_B_IA, velocityTurnMotorB);
  analogWrite(motor_B_IB, 0);
  delay(timeMotor);
}

void turnLeft(unsigned int timeMotor) {
  // Mover carro para esquerda
  analogWrite(motor_A_IA, velocityTurnMotorA);
  analogWrite(motor_A_IB, 0);
  analogWrite(motor_B_IA, velocityMotorB);
  analogWrite(motor_B_IB, 0);
  delay(timeMotor);
}

void setup() {
  setMotors();
  setServos();
 }

void loop() {
  moveFrente();
  if (isBlack){
    
  }
}

