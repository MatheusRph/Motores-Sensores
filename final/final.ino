// ===================================
//            Bibliotecas
// ===================================

#include <Arduino.h> //Inclui a biblioteca do arduino
//#include <analogWrite.h>
#include <ServoEasing.hpp> //Inclui a biblioteca dos Servos
#include <Wire.h>
#include <cmath> // Adicione esta linha para usar sqrt


// ===================================
//            Defines
// ===================================

/*Define os pinos do sensor ultrassônico*/
#define sensorIndutivo A0
#define triggerPin 6
#define echoPin 7
int cm = 0;

/*Definição do arduino em geral*/
#define D_in 16 //Nomeia o pino 6 do Arduino

// ===================================
//            Constantes
// ===================================

/*Constantes de velocidade*/
// Constantes para definição da velocidade dos motores ao iniciar motor
const byte velocityTurnMotorA = 0x32; // 50
const byte velocityTurnMotorB = 0x32; // 50
unsigned int timeMotor = 1000;  //Tempo de ligar o Motor

/*Constantes para definição dos pinos dos motores*/ 
//M1 27 13
//M2 4 2
//M3 17 12
//M4 15 14

const byte motor_RA_IN = 2; //Motor A; // Pino de entrada negativo do Motor A (Faz a roda girar no sentido anti-horário)
const byte motor_RA_IP = 4; //Motor A; // Pino de entrada positiva do Motor A (Faz a roda girar no sentido horário)

//Motor B;
const byte motor_RB_IN = 15;  // Pino de entrada negativa do Motor B (Faz a roda girar no sentido anti-horário)
const byte motor_RB_IP = 14;  //Pino de entrada positiva do Motor B (Faz a roda girar no sentido horário)

const byte motor_LA_IN = 13; //Motor A; // Pino de entrada negativo do Motor A (Faz a roda girar no sentido anti-horário)
const byte motor_LA_IP = 27; //Motor A; // Pino de entrada positiva do Motor A (Faz a roda girar no sentido horário)

//Motor B;
const byte motor_LB_IN = 17;  // Pino de entrada negativa do Motor B (Faz a roda girar no sentido anti-horário)
const byte motor_LB_IP = 12;  //Pino de entrada positiva do Motor B (Faz a roda girar no sentido horário)

// ===================================
//            Variáveis
// ===================================
int rampTime = 1000;
int minSpeed = 127;
int maxSpeed = 190;

// Variáveis para definição da velocidade dos motores
byte velocityMotorR = 200; // Velocidade dos motores do lado Direito
byte velocityMotorL = 200; // Velocidade do Motor do lado esquerdo

/*Variáveis de tempo*/
unsigned int timeXV = 14 +1;

// ===================================
//            Funções
// ===================================

/*
    Função: setUltra
    Descrição: Configura os pinos do sensor ultrassônico.
*/
void setUltra(){
    pinMode(triggerPin, OUTPUT); // Clear the trigger
    pinMode(echoPin, INPUT);
}

/*
    Função: readUltrassonic
    Descrição: ~Faz o acionamento do sensor para depois retornar o valor
*/
long readUltrassonic()
{
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    return pulseIn(echoPin, HIGH);
}

/*
    Função: setMotors
    Descrição: Configura os pinos dos motores como saída.
*/
void setMotors() {
    pinMode(motor_RA_IN, OUTPUT); // Configura o pino de controle positivo do Motor A como saída
    pinMode(motor_RA_IP, OUTPUT); // Configura o pino de controle negativo do Motor A como saída

    pinMode(motor_LA_IN, OUTPUT);  // Configura o pino de controle positivo do Motor B como saída
    pinMode(motor_LA_IP, OUTPUT);  // Configura o pino de controle negativo do Motor B como saída

    pinMode(motor_RB_IN, OUTPUT); // Configura o pino de controle positivo do Motor A como saída
    pinMode(motor_RB_IP, OUTPUT); // Configura o pino de controle negativo do Motor A como saída

    pinMode(motor_LB_IN, OUTPUT);  // Configura o pino de controle positivo do Motor B como saída
    pinMode(motor_LB_IP, OUTPUT);  // Configura o pino de controle negativo do Motor B como saída
}

/*
    Função: stopMotors
    Descrição: Para os motores imediatamente.
*/
void stopMotors() {
  // Parar os Motores
  analogWrite(motor_RA_IN, 0); // Define a saída do pino positivo do Motor A para 0
  analogWrite(motor_RA_IP, 0); // Define a saída do pino negativo do Motor A para 0

  analogWrite(motor_LA_IN, 0);  // Define a saída do pino positivo do Motor B para 0
  analogWrite(motor_LA_IP, 0);  // Define a saída do pino negativo do Motor B para 0

  analogWrite(motor_RB_IN, 0); // Define a saída do pino positivo do Motor C para 0
  analogWrite(motor_RB_IP, 0); // Define a saída do pino negativo do Motor C para 0

  analogWrite(motor_LB_IN, 0);  // Define a saída do pino positivo do Motor D para 0
  analogWrite(motor_LB_IP, 0);  // Define a saída do pino negativo do Motor D para 0
}


/*
    Função: stopCar
    Descrição: Para os motores e aguarda um tempo definido.
*/
void stopCar(unsigned int timeMotor) {
  // Parar o Carro
  analogWrite(motor_RA_IN, 0);  // Define a saída do pino positivo do Motor A para 0, parando o motor
  analogWrite(motor_RA_IP, 0);  // Define a saída do pino negativo do Motor A para 0, parando o motor

  analogWrite(motor_LA_IN, 0);  // Define a saída do pino positivo do Motor B para 0, parando o motor
  analogWrite(motor_LA_IP, 0);  // Define a saída do pino negativo do Motor B para 0, parando o motor

  analogWrite(motor_RB_IN, 0);  // Define a saída do pino positivo do Motor C para 0, parando o motor
  analogWrite(motor_RB_IP, 0);  // Define a saída do pino negativo do Motor C para 0, parando o motor

  analogWrite(motor_LB_IN, 0);  // Define a saída do pino positivo do Motor D para 0, parando o motor
  analogWrite(motor_LB_IP, 0);  // Define a saída do pino negativo do Motor D para 0, parando o motor
  
  delay(timeMotor); // Aguarda pelo tempo especificado (timeMotor) antes de continuar
}


/*
    Função: moveBackward
    Descrição: Move o carro para trás por um tempo determinado.
*/
void moveBackward(unsigned int timeMotor) {

    stopMotors();
  // Mover carro para trás
    for (int speed = minSpeed; speed <= maxSpeed; speed++) {
        analogWrite(motor_RA_IN, velocityMotorR); // Ativa o motor A no sentido ant-horário com a velocidade definida
        analogWrite(motor_RB_IN, speed); // Ativa o motor B no sentido ant-horário com a velocidade definida
        analogWrite(motor_LA_IN, speed); // Ativa o motor A no sentido ant-horário com a velocidade definida
        analogWrite(motor_LB_IN, speed); // Ativa o motor B no sentido ant-horário com a velocidade definida
        delay(rampTime / maxSpeed);

    }

    delay(timeMotor); // Aguarda pelo tempo especificado (timeMotor) antes de continuar

    for (int speed = maxSpeed; speed >= minSpeed; speed--) {
        analogWrite(motor_RA_IN, speed); // Ativa o motor A no sentido ant-horário com a velocidade definida
        analogWrite(motor_RB_IN, speed); // Ativa o motor B no sentido ant-horário com a velocidade definida
        analogWrite(motor_LA_IN, speed); // Ativa o motor A no sentido ant-horário com a velocidade definida
        analogWrite(motor_LB_IN, speed); // Ativa o motor B no sentido ant-horário com a velocidade definida
        delay(rampTime / maxSpeed);
    }
}

/*
    Função: moveForward
    Descrição: Move o carro para frente por um tempo determinado.
*/
void moveForward(unsigned int timeMotor) {

    stopMotors();
    // Mover carro para frente
    for (int speed = minSpeed; speed <= maxSpeed; speed++) {
        analogWrite(motor_RA_IP, speed); // Ativa o motor A no sentido horário com a velocidade definida
        analogWrite(motor_RB_IP, speed); // Ativa o motor B no sentido horário com a velocidade definida
        analogWrite(motor_LA_IP, speed); // Desativa o motor A no sentido horário
        analogWrite(motor_LB_IP, speed); // Desativa o motor B no sentido horário
        delay(rampTime / maxSpeed);
    }

    delay(timeMotor); // Aguarda pelo tempo especificado (timeMotor) antes de continuar

    for (int speed = maxSpeed; speed >= minSpeed; speed--) {
        analogWrite(motor_RA_IP, speed); // Ativa o motor A no sentido horário com a velocidade definida
        analogWrite(motor_RB_IP, speed); // Ativa o motor B no sentido horário com a velocidade definida
        analogWrite(motor_LA_IP, speed); // Desativa o motor A no sentido horário
        analogWrite(motor_LB_IP, speed); // Desativa o motor B no sentido horário
        delay(rampTime / maxSpeed);
    }
}

void Direita (int timeMotor) {
// MotorFrontLeft - MotorBackRight (Mesma direção) Horário
// MotorFrontRight	- MotorBackLeft (Mesma direção) Ant-Horário

    stopMotors();

    for (int speed = minSpeed; speed <= maxSpeed; speed++) {
        analogWrite(motor_RA_IN, speed); //Liga o motor com uma velocidade x, no sentido ant-horário

        analogWrite(motor_LB_IN, speed); //Liga o motor com uma velocidade x, no sentido ant-horário
        
        analogWrite(motor_LA_IP, speed); //Liga o motor com uma velocidade x, no sentido horário

        analogWrite(motor_RB_IP, speed); //Liga o motor com uma velocidade x, no sentido horário

        delay(rampTime / maxSpeed);
    }   

    delay(timeMotor);  // Tempo de movimento na velocidade máxima

    for (int speed = maxSpeed; speed >= minSpeed; speed--) {
        analogWrite(motor_RA_IN, speed); //Liga o motor com uma velocidade x, no sentido ant-horário

        analogWrite(motor_LB_IN, speed); //Liga o motor com uma velocidade x, no sentido ant-horário

        analogWrite(motor_LA_IP, speed); //Liga o motor com uma velocidade x, no sentido horário

        analogWrite(motor_RB_IP, speed); //Liga o motor com uma velocidade x, no sentido horário

        
        delay(rampTime / maxSpeed);
    }

}

//Função que fará o carrinho andar para Esquerda
void Esquerda(int timeMotor) {
    // MotorFrontLeft - MotorBackRight (Mesma direção) Ant-Horário
    // MotorFrontRight - MotorBackLeft (Mesma direção) Horário

    stopMotors();

     for (int speed = minSpeed; speed <= maxSpeed; speed++) {
        // Ativa os motores do lado direito (sentido ant-horário)
        analogWrite(motor_RA_IP, speed); // Desativa o motor do lado direito (horário)   

        analogWrite(motor_LB_IP, speed); // Desativa o motor do lado direito (horário)

        analogWrite(motor_LA_IN, speed); // Desativa o motor do lado esquerdo (ant-horário)

        analogWrite(motor_RB_IN, speed); // Desativa o motor do lado esquerdo (ant-horário)
        delay(rampTime / maxSpeed);
     }

    delay(timeMotor);  // Tempo de movimento na velocidade máxima

     for (int speed = maxSpeed; speed >= minSpeed; speed--) {
        // Ativa os motores do lado direito (sentido ant-horário)
        analogWrite(motor_RA_IP, speed); // Desativa o motor do lado direito (horário)

        analogWrite(motor_LB_IP, speed); // Desativa o motor do lado direito (horário)

        analogWrite(motor_LA_IN, speed); // Desativa o motor do lado esquerdo (ant-horário)

        analogWrite(motor_RB_IN, speed); // Desativa o motor do lado esquerdo (ant-horário)
        delay(rampTime / maxSpeed);
     }
}
/*
    Função: turnRight
    Descrição: Vira o carro para a direita.
*/

void turnRight(unsigned int timeMotor) {
    // Parar todos os motores antes de executar a virada
    stopMotors();


    for (int speed = minSpeed; speed <= maxSpeed; speed++) {
        analogWrite(motor_RA_IN, speed); // Ativa o motor do lado direito no sentido anti-horário
        analogWrite(motor_RB_IN, speed); // Ativa o motor do lado direito no sentido anti-horário
        delay(rampTime / maxSpeed);
    }

    delay(timeMotor); // Aguarda pelo tempo especificado antes de continuar

    for (int speed = maxSpeed; speed >= minSpeed; speed--) {
        analogWrite(motor_RA_IN, speed); // Ativa o motor do lado direito no sentido anti-horário
        analogWrite(motor_RB_IN, speed); // Ativa o motor do lado direito no sentido anti-horário
        delay(rampTime / maxSpeed);
    }
}

// /*
//     Função: turnLeft
//     Descrição: Vira o carro para a esquerda.
// */

void turnLeft(unsigned int timeMotor) {
    // Parar todos os motores antes de executar a virada
    stopMotors();

    for (int speed = minSpeed; speed <= maxSpeed; speed++) {
        analogWrite(motor_LA_IN, speed); // Ativa o motor do lado esquerdo no sentido anti-horário
        analogWrite(motor_LB_IN, speed); // Ativa o motor do lado esquerdo no sentido anti-horário
        delay(rampTime / maxSpeed);  
    }

    delay(timeMotor); // Aguarda pelo tempo especificado antes de continuar

    for (int speed = maxSpeed; speed >= minSpeed; speed--) {
        analogWrite(motor_LA_IN, speed); // Ativa o motor do lado esquerdo no sentido anti-horário
        analogWrite(motor_LB_IN, speed); // Ativa o motor do lado esquerdo no sentido anti-horário
        delay(rampTime / maxSpeed); 
    }
}


//Função que fará o carrinho andar para Sudoeste
void So(unsigned int timeMotor) {
  stopMotors();
    // Liga os motores adequadamente para mover para sudoeste
    for (int speed = minSpeed; speed <= maxSpeed; speed++) {
        analogWrite(motor_LA_IN, speed); // Ativa no sentido anti-horário
        analogWrite(motor_RB_IN, speed); // Ativa no sentido anti-horário
        delay(rampTime / maxSpeed);
    }

    delay(timeMotor); // Aguarda pelo tempo especificado antes de continuar

    for (int speed = maxSpeed; speed >= minSpeed; speed--) {
        analogWrite(motor_LA_IN, speed); // Ativa no sentido anti-horário
        analogWrite(motor_RB_IN, speed); // Ativa no sentido anti-horário
        delay(rampTime / maxSpeed);
    }
}


//Função que fará o carrinho andar para Nordeste
void Ne(unsigned int timeMotor){
    stopMotors();

    for (int speed = minSpeed; speed <= maxSpeed; speed++) {
        analogWrite(motor_LA_IP, speed); // Ativa no sentido anti-horário
        analogWrite(motor_RB_IP, speed); // Ativa no sentido anti-horário
    }

    delay(timeMotor); // Aguarda pelo tempo especificado antes de continuar

    for (int speed = maxSpeed; speed >= minSpeed; speed--) {
        analogWrite(motor_LA_IP, speed); //Liga o motor com uma velocidade x, no sentido horário
        analogWrite(motor_RB_IP, speed); //Liga o motor com uma velocidade x, no sentido horário
        delay(rampTime / maxSpeed);
    }
}

// //Função que fará o carrinho andar para Sudeste
// void Se(){

//     // Diagonal SE: 

//     // MotorFrontRight	- MotorBackLeft (Mesma direção) Ant-Horário

//     acelerar();

//     analogWrite(MotorFrontRightA, LOW); //Desliga o sentido horário do motor
//     digitalWrite(MotorFrontRightB, vel); //Liga o motor com uma velocidade x, no sentido ant-horário

//     analogWrite(MotorBackLeftA, LOW); //Desliga o sentido horário do motor
//     digitalWrite(MotorBackLeftB, vel); //Liga o motor com uma velocidade x, no sentido ant-horário

// }

// //Função que fará o carrinho andar para Noroeste
// void No(){
//     // Diagonal NO: 

//     // MotorFrontRight	- MotorBackLeft (Mesma direção) Horário

//     analogWrite(MotorFrontRightA, vel); //Desliga o sentido horário do motor
//     digitalWrite(MotorFrontRightB, LOW); //Liga o motor com uma velocidade x, no sentido ant-horário

//     analogWrite(MotorBackLeftA, vel); //Desliga o sentido horário do motor
//     digitalWrite(MotorBackLeftB, LOW); //Liga o motor com uma velocidade x, no sentido ant-horário
// }

void setup(){
    setMotors();
    setUltra();
}

void loop(){
    cm = 0.01723 * readUltrasonic();
    Ne
}