// ===================================
//            Bibliotecas
// ===================================

#include "Adafruit_TCS34725.h"
#include <Arduino.h> // Inclui a biblioteca do Arduino
#include <ServoEasing.hpp> // Inclui a biblioteca dos Servos
#include <Adafruit_NeoPixel.h> // Adiciona a biblioteca para controle de LEDs endereçáveis da Adafruit
#include <Adafruit_TCS34725.h> // Biblioteca dos Sensores de Cor
#include <Wire.h>
#include <cmath> // Adicione esta linha para usar sqrt

// ===================================
//            Defines
// ===================================

#define QTD_COLOR 10

#define SDA_PIN 21 // Substitua pelo pino SDA que você está usando
#define SCL_PIN 22 // Substitua pelo pino SCL que você está usando

/* Definição dos LEDs */
#define qtdeLeds 4 // Informa a quantidade de LEDs que serão ligados em cascata

/* Definição dos Servos */
#define servo_Pino_A 32 // Pino do Servo A
#define servo_Pino_B 33 // Pino do Servo B
#define servo_Pino_C 25 // Pino do Servo C
#define servo_Pino_D 26 // Pino do Servo D
#define servo_Pino_E 5 // Pino do Servo E
#define qtdServos 5 // Informa a quantidade de Servos que serão ligados em cascata

/* Definição do Arduino em geral */
#define D_in 16 // Nomeia o pino 16 do Arduino

#define pos0 0
#define pos1 0
#define pos2 50
#define pos3 30
#define pos4 90 // Posição inicial para Servo E

// ===================================
//            Constantes
// ===================================

Adafruit_TCS34725 tcsG = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X);
int calibR = 0, calibG = 0, calibB = 0;
int calibBR = 0, calibBG = 0, calibBB = 0;
int calibWR = 0, calibWG = 0, calibWB = 0;
int calibW = 0;
int margem = 50;
String ColorTcs;

/* Constantes dos servos */
const byte pinServos[qtdServos] = {servo_Pino_A, servo_Pino_B, servo_Pino_C, servo_Pino_D, servo_Pino_E};

/* Constantes de velocidade */
const byte velocityTurnMotorA = 0x32; // 50
const byte velocityTurnMotorB = 0x32; // 50
unsigned int timeMotor = 1000; // Tempo de ligar o Motor

/* Constantes para definição dos pinos dos motores */ 
const byte motor_A_IN = 4; // Motor A; Pino de entrada negativo do Motor A (Faz a roda girar no sentido anti-horário)
const byte motor_A_IP = 2; // Motor A; Pino de entrada positiva do Motor A (Faz a roda girar no sentido horário)
const byte motor_B_IN = 13; // Pino de entrada negativa do Motor B (Faz a roda girar no sentido anti-horário)
const byte motor_B_IP = 27; // Pino de entrada positiva do Motor B (Faz a roda girar no sentido horário)

// ===================================
//            Variáveis
// ===================================

byte velocityMotorA = 127; // Velocidade do Motor A
byte velocityMotorB = 127; // Velocidade do Motor B

String recebi;
ServoEasing servos[qtdServos]; // Ajustado para 5 servos

/* Variáveis de tempo */
unsigned int timeXV = 15;

/* Variável para o LED */
Adafruit_NeoPixel pixels(qtdeLeds, D_in); // Instancia o objeto "pixels", informando a quantidade e o pino de sinal
uint8_t R = 255, G = 255, B = 255;

// ===================================
//            Funções
// ===================================

void checkSensorsColor() {
    if (tcsG.begin()) {
        Serial.println("Sensor de cor encontrado");
    } else {
        Serial.println("Sensor de cor não encontrado");
    }
}

void calibratorColors() {
    delay(3000);
    uint16_t r, g, b, c;

    tcsG.getRawData(&r, &g, &b, &c);

    int rSum = 0, gSum = 0, bSum = 0, cSum = 0;

    for (int i = 0; i < QTD_COLOR; i++) {
        tcsG.getRawData(&r, &g, &b, &c);
        rSum += r;
        gSum += g;
        bSum += b;
        cSum += c;
    }

    r = rSum / QTD_COLOR;
    g = gSum / QTD_COLOR;
    b = bSum / QTD_COLOR;

    calibR = r / 2.26;
    calibG = g / 2.4;
    calibB = b / 2.4;
    calibBR = std::sqrt(r) * 4.3;
    calibBG = std::sqrt(g) * 4.3;
    calibBB = std::sqrt(b) * 4.3;
    calibWR = r / 1.155;
    calibWG = g / 1.155;
    calibWB = b / 1.155;
    calibW = (r + g + b) / 1.158;
}

void identifyColor(uint16_t rp, uint16_t gp, uint16_t bp, uint16_t cp, String &cor) {
    if (bp <= calibBB && gp <= calibBG && rp <= calibBR) {
        cor = "Preto";
    } else if (rp >= calibR && gp <= (calibG + margem) && bp <= (calibB + margem)) {
        cor = "Vermelho";
    } else if (gp >= calibG && rp <= (calibR + margem) && bp <= (calibB + margem) && gp > bp) {
        cor = "Verde";
    } else if (bp >= calibB && rp <= (calibR + margem) && gp <= (calibG + margem) && bp > gp) {
        cor = "Azul";
    } else if (rp >= calibWR && gp >= calibWG && bp >= calibWB && cp >= calibW) {
        cor = "Branco";
    } else {
        cor = "Desconhecida";
    }
}

void readTcs() {
    uint16_t r, g, b, c;
    int rSum = 0, gSum = 0, bSum = 0, cSum = 0;

    for (int i = 0; i < QTD_COLOR; i++) {
        tcsG.getRawData(&r, &g, &b, &c);
        rSum += r;
        gSum += g;
        bSum += b;
        cSum += c;
        delay(10);
    }

    r = rSum / QTD_COLOR;
    g = gSum / QTD_COLOR;
    b = bSum / QTD_COLOR;
    c = cSum / QTD_COLOR;

    identifyColor(r, g, b, c, ColorTcs);

    Serial.print("Cores de leitura R="); Serial.print(r);
    Serial.print(", G="); Serial.print(g);
    Serial.print(", B="); Serial.print(b);
    Serial.print(", C="); Serial.print(c);
    Serial.println("Cor detectada: " + ColorTcs);
}

/*
    Função: setServos
    Descrição: Configura todos os servos, definindo pinos, suavização, velocidade e posição inicial.
*/
void setServos() {
  for (int x = 0; x < qtdServos; x++) {
    pinMode(pinServos[x], OUTPUT); // Define pino servo e saída de energia 
    servos[x].attach(pinServos[x]); // Conecta o servo ao pino definido no array pinServos.
    servos[x].setEasingType(EASE_CUBIC_IN_OUT); // Define o tipo de suavização do movimento do servo
    servos[x].setSpeed(50); // Estabelece a velocidade do movimento do servo.
    servos[x].write(90); // Move o servo para a posição inicial de 90 graus.
    delay(timeXV); // Tempo de espera
  }
}

/*
    Função: acionarServo
    Descrição: Aciona um servomotor específico para um ângulo determinado.
*/
void acionarServo(int numeroServo, int angulo) { 
  if (numeroServo >= 0 && numeroServo < qtdServos) { 
    servos[numeroServo].write(angulo); // Move o servo para o ângulo especificado
  } else {
    Serial.println("Número de servo inválido!"); // Mensagem de erro
  }
}

/*
    Função: setup
    Descrição: Inicia o código apenas uma vez ao ligar o microcontrolador
*/
void setup() {
  Serial.begin(115200);
  setServos(); // Configura os servos
  delay(1000); // Espera 1 segundo para tudo ser carregado
  Serial.println("Servos Iniciados");

  // Acionando os servos nas posições iniciais
  acionarServo(0, pos0);
  delay(100);
  acionarServo(1, pos1);
  delay(100);
  acionarServo(2, pos2);
  delay(100);
  acionarServo(3, pos3);
  delay(100);
  acionarServo(4, pos4); // Aciona o Servo E na posição inicial
  delay(100);
  checkSensorsColor();
  calibratorColors();
}

/*
    Função: loop
    Descrição: Inicia o código e fica em loop
*/
void loop() {
  readTcs();
  if (Serial.available() > 0) { // Verifica se há dados disponíveis na Serial
    recebi = Serial.readStringUntil('\n'); // Lê até o enter ('\n')
    Serial.println(recebi); // Mostra na Serial o que foi digitado
    
    if (recebi.startsWith("M")) {
      recebi.remove(0, 1);
      int pos = recebi.toInt();
      acionarServo(3, pos); // Aciona o Servo D
      delay(20);
    } else if (recebi.startsWith("O")) {
      recebi.remove(0, 1);
      int pos = recebi.toInt();
      acionarServo(1, pos); // Aciona o Servo B
      delay(20);
    } else if (recebi.startsWith("C")) {
      recebi.remove(0, 1);
      int pos = recebi.toInt();
      acionarServo(2, pos); // Aciona o Servo C
      delay(20);
    } else if (recebi.startsWith("B")) {
      recebi.remove(0, 1);
      int pos = recebi.toInt();
      acionarServo(0, pos); // Aciona o Servo A
      delay(200);
    } else if (recebi.startsWith("G")) { // Comando para o Servo E
      recebi.remove(0, 1);
      int pos = recebi.toInt();
      acionarServo(4, pos); // Aciona o Servo E
      delay(200);
    }
  }
}
