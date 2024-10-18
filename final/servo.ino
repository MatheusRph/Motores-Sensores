// ===================================
//            Bibliotecas
// ===================================

#include <Arduino.h> //Inclui a biblioteca do arduino
//#include <analogWrite.h>
#include <ServoEasing.hpp> //Inclui a biblioteca dos Servos
#include <Adafruit_NeoPixel.h> // Adiciona a biblioteca para controle de LEDs endereçáveis da Adafruit
#include <Adafruit_TCS34725.h> // Biblioteca dos Sensores de Cor
#include <Wire.h>
#include <cmath> // Adicione esta linha para usar sqrt


// ===================================
//            Defines
// ===================================

#define SDA_PIN 21 // Substitua pelo pino SDA que você está usando
#define SCL_PIN 22 // Substitua pelo pino SCL que você está usando


/*Definição dos leds*/
#define qtdeLeds 4 //Informa a quantidade de LEDs que serão ligados em cascata

/*Definição dos Servos*/
#define servo_Pino_A 32 // Pino do Servo A
#define servo_Pino_B 33 // Pino do Servo B
#define servo_Pino_C 25 // Pino do Servo C
#define servo_Pino_D 26 // Pino do Servo D
#define servo_Pino_E 5  // Pino de Servo E

#define qtdServos 5 //Informa a quantidade de Servos que serão ligados em cascata

/*Definição do arduino em geral*/
#define D_in 16 //Nomeia o pino 6 do Arduino

// ===================================
//            Constantes
// ===================================

/*Constantes dos servos*/
const byte pinServos[qtdServos] = {servo_Pino_A, servo_Pino_B, servo_Pino_C, servo_Pino_D, servo_Pino_E};

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
const byte motor_A_IN = 4; //Motor A; // Pino de entrada negativo do Motor A (Faz a roda girar no sentido anti-horário)
const byte motor_A_IP = 2; //Motor A; // Pino de entrada positiva do Motor A (Faz a roda girar no sentido horário)

//Motor B;
const byte motor_B_IN = 13;  // Pino de entrada negativa do Motor B (Faz a roda girar no sentido anti-horário)
const byte motor_B_IP = 27;  //Pino de entrada positiva do Motor B (Faz a roda girar no sentido horário)

// ===================================
//            Variáveis
// ===================================


// Variáveis para definição da velocidade dos motores
byte velocityMotorA = 127; // Velocidade do Motor A
byte velocityMotorB = 127; // Velocidade do Motor B

/* Variáveis de configuração dos sensores de cor */
Adafruit_TCS34725 tcsF = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X); // Sensor na frente
Adafruit_TCS34725 tcsR = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X); // Sensor na direita
Adafruit_TCS34725 tcsL = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X); // Sensor na esquerda

int calibR = 0;
int calibG = 0;
int calibB = 0;
int calibBR = 0;
int calibBG = 0;
int calibBB = 0;
int calibWR = 0;
int calibWG = 0;
int calibWB = 0;
int calibW = 0;
int calibBL = 0;
int margem = 50;

String colorF; // Variável para armazenar a cor lida pelo sensor da frente
String colorL; // Variável para armazenar a cor lida pelo sensor esquerdo
String colorR; // Variável para armazenar a cor lida pelo sensor direito

/*Variáveis sobre servo*/
ServoEasing servos[qtdServos], servo_A, servo_B, servo_C, servo_D, servo_E;

/*Variáveis de tempo*/
unsigned int timeXV = 14 +1;

/*Variável para o led*/
Adafruit_NeoPixel pixels(qtdeLeds, D_in); //Instancia o objeto "pixels", informando a quantidade e o pino de sinal
uint8_t R = 255, G = 255, B = 255;

// ===================================
//            Funções
// ===================================

/*
    Função: setServos
    Descrição: Configura todos os servos, definindo pinos, suavização, velocidade e posição inicial.
*/
void setServos(){
  for (int x = 0; x < qtdServos; x++){
    pinMode(pinServos[x], OUTPUT);//Define pino servo e saída de energia 
    servos[x].attach(pinServos[x]);// Conecta o servo ao pino definido no array pinServos.
    servos[x].setEasingType(EASE_CUBIC_IN_OUT);// Define o tipo de suavização do movimento do servo
    servos[x].setSpeed(70);//Estabelece a velocidade do movimento do servo.
   // servos[x].write(90);//Move o servo para a posição inicial de 90 .
    delay(timeXV);//Tempo de espera de 15 segundos
    }
  servos[32].write(90); // Move o servo para o ângulo especificado
  servos[33].write(50); // Move o servo para o ângulo especificado
  servos[25].write(170); // Move o servo para o ângulo especificado
  servos[26].write(90); // Move o servo para o ângulo especificado
  servos[5].write(0); // Move o servo para o ângulo especificado
}

/*
    Função: acionarServo
    Descrição: Aciona um servomotor específico para um ângulo determinado.
*/
void acionarServo(int numeroServo, int angulo) { 
    // Verifica se o número do servo está dentro do intervalo válido
    if (numeroServo >= 0 && numeroServo < qtdServos) { 
    servos[numeroServo].write(angulo); // Move o servo para o ângulo especificado
    } else { // Caso o número do servo não seja válido
    Serial.println("Número de servo inválido!"); // Imprime mensagem de erro no console
    }
}

/*
    Função: servoEsquerda
    Descrição: Gira o servo especificado para a esquerda e retorna à posição inicial.
*/
void servoEsquerda(int numeroServo) { 
    // Verifica se o número do servo está dentro do intervalo válido
    if (numeroServo >= 0 && numeroServo < qtdServos) {
    // Move o servo de 90 graus até 180 graus
    for (int pos = 90; pos <= 180; pos += 1) { 
        servos[numeroServo].write(pos); // Define a posição do servo
        delay(timeXV); // Aguarda um tempo definido antes de mudar a posição
    }
    // Move o servo de 180 graus de volta para 90 graus
    for (int pos = 180; pos >= 90; pos -= 1) { 
        servos[numeroServo].write(pos); // Define a posição do servo
        delay(timeXV); // Aguarda um tempo definido antes de mudar a posição
    }
    } else { // Caso o número do servo não seja válido
        Serial.println("Número de servo inválido!"); // Imprime mensagem de erro no console
  }
}


/*
    Função: servoDireita
    Descrição: Gira o servo especificado para a direita e retorna à posição inicial.
*/
void servoDireita(int numeroServo) { 
  // Verifica se o número do servo está dentro do intervalo válido
  if (numeroServo >= 0 && numeroServo < qtdServos) {
    // Move o servo de 90 graus até 0 graus
    for (int pos = 90; pos >= 0; pos -= 1) { 
      servos[numeroServo].write(pos); // Define a posição do servo
      delay(timeXV); // Aguarda um tempo definido antes de mudar a posição
    }
    // Move o servo de 0 graus de volta para 90 graus
    for (int pos = 0; pos <= 90; pos += 1) { 
      servos[numeroServo].write(pos); // Define a posição do servo
      delay(timeXV); // Aguarda um tempo definido antes de mudar a posição
    }
  } else { // Caso o número do servo não seja válido
    Serial.println("Número de servo inválido!"); // Imprime mensagem de erro no console
  }
}

/*
    Função: showColors
    Descrição: Exibe cores nos LEDs RGB.
*/
void showColors(uint8_t R, uint8_t G, uint8_t B) {
  for (int i = 0; i < qtdeLeds; i++) { // Para cada LED
    pixels.setPixelColor(i, pixels.Color(R, G, B)); // Liga LED com a cor especificada
    // pixels.setPixelColor(i, pixels.Color(random(255), random(255), random(255))); // Liga LED com cor aleatória
    pixels.show(); // Atualiza a exibição dos LEDs
    delay(timeXV); // Aguarda um tempo definido
    pixels.clear(); // Desliga todos os LEDs
    // pixels.setPixelColor(i, pixels.Color(0, 0, 0)); // Desliga LED RGB
    // pixels.show(); // Atualiza a exibição para desligar LEDs
    delay(timeXV); // Aguarda um tempo definido
  }
}

/*
    Função: setup
    Descrição: Inicia o código apenas uma vez ao ligar o microcontrolador
*/
void setup() {
  Serial.begin(115200);
  //pinMode(D_in, OUTPUT); //Configura o pino 16 como saída
  //pixels.begin(); //Inicia o objeto "pixels"
  //pixels.clear(); //desliga todos os LEDs
  //setMotors(); //configura os motores
  setServos(); //configura os servos
  delay(1000); //espera 1 segundo para tudo ser carregado
}

/*
    Função: loop
    Descrição: Inicia o código e fica em loop
*/
void loop() {
    acionarServo(1, 0);
    delay(1000);
    acionarServo(2, 80);
}
  /*//// Exemplos de como usar a função acionarServo:
  //acionarServo(0, 90);  // Aciona o primeiro servo para 90 graus
  //delay(1000);
  //acionarServo(2, 180); // Aciona o terceiro servo para 180 graus
  //delay(1000);
  //moveForward(1000);
  //stopCar(1000);
  //stopMotors();
  //delay(1000);
  //moveBackward(1000);
  //stopCar(1000);
  //turnLeft();
  //turnRight();
  //servo_A.easeTo(90, 0);
  //delay(20);
  //servo_A.easeTo(0, 90);
  //delay(20);
  //servo_B.easeTo(90, 0);
  //delay(20);
  //servo_B.easeTo(0, 90);
  //delay(20);
//}
*/

// ===================================
//            Bibliotecas
// ===================================

#include <Arduino.h>
#include <ServoEasing.hpp>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_TCS34725.h>
#include <Wire.h>
#include <cmath>

// ===================================
//            Defines
// ===================================

#define SDA_PIN 21
#define SCL_PIN 22

/*Definição dos leds*/
#define qtdeLeds 4 

/*Definição dos Servos*/
#define servo_Pino_A 32 // Pino do Servo A
#define servo_Pino_B 33 // Pino do Servo B
#define servo_Pino_C 25 // Pino do Servo C
#define servo_Pino_D 26 // Pino do Servo D
#define servo_Pino_E 5 // Pino do Servo E (Novo Servo)
#define qtdServos 5 // Aumenta a quantidade de Servos para 5

/*Definição do arduino em geral*/
#define D_in 16 

#define pos0 90
#define pos1 50
#define pos2 170
#define pos3 90
#define pos4 0 // Posição inicial para Servo E

// ===================================
//            Constantes
// ===================================

const byte pinServos[qtdServos] = {servo_Pino_A, servo_Pino_B, servo_Pino_C, servo_Pino_D, servo_Pino_E};

/*Constantes de velocidade*/
const byte velocityTurnMotorA = 0x32; 
const byte velocityTurnMotorB = 0x32; 
unsigned int timeMotor = 1000;  

/*Constantes para definição dos pinos dos motores*/ 
const byte motor_A_IN = 4; 
const byte motor_A_IP = 2; 
const byte motor_B_IN = 13;  
const byte motor_B_IP = 27;  

// ===================================
//            Variáveis
// ===================================

byte velocityMotorA = 127; 
byte velocityMotorB = 127; 

String recebi;
ServoEasing servos[qtdServos]; // Ajustado para 5 servos

/*Variáveis de tempo*/
unsigned int timeXV = 14 + 1;

/*Variável para o led*/
Adafruit_NeoPixel pixels(qtdeLeds, D_in); 
uint8_t R = 255, G = 255, B = 255;

// ===================================
//            Funções
// ===================================

void setServos() {
  for (int x = 0; x < qtdServos; x++) {
    pinMode(pinServos[x], OUTPUT);
    servos[x].attach(pinServos[x]);
    servos[x].setEasingType(EASE_CUBIC_IN_OUT);
    servos[x].setSpeed(70);
    servos[x].write(pos4); // Usa a nova posição inicial para o Servo E
    delay(timeXV);
  }
}

void acionarServo(int numeroServo, int angulo) { 
  if (numeroServo >= 0 && numeroServo < qtdServos) { 
    servos[numeroServo].write(angulo);
  } else { 
    Serial.println("Número de servo inválido!");
  }
}

// (Outras funções como servoEsquerda, servoDireita, showColors, setup e loop permanecem as mesmas, exceto onde o servo é acionado)

void setup() {
  Serial.begin(115200);
  setServos(); 
  delay(1000); 
  Serial.println("Servos Iniciados");
  delay(1000);
  acionarServo(0, 90); // Servo A inicia em 90 graus
  delay(5000);
  acionarServo(1, pos1);
  delay(5000);
  acionarServo(2, pos2);
  delay(5000);
  acionarServo(3, pos3);
  delay(5000);
  acionarServo(4, pos4); // Aciona o Servo E na posição inicial
  delay(5000);
}

void loop() {
  acionarServo(0, 90); // Servo A inicia em 90 graus
  delay(5000);
  if (Serial.available() > 0) {
    recebi = Serial.readStringUntil('\n');
    Serial.println(recebi);
    if (recebi.startsWith("G")) {
      recebi.remove(0, 1);
      int pos = recebi.toInt();
      acionarServo(3, pos);
      delay(20);
    } else if (recebi.startsWith("M")) {
      recebi.remove(0, 1);
      int pos = recebi.toInt();
      acionarServo(1, pos);
      delay(20);
    } else if (recebi.startsWith("P")) {
      recebi.remove(0, 1);
      int pos = recebi.toInt();
      acionarServo(2, pos);
      delay(20);
    } else if (recebi.startsWith("C")) {
      recebi.remove(0, 1);
      int pos = recebi.toInt();
      acionarServo(0, pos);
      delay(200);
    } else if (recebi.startsWith("E")) { // Comando para o Servo E
      recebi.remove(0, 1);
      int pos = recebi.toInt();
      acionarServo(4, pos); // Aciona o Servo E
      delay(200);
    }
  }
}
