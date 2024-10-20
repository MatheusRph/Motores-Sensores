/*
    Color = X
    Ultra = Check
    LDR = CHECK
    Servo = X
    Motors = CHECK
    

*/

// ===================================
//            Bibliotecas
// ===================================

#include <Arduino.h> //Inclui a biblioteca do arduino
//#include <analogWrite.h>
#include <ServoEasing.hpp> //Inclui a biblioteca dos Servos
#include <Wire.h>
#include <cmath> // Adicione esta linha para usar sqrt
#include "Adafruit_TCS34725.h"


// ===================================
//            Defines
// ===================================

/*Definição sensor de cor*/
#define QTD_COLOR 10

/*Definição sensor ultrassônico*/
#define triggerPin 19
#define echoPin 36
#define echoPin2 18
#define QTD_ULTRA 2
#define QTD_RULTRA 5

//Definição do Pino do LDR
#define pinLDR0 36
#define pinLDR1 39  
#define pinLDR2 34
#define QTD_LDR 3

// /* Definição dos Pinos do seguidor de linha */
// #define pinLine1 36 // Pino do seguidor de linha A
// #define pinLine2 35 // Pino do seguidor de linha B
// #define pinLine3 39 // Pino do seguidor de linha C
// #define qtdLine 3    // Informa a quantidade de sensores preto e branco serão usados

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

// Enum para os servos
enum ServoNames {
    B = 0,
    O = 1,
    C = 2,
    M = 3,
    G = 4,
    TOTAL_SERVOS // Adiciona um total para facilitar futuras validações
};

const byte pinUltra[] ={echoPin, echoPin2};

const byte pinLDRS[] = {pinLDR0, pinLDR1, pinLDR2};
byte ColorLDR[] = {0,0,0};
const int QTD_LEITURA_LDR = 100;

const int dAG = 25;

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

int counter = 0;


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

/*Matriz para fazer um for automatizando o processo*/
// const byte pinLine[qtdLine] = {pinLine1, pinLine2, pinLine3}; // Matriz de pinos

// ===================================
//            Variáveis
// ===================================
int rampTime = 1000;
int minSpeed = 65;
int maxSpeed = 85;

/*Variáveis sobre servo*/
ServoEasing servos[qtdServos], servo_A, servo_B, servo_C, servo_D, servo_E;

// Variáveis para definição da velocidade dos motores
byte velocityMotorR = 200; // Velocidade dos motores do lado Direito
byte velocityMotorL = 200; // Velocidade do Motor do lado esquerdo

/*Variáveis de tempo*/
unsigned int timeXV = 14 +1;

/*Variáveis para o sensor ultrassônico*/
float cm = 0, acumulado = 0, mediaMovelUltrassom[] = {0, 0}, mediaMovelLDR[] = {0,0,0};

/*Variáveis pra o sensor de cor*/
Adafruit_TCS34725 tcsG = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X); //240 1x

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

String ColorTcs;

// ===================================
//            Funções
// ===================================

void checkSensorsColor() {
    if (tcsG.begin()) {
        Serial.println("Sensor da garra encontrado");
    } else {
        Serial.println("Sensor da garra não encontrado");
    }

    // if (tcs2.begin()) {
    //     Serial.println("Sensor trás encontrado");
    // } else {
    //     Serial.println("Sensor trás não encontrado");
    // }
}

void calibratorColors() {
    delay(3000);
    uint16_t r, g, b, c;

    tcsG.getRawData(&r, &g, &b, &c);

    if (c < 750){
      tcsG = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_60MS, TCS34725_GAIN_4X); //240 1x
    }


    int rSum = 0, gSum = 0, bSum = 0, cSum = 0;

    for (int i = 0; i < QTD_COLOR; i++) {
        tcsG.getRawData(&r, &g, &b, &c);
        rSum += r;
        gSum += g;
        bSum += b;
        cSum += c;
    }

    r = rSum/QTD_COLOR;
    g = gSum/QTD_COLOR;
    b = bSum/QTD_COLOR;
    c = cSum/QTD_COLOR;
       
    // Serial.print("Não Calibradas: R=");
    // Serial.print(r);
    // Serial.print(", G=");
    // Serial.print(g);
    // Serial.print(", B=");
    // Serial.print(b);
    // Serial.print(", C=");
    // Serial.print(c);
  
    delay(100);
    calibR = r / 2.26;
    calibG = g / 2.4;
    calibB = b / 2.4;
    calibBR = std::sqrt(r) * 4.3;
    calibBG = std::sqrt(g) * 4.3;
    calibBB = std::sqrt(b) * 4.3;
    calibBB = std::sqrt(c) * 4.3;
    calibWR = r / 1.155; //1.178 //1.152 //1.141
    calibWG = g / 1.155; //1.178
    calibWB = b / 1.155; //1.178
    calibW = (r + b + g) / 1.158;

    // Serial.println("Cores calibradas.");
    // Serial.print("Calibrações: R=");
    // Serial.print(calibR);
    // Serial.print(", G=");
    // Serial.print(calibG);
    // Serial.print(", B=");
    // Serial.print(calibB);
    // Serial.print(", BR=");
    // Serial.print(calibBR);
    // Serial.print(", BG=");
    // Serial.print(calibBG);
    // Serial.print(", BB=");
    // Serial.print(calibBB);
    // Serial.print(", WR=");
    // Serial.print(calibWR);
    // Serial.print(", WG=");
    // Serial.print(calibWG);
    // Serial.print(", WB=");
    // Serial.println(calibWB);
    // Serial.print(", W=");
    // Serial.println(calibW);
}

void identifyColor(uint16_t rp, uint16_t gp, uint16_t bp, uint16_t cp, String &cor) {
    delay(1);

    if (bp <= (calibBB) &&
        gp <= (calibBG) &&
        rp <= (calibBR)) {
        cor = "Preto"; // Cor detectada: Preto
    } else if (rp >= calibR && gp <= (calibG + margem) && bp <= (calibB + margem)) {
        cor = "Vermelho"; // Cor detectada: Vermelho
    } else if (gp >= calibG && rp <= (calibR + margem) && bp <= (calibB + margem) && gp > bp) {
        cor = "Verde"; // Cor detectada: Verde
    } else if (bp >= calibB && rp <= (calibR + margem) && gp <= (calibG + margem) && bp > gp) {
        cor = "Azul"; // Cor detectada: Azul
    } else if (rp >= calibWR && gp >= calibWG && bp >= calibWB && cp >= calibW) {                                                       
        cor = "Branco"; // Cor detectada: Branco
    } else {
        cor = "Desconhecida"; // Cor desconhecida
    }
}

void readTcs() {
    uint16_t r, g, b, c;
    int rSum = 0, gSum = 0, bSum = 0, cSum = 0;

    // Fazer 10 leituras e acumular os resultados
    for (int i = 0; i < QTD_COLOR; i++) {
        tcsG.getRawData(&r, &g, &b, &c);
        rSum += r;
        gSum += g;
        bSum += b;
        cSum += c;
        delay(10); // Delay opcional entre as leituras
    }

    // Calcular a média
    r = rSum / QTD_COLOR;
    g = gSum / QTD_COLOR;
    b = bSum / QTD_COLOR;
    c = cSum / QTD_COLOR;

    // Identificar a cor com a média
    identifyColor(r, g, b, c, ColorTcs);

    // Imprimir resultados
    Serial.print("Cores de leitura R="); Serial.print(r);
    Serial.print(", G="); Serial.print(g);
    Serial.print(", B="); Serial.print(b);
    Serial.print(", C="); Serial.print(c);
    Serial.println("Cor detectada: " + ColorTcs);
}

void setServos(){
  for (int x = 0; x < qtdServos; x++){
    pinMode(pinServos[x], OUTPUT);//Define pino servo e saída de energia 
    servos[x].attach(pinServos[x]);// Conecta o servo ao pino definido no array pinServos.
    servos[x].setEasingType(EASE_CUBIC_IN_OUT);// Define o tipo de suavização do movimento do servo
    servos[x].setSpeed(40);//Estabelece a velocidade do movimento do servo.
   // servos[x].write(90);//Move o servo para a posição inicial de 90 .
    delay(timeXV);//Tempo de espera de 15 segundos
    }
//   servos[32].write(90); // Move o servo para o ângulo especificado
//   servos[33].write(50); // Move o servo para o ângulo especificado
//   servos[25].write(170); // Move o servo para o ângulo especificado
//   servos[26].write(90); // Move o servo para o ângulo especificado
//   servos[5].write(0); // Move o servo para o ângulo especificado
}

/*
    Função: acionarServo
    Descrição: Aciona um servomotor específico para um ângulo determinado.
*/
void acionarServo(ServoNames nomeServo, int angulo) {
     int numeroServo = static_cast<int>(nomeServo); // Converte para inteiro
    // Verifica se o número do servo está dentro do intervalo válido
    if (numeroServo >= 0 && numeroServo < qtdServos) {
        int anguloAtual = servos[numeroServo].read(); // Lê o ângulo atual do servo
        int incremento = (angulo > anguloAtual) ? 1 : -1; // Define o incremento

        for (int n = anguloAtual; n != angulo; n += incremento) {
            servos[numeroServo].write(n); // Move o servo para o ângulo especificado
            delay(20);
        }
        
        servos[numeroServo].write(angulo); // Garante que o servo chegue ao ângulo final
    } else { // Caso o número do servo não seja válido
        Serial.println("Número de servo inválido!"); // Imprime mensagem de erro no console
    }
    delay(400);
}

void servo360(int angulo, unsigned int time) {
    //servo360(0, 86, 1900); Gira em 90 graus para a direita
    //servo360(0, 100, 1900); Gira em 90 graus para a esquerda
  if (0 >= 0 && 0 < qtdServos) {
    servos[0].write(angulo); // Aciona o servo no ângulo desejado
    delay(time); // Espera o tempo especificado
    servos[0].write(90); // Retorna o servo para a posição neutra
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
    Função: setLine
    Descrição: Configura os pinos do seguidor de linha
*/
/*
void setLine() {
    for (int x = 0; x < qtdLine; x++) {
        pinMode(pinLine[x], INPUT); // Define pino como entrada
        delay(timeXV); // Tempo de espera
    }
}
*/

/*
    Função: readPin
    Descrição: Le o pino escolhido do seguidor de linha
*/
// int readPin(int pino) { // Change return type to int
//     return digitalRead(pinLine[pino]);
//     delay(10);
// }

/*
    Função: setUltra
    Descrição: Configura os pinos do sensor ultrassônico
*/
// void setUltra(){
//     pinMode(triggerPin, OUTPUT); // Clear the trigger
//     for (int x = 0; x < QTD_ULTRA; x++){
//         pinMode(pinUltra[x], INPUT);
//     }
// }

void setUltra(){
    for( int x = 0; x < QTD_LDR; x++){
        pinMode(pinUltra[x], INPUT);
        delay(1);
    }
    pinMode(triggerPin, OUTPUT); // Clear the trigger
}

/*
    Função: readUltrasonicDistance
    Descrição: Se os valores do pino echo do sensor ultrassônico 
*/
long readUltrasonicDistance(int echo)
{
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    return pulseIn(echo, HIGH);
}
/*
float mediaUltrassom() {
    acumulado = 0;
    for (int i = 0; i < 10; i++) {
        cm = 0.01723 * readUltrasonicDistance() ; // Converte a duração para centímetros
        acumulado += cm; // Soma o valor lido
        //Serial.println(cm); // Imprime o valor atual
        delay(10); // Aguarda 1 segundo entre as leituras
    }
    mediaMovelUltrassom = acumulado / 10.0; // Retorna a média
    Serial.print("Distancia em cm: ");
    Serial.println(mediaMovelUltrassom); 
    return mediaMovelUltrassom;
}
*/
void setLDR(){
    for( int x = 0; x < QTD_LDR; x++){
        pinMode(pinLDRS[x], INPUT);
        delay(1);
    }
  
}

//////////////////////////////////////////////////////
// Função que calcula a média da leitura do ultrassom
void mediaUltrassomTask(void *pvParameters) {
    while (true) {
        for (int x = 0; x < QTD_ULTRA; x++){
            float acumulado = 0;
            for (int i = 0; i < QTD_RULTRA; i++) {
                cm = 0.01723 * readUltrasonicDistance(pinUltra[x]); // Converte a duração para centímetros
                acumulado += cm; // Soma o valor lido
                delay(15); // Aguarda um pouco entre as leituras
            }
            mediaMovelUltrassom[x] = acumulado / QTD_RULTRA; // Retorna a média
            Serial.print("Distancia em cm ");
            Serial.print(x);
            Serial.print(" : ");
            Serial.println(mediaMovelLDR[x]); 
            if (mediaMovelUltrassom[x] >= 500){
                mediaMovelUltrassom[x] = 500;
            }
            vTaskDelay(1000 / portTICK_PERIOD_MS); // Aguarda 1 segundo antes da próxima execução
        }
    }
}

// Função que calcula a média da leitura do LDR
void mediaLDRTask(void *pvParameters) {
   while (true) {
        for (int x = 0; x < QTD_LDR; x++){
            double acumulo = 0;
            for (int i = 0; i < QTD_LEITURA_LDR; i++) {
                int valueLDR = analogRead(pinLDRS[x]);
                acumulo += valueLDR; // Soma o valor lido
                delay(1); // Aguarda um pouco entre as leituras
            }
            mediaMovelLDR[x] = acumulo / QTD_LEITURA_LDR; // Retorna a média
            Serial.print("valor LDR: ");
            Serial.print(x);
            Serial.print(" : ");
            Serial.println(mediaMovelLDR[x]);
            delay(1);
        }
        ColorsLDR();
        vTaskDelay(500 / portTICK_PERIOD_MS); // Aguarda 1 segundo antes da próxima execução
  }
}

void ColorsLDR(){
        for (int x = 0; x < QTD_LDR; x++){
            if (mediaMovelLDR[x] >= 3940){
                ColorLDR[x] = 1;
            } else if (mediaMovelLDR[x] <= 3900) {
                ColorLDR[x] = 0;
            }
        }
}
/*
double mediaLDR() {
    double acumulo = 0;
    for (int i = 0; i < 100; i++) {
        int valueLDR = analogRead(pinLDR);
        delay(1);
         acumulo += valueLDR; // Soma o valor lido
        //Serial.println(cm); // Imprime o valor atual
        delay(10); // Aguarda 1 segundo entre as leituras
    }
    mediaMovelLDR = acumulo / 100.0; // Retorna a média
    Serial.printf("Intensidade LDR: %.2f\n", mediaMovelLDR);
    //Serial.println("Intesidade LDR: ",mediaMovelLDR); 
    return mediaMovelLDR;
}
*/
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
void moveBackward(/*unsigned int timeMotor*/) {

    //stopMotors();
  // Mover carro para trás
    for (int speed = minSpeed; speed <= maxSpeed; speed++) {
        analogWrite(motor_RB_IN, speed); // Ativa o motor B no sentido ant-horário com a velocidade definida
        analogWrite(motor_LB_IN, speed); // Ativa o motor B no sentido ant-horário com a velocidade definida
        analogWrite(motor_RA_IN, speed); // Ativa o motor B no sentido ant-horário com a velocidade definida
        analogWrite(motor_LA_IN, speed); // Ativa o motor B no sentido ant-horário com a velocidade definida
        delay(rampTime / maxSpeed);

    }

   // delay(timeMotor); // Aguarda pelo tempo especificado (timeMotor) antes de continuar

    for (int speed = maxSpeed; speed >= minSpeed; speed--) {
        analogWrite(motor_RB_IN, speed); // Ativa o motor B no sentido ant-horário com a velocidade definida
        analogWrite(motor_LB_IN, speed); // Ativa o motor B no sentido ant-horário com a velocidade definida
        analogWrite(motor_RA_IN, speed); // Ativa o motor B no sentido ant-horário com a velocidade definida
        analogWrite(motor_LA_IN, speed); // Ativa o motor B no sentido ant-horário com a velocidade definida
        delay(rampTime / maxSpeed);
    }
}

/*
    Função: moveForward
    Descrição: Move o carro para frente por um tempo determinado.
*/
void moveForward(/*unsigned int timeMotor*/) {

    //stopMotors();
    // Mover carro para frente
    for (int speed = minSpeed; speed <= maxSpeed; speed++) {
        analogWrite(motor_RA_IP, speed); // Ativa o motor A no sentido horário com a velocidade definida
        analogWrite(motor_LA_IP, speed); // Desativa o motor A no sentido horário
        analogWrite(motor_RB_IP, speed); // Ativa o motor B no sentido ant-horário com a velocidade definida
        analogWrite(motor_LB_IP, speed); // Ativa o motor B no sentido ant-horário com a velocidade definida

        delay(rampTime / maxSpeed);
    }

    //delay(timeMotor); // Aguarda pelo tempo especificado (timeMotor) antes de continuar

    for (int speed = maxSpeed; speed >= minSpeed; speed--) {
        analogWrite(motor_RA_IP, speed); // Ativa o motor A no sentido horário com a velocidade definida
        analogWrite(motor_LA_IP, speed); // Desativa o motor A no sentido horário
        analogWrite(motor_RB_IP, speed); // Ativa o motor B no sentido ant-horário com a velocidade definida
        analogWrite(motor_LB_IP, speed); // Ativa o motor B no sentido ant-horário com a velocidade definida

        delay(rampTime / maxSpeed);
    }
}

void Direita (/*unsigned int timeMotor*/) {
// MotorFrontLeft - MotorBackRight (Mesma direção) Horário
// MotorFrontRight	- MotorBackLeft (Mesma direção) Ant-Horário

    //stopMotors();

    for (int speed = minSpeed; speed <= maxSpeed; speed++) {
        analogWrite(motor_RA_IN, speed); //Liga o motor com uma velocidade x, no sentido ant-horário

        analogWrite(motor_LB_IN, speed); //Liga o motor com uma velocidade x, no sentido ant-horário
        
        analogWrite(motor_LA_IP, speed); //Liga o motor com uma velocidade x, no sentido horário

        analogWrite(motor_RB_IP, speed); //Liga o motor com uma velocidade x, no sentido horário

        delay(rampTime / maxSpeed);
    }   

    //delay(timeMotor);  // Tempo de movimento na velocidade máxima

    for (int speed = maxSpeed; speed >= minSpeed; speed--) {
        analogWrite(motor_RA_IN, speed); //Liga o motor com uma velocidade x, no sentido ant-horário

        analogWrite(motor_LB_IN, speed); //Liga o motor com uma velocidade x, no sentido ant-horário

        analogWrite(motor_LA_IP, speed); //Liga o motor com uma velocidade x, no sentido horário

        analogWrite(motor_RB_IP, speed); //Liga o motor com uma velocidade x, no sentido horário

        delay(rampTime / maxSpeed);
    }

}

//Função que fará o carrinho andar para Esquerda
void Esquerda(/*unsigned int timeMotor*/) {
    // MotorFrontLeft - MotorBackRight (Mesma direção) Ant-Horário
    // MotorFrontRight - MotorBackLeft (Mesma direção) Horário

    //stopMotors();

     for (int speed = minSpeed; speed <= maxSpeed; speed++) {
        // Ativa os motores do lado direito (sentido ant-horário)
        analogWrite(motor_RA_IP, speed); // Desativa o motor do lado direito (horário)   

        analogWrite(motor_LB_IP, speed); // Desativa o motor do lado direito (horário)

        analogWrite(motor_LA_IN, speed); // Desativa o motor do lado esquerdo (ant-horário)

        analogWrite(motor_RB_IN, speed); // Desativa o motor do lado esquerdo (ant-horário)
        delay(rampTime / maxSpeed);
     }

    //delay(timeMotor);  // Tempo de movimento na velocidade máxima

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

void turnRight(/*unsigned int timeMotor*/) {
    // Parar todos os motores antes de executar a virada
    //stopMotors();


    for (int speed = minSpeed; speed <= maxSpeed; speed++) {
        analogWrite(motor_RA_IN, speed); // Ativa o motor do lado direito no sentido anti-horário
        analogWrite(motor_RB_IN, speed); // Ativa o motor do lado direito no sentido anti-horário
        delay(rampTime / maxSpeed);
    }

   // delay(timeMotor); // Aguarda pelo tempo especificado antes de continuar

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

void turnLeft(/*unsigned int timeMotor*/) {
    // Parar todos os motores antes de executar a virada
    //stopMotors();

    for (int speed = minSpeed; speed <= maxSpeed; speed++) {
        analogWrite(motor_LA_IN, speed); // Ativa o motor do lado esquerdo no sentido anti-horário
        analogWrite(motor_LB_IN, speed); // Ativa o motor do lado esquerdo no sentido anti-horário
        delay(rampTime / maxSpeed);  
    }

    //delay(timeMotor); // Aguarda pelo tempo especificado antes de continuar

    for (int speed = maxSpeed; speed >= minSpeed; speed--) {
        analogWrite(motor_LA_IN, speed); // Ativa o motor do lado esquerdo no sentido anti-horário
        analogWrite(motor_LB_IN, speed); // Ativa o motor do lado esquerdo no sentido anti-horário
        delay(rampTime / maxSpeed); 
    }
}

void gEd(/*unsigned int timeMotor*/){
    //stopMotors();

    for (int speed = minSpeed; speed <= maxSpeed; speed++) {
        analogWrite(motor_LA_IP, speed); // Ativa o motor do lado esquerdo no sentido anti-horário

        analogWrite(motor_LB_IP, speed); // Ativa o motor do lado esquerdo no sentido anti-horário

        analogWrite(motor_RA_IN, speed); // Ativa o motor do lado esquerdo no sentido anti-horário
        
        analogWrite(motor_RB_IN, speed); // Ativa o motor do lado esquerdo no sentido anti-horário

        delay(rampTime / maxSpeed);  
    }

   // delay(timeMotor); // Aguarda pelo tempo especificado antes de continuar

    for (int speed = maxSpeed; speed >= minSpeed; speed--) {
        analogWrite(motor_LA_IP, speed); // Ativa o motor do lado esquerdo no sentido anti-horário

        analogWrite(motor_LB_IP, speed); // Ativa o motor do lado esquerdo no sentido anti-horário

        analogWrite(motor_RA_IN, speed); // Ativa o motor do lado esquerdo no sentido anti-horário
        
        analogWrite(motor_RB_IN, speed); // Ativa o motor do lado esquerdo no sentido anti-horário
        delay(rampTime / maxSpeed); 
    }
}

void gEe(/*unsigned int timeMotor*/){
    //stopMotors();

    for (int speed = minSpeed; speed <= maxSpeed; speed++) {
        analogWrite(motor_LA_IN, speed); // Ativa o motor do lado esquerdo no sentido anti-horário

        analogWrite(motor_LB_IN, speed); // Ativa o motor do lado esquerdo no sentido anti-horário

        analogWrite(motor_RA_IP, speed); // Ativa o motor do lado esquerdo no sentido anti-horário
        
        analogWrite(motor_RB_IP, speed); // Ativa o motor do lado esquerdo no sentido anti-horário

        delay(rampTime / maxSpeed);  
    }

    //delay(timeMotor); // Aguarda pelo tempo especificado antes de continuar

    for (int speed = maxSpeed; speed >= minSpeed; speed--) {
        analogWrite(motor_LA_IN, speed); // Ativa o motor do lado esquerdo no sentido anti-horário

        analogWrite(motor_LB_IN, speed); // Ativa o motor do lado esquerdo no sentido anti-horário

        analogWrite(motor_RA_IP, speed); // Ativa o motor do lado esquerdo no sentido anti-horário
        
        analogWrite(motor_RB_IP, speed); // Ativa o motor do lado esquerdo no sentido anti-horário
        delay(rampTime / maxSpeed); 
    }
}


//Função que fará o carrinho andar para Sudoeste
void So(/*unsigned int timeMotor*/) {
  //stopMotors();
    // Liga os motores adequadamente para mover para sudoeste
    for (int speed = minSpeed; speed <= maxSpeed; speed++) {
        analogWrite(motor_LA_IN, speed); // Ativa no sentido anti-horário
        analogWrite(motor_RB_IN, speed); // Ativa no sentido anti-horário
        delay(rampTime / maxSpeed);
    }

   // delay(timeMotor); // Aguarda pelo tempo especificado antes de continuar

    for (int speed = maxSpeed; speed >= minSpeed; speed--) {
        analogWrite(motor_LA_IN, speed); // Ativa no sentido anti-horário
        analogWrite(motor_RB_IN, speed); // Ativa no sentido anti-horário
        delay(rampTime / maxSpeed);
    }
    delay(10);
}


//Função que fará o carrinho andar para Nordeste
void Ne(unsigned int timeMotor){
  //stopMotors();
    // Liga os motores adequadamente para mover para sudoeste
    for (int speed = minSpeed; speed <= maxSpeed; speed++) {
        analogWrite(motor_LA_IP, speed); // Ativa no sentido anti-horário
        analogWrite(motor_RB_IP, speed); // Ativa no sentido anti-horário
        delay(rampTime / maxSpeed);
    }

    delay(timeMotor); // Aguarda pelo tempo especificado antes de continuar

    for (int speed = maxSpeed; speed >= minSpeed; speed--) {
        analogWrite(motor_LA_IP, speed); // Ativa no sentido anti-horário
        analogWrite(motor_RB_IP, speed); // Ativa no sentido anti-horário
        delay(rampTime / maxSpeed);
    }
    delay(10);
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

void fixLeft() {
    stopMotors();
    delay(10);
    digitalWrite(motor_RA_IP, HIGH);
}

void fixRight() {
    stopMotors();
    delay(10);
    digitalWrite(motor_LA_IP, HIGH);
}

//Pega a arvore e solta
void ArvoreG(){
    acionarServo(M, 10);
    acionarServo(C, 35);
    acionarServo(O, 0);
    acionarServo(C, 45);
    acionarServo(M, 45);
    acionarServo(G, 180);
    // acionarServo(B, ADEFINIR);
    acionarServo(C, 140);
    //Talvez o robô tenha que andar para trás
    acionarServo(C, 90);
    acionarServo(M, 10);
    acionarServo(C, 60);
    acionarServo(M, 50);
    acionarServo(G, 0);
    acionarServo(M, 10);
    acionarServo(C, 120);
}

//Pega a arvore e solta
void arvoreP2(){
    // acionarServo(B, A DEFINIR)
    acionarServo(C, 70);
    acionarServo(M, 75);
    acionarServo(G, 180);
    acionarServo(M, 10);
    acionarServo(C, 70);
    // acionarServo(B, A DEFINIR)
    acionarServo(M, 70);
    acionarServo(G, 0);
}

// void arvoreP(){

// }

// void arvoreG2(){

// }

void fixrote() {
    if (ColorLDR[0] == 1 && ColorLDR[2] == 0) {
        fixRight(); // Corrige à direita
    } else if (ColorLDR[0] == 0 && ColorLDR[2] == 1) {
        fixLeft(); // Corrige à esquerda
    }
}

//Ha a possibilidade de isto estar errado
void arvoreG(){
    if(mediaMovelUltrassom[0] <= dAG && mediaMovelUltrassom[1] <= dAG){
        stopMotors();
        arvoreG();
        delay(2000);
    //  while (true)
      //  {
        //    moveForward();
            //Aqui
            //Se o carro estiver andando torto colocar o fixrote em cima
            //if(LDR1 == 1 && LDR2 == 1 && LDR3 == 1){
                // stopMotors();
                // descerArvoreG();
                // delay(1000);
                //break;
            //}
           // fixrote();
        //}
        // while (true)
        // {
            // moveBackward();
            //Aqui
            //Se o carro estiver andando torto colocar o fixrote em cima
            //Pra não ter outra função a lógica da arvore pequena vem aq
            //Automaticamente não há necessecidade de verificar se é pequena ou grande, poi uma já terá sido eliminada(enfiada no lugar certo)
            //if(Ultra1 <= X && Ultra2 > X){
                stopMotors();
                arvoreP2();
                delay(2000);
                //moveForward();
                //if(LDR1 == 1 && LDR2 == 1 && LDR3 == 1){
                  //  break;
              //  }
            // }
            // fixrote();
        //}   
    //    break;   
    }
}

void arvoreP(){
    if(mediaMovelUltrassom[0] <= dAG && mediaMovelUltrassom[1] > dAG){
        stopMotors();
        arvoreP();
        delay(2000);
       // while (true)
    //  {
            //moveForward();
            //Aqui
            //Se o carro estiver andando torto colocar o fixrote em cima
            //if(LDR1 == 1 && LDR2 == 1 && LDR3 == 1){
                // stopMotors();
                // descerArvoreP();
                // delay(1000);
               // break;
          //  }
          //  fixrote();
      //  }
        // while (true)
        // {
            // moveBackward();
            //Aqui
            //Se o carro estiver andando torto colocar o fixrote em cima
            //Pra não ter outra função a lógica da arvore pequena vem aq
            //Automaticamente não há necessecidade de verificar se é pequena ou grande, poi uma já terá sido eliminada(enfiada no lugar certo)
            //if(Ultra1 <= X && Ultra2 > X){
                //stopMotors();
               // arvoreG2();
                delay(2000);
                //moveForward();
               // if(LDR1 == 1 && LDR2 == 1 && LDR3 == 1){
                    // descerArvoreG2();
                    // delay(2000);
               //     break;
              //  }
            // }
            // fixrote();
        //}  
       // break;    
    }
}

void setup(){
    Serial.begin(115200);
    while(!Serial){
      delay(10);
    }
    setUltra();
    setServos(); //configura os servos
    setLDR();
    //setMotors();
    // Criar as tarefas
    //xTaskCreatePinnedToCore(mediaUltrassomTask, "Media Ultrassom", 2048, NULL, 1, NULL, 1); // Núcleo 1
    xTaskCreatePinnedToCore(mediaLDRTask, "Media LDR", 2048, NULL, 1, NULL, 1); // Núcleo 0
    delay(100);
    Ne(200);
    stopCar(6000);
}

void loop(){
    if(ColorLDR[0] == 0 && ColorLDR[1] == 1 && ColorLDR[2] == 0){
        stopCar(200);
        while (true)
        {
            moveForward();
            fixrote();
            arvoreG(); //Se for a arvore grande ele captura a arvre grande e depois a pequena dentro da função
            arvoreP(); //Se for a arvore pequena ele captura a arvre pequena e depois a grande dentro da função
            //Se verdadeiro uma das duas, o while da break e a missão arvore foi executada
            fixrote();
        }
        stopCar(200);
        while (true)
        {
            moveBackward();
            if(ColorLDR[0] == 0 && ColorLDR[1] == 1 && ColorLDR[2] == 1){
                stopCar(200);
                turnRight();
                if(ColorLDR[0] == 0 && ColorLDR[1] == 1 && ColorLDR[3] == 0){
                    stopCar(200);
                    break;
                }
            }
            fixrote();
        }
        while (true)
        {
            moveForward();
            if(ColorLDR[0] == 1 && ColorLDR[1] == 1 && ColorLDR[2] == 1){
                stopCar(200);
                break;
            }
            fixrote();

        }        
    }

//   delay(2000);
//   Serial.print("Loop OK");
  //mediaUltrassom();
  //mediaLDR();w
  //mediaUltrassomTask("Media Ultrassom", 2048, NULL, 1, NULL, 1); // Núcleo 1

    //Robo inicia andando de lado NE
    // while(counter == 0){
    //     Ne();
    //     //Se esquerda for branco, centro preto, direita branco
    //     if (LDR1 == 0 && LDR2 == 1 && LDR3 == 0){
    //         stopCar(300);
    //         while (true)
    //         {
    //             moveForward();
    //             fixrote();
    //             if( ultra1 <= X && ultra2 <= X){ //Logica Arvore grande
    //                 stopCar(300);
    //                 //Movimentação dos servos (Pegar a arvore e depois erguer a garra)
    //                 //pegarArvore();
    //                 while (true)
    //                 {
    //                     moveForward();
    //                     fixrote();
    //                     if(LDR1 == 1 && LDR2 == 1 && LDR3 == 1){
    //                         stopMotors();
    //                         //desce a arvore
    //                         //ergue a garra
    //                         while (true)
    //                         {
    //                             moveBackward();
    //                             fixrote();
    //                             if (ultra1 <= X && ultra2 >= X){
    //                                 stopMotors();
    //                                 //pegar arvore pegarArvore();
    //                                 while (true)
    //                                 {
    //                                     moveForward();
    //                                     fixrote();
    //                                     if(LDR1 == 1 && LDR2 == 1 && LDR3 == 1){
    //                                         stopMotors();
    //                                         //desce a arvore
    //                                         //ergue a garra
    //                                     }
    //                                 }
    //                             }
    //                         }
    //                     }
    //                 }
                    
    //                 // if(LDR1 == 1 && LDR2 == 1 && LDR3 == 1){
    //                 //     stopMotors();
    //                 // }
    //                 arvore grande

    //             } else if ( ultra1 <= X && ultra2 >= X){
    //                 stopCar(300);
    //                 //Movimentação dos servos (Pegar a arvore e depois erguer a garra)
    //                 //pegarArvore();
    //                 if(LDR1 == 1 && LDR2 == 1 && LDR3 == 1){
    //                     stopCar(300);
    //                 }
    //                 arvore pequena
    //             } else{
    //                 Serial.print("Erro ao encontrar arvores")
    //                 break; // Sai do loop se não encontrar árvores
    //             }
    //         }
    //     }
    // }
}
