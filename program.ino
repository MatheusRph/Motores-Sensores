// ===================================
//            Bibliotecas
// ===================================

#include <Arduino.h> // Biblioteca do Arduino
#include <ServoEasing.hpp> // Biblioteca dos Servos
#include <Adafruit_TCS34725.h> // Biblioteca dos Sensores de Cor

// ===================================
//            Defines
// ===================================

ServoEasing servo_A, servo_B; // Objetos de controle para dois servos com suavização de movimento
// Define dos Servos
#define servo_Pino_A 32 // Pino do Servo A
#define servo_Pino_B 33 // Pino do Servo B
#define servo_Pino_C 25 // Pino do Servo C
#define servo_Pino_D 26 // Pino do Servo D

// ===================================
//            Constantes
// ===================================

// Velocidade ao ligar motor
const byte velocityTurnMotorA = 0x32; // Velocidade de giro do Motor A (50)
const byte velocityTurnMotorB = 0x32; // Velocidade de giro do Motor B (50)

// Constantes de definição dos pinos dos motores
// Motor A
const byte motor_A_IN = 27; // Pino de entrada positiva do Motor A (Faz a roda girar no sentido horário)
const byte motor_A_IP = 13; // Pino de entrada negativa do Motor A (Faz a roda girar no sentido anti-horário)
// Motor B
const byte motor_B_IN = 4;  // Pino de entrada positiva do Motor B (Faz a roda girar no sentido horário)
const byte motor_B_IP = 2;  // Pino de entrada negativa do Motor B (Faz a roda girar no sentido anti-horário)

// ===================================
//            Variáveis
// ===================================

/* Variáveis de velocidade */
const byte velocityMotorA = 0x5f; // Velocidade do Motor A (127)
const byte velocityMotorB = 0x5f; // Velocidade do Motor B (127)

/* Variáveis de tempo */
unsigned int timeMotor = 1000;  // Tempo de ligar o Motor

/* Variáveis de configuração dos sensores de cor */
Adafruit_TCS34725 tcsF = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X); // Sensor na frente
Adafruit_TCS34725 tcsR = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X); // Sensor na direita
Adafruit_TCS34725 tcsL = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X); // Sensor na esquerda

String colorF; // Variável para armazenar a cor lida pelo sensor da frente
String colorL; // Variável para armazenar a cor lida pelo sensor esquerdo
String colorR; // Variável para armazenar a cor lida pelo sensor direito


// ===================================
//            Funções
// ===================================

/*
    Função: setServos
    Descrição: Configuração dos servos ao iniciar o Arduino.
*/
void setServos() {
  // Define os pinos dos Servos como Saída
  pinMode(servo_Pino_A, OUTPUT);
  pinMode(servo_Pino_B, OUTPUT);
  // Anexa os Pinos dos Servos
  servo_A.attach(servo_Pino_A);
  servo_B.attach(servo_Pino_B);
  // Define o tipo de suavização do movimento (Easing)
  servo_A.setEasingType(EASE_CUBIC_IN_OUT);
  servo_B.setEasingType(EASE_CUBIC_IN_OUT);
  // Define a velocidade dos Servos
  servo_A.setSpeed(70);
  servo_B.setSpeed(70);
  // Define a posição inicial dos Servos
  servo_A.write(90);
  servo_B.write(90);
}

/*
    Função: checkSensorColor
    Descrição: Verifica se o sensor de cor foi encontrado
*/
void checkSensorColor() {
  // Inicializa e verifica cada sensor
  if (!tcsF.begin()) {
    Serial.println("Não foi possível encontrar o sensor ColorFrente.");
    while (1); // Para o programa se o sensor não for encontrado
  }
  
  if (!tcsL.begin()) {
    Serial.println("Não foi possível encontrar o sensor ColorLeft.");
    while (1); // Para o programa se o sensor não for encontrado
  }
  
  if (!tcsR.begin()) {
    Serial.println("Não foi possível encontrar o sensor ColorRight.");
    while (1); // Para o programa se o sensor não for encontrado
  }
  
  Serial.println("Todos os sensores foram inicializados com sucesso.");
}

/*
    Função: readTcsF
    Descrição: Lê os dados de cor do sensor tcsF e identifica a cor.
*/
void readTcsF() {
    uint16_t redF, greenF, blueF, clearF;
    tcsF.getRawData(&redF, &greenF, &blueF, &clearF);
    colorF = identifyColor(redF, greenF, blueF);
    Serial.print("Cor na frente: ");
    Serial.println(colorF);
}

/*
    Função: readTcsL
    Descrição: Lê os dados de cor do sensor tcsL e identifica a cor.
*/
void readTcsL() {
    uint16_t redL, greenL, blueL, clearL;
    tcsL.getRawData(&redL, &greenL, &blueL, &clearL);
    colorL = identifyColor(redL, greenL, blueL);
    // Serial.print("Cor na esquerda: ");
    // Serial.println(colorL);
}

/*
    Função: readTcsR
    Descrição: Lê os dados de cor do sensor tcsR e identifica a cor.
*/
void readTcsR() {
    uint16_t redR, greenR, blueR, clearR;
    tcsR.getRawData(&redR, &greenR, &blueR, &clearR);
    colorR = identifyColor(redR, greenR, blueR); // Armazena a cor lida na variável global colorR
    // Serial.print("Cor na direita: ");
    // Serial.println(colorR);
}

/*
    Função: identifyColor
    Descrição: Identifica as cores com base nos valores RGB.
*/
String identifyColor(uint16_t red, uint16_t green, uint16_t blue) {
  // Limiares para identificação das cores
  const uint16_t blackThreshold = 100;    // Limite para considerar a cor como preto (baixa intensidade)
  const uint16_t whiteThreshold = 1000;   // Limite para considerar a cor como branco (alta intensidade)
  const uint16_t redThreshold = 1000;     // Limite para considerar a cor como vermelha (alta intensidade vermelha)
  const uint16_t greenThreshold = 1000;   // Limite para considerar a cor como verde (alta intensidade verde)
  const uint16_t blueThreshold = 1000;    // Limite para considerar a cor como azul (alta intensidade azul)

  // Verifica se a cor detectada é preta
  if (red < blackThreshold && green < blackThreshold && blue < blackThreshold) {
    return "Preto";
  }

  // Verifica se a cor detectada é branca
  if (red > whiteThreshold && green > whiteThreshold && blue > whiteThreshold) {
    return "Branco";
  }

  // Verifica se a cor detectada é vermelha
  if (red > redThreshold && green < greenThreshold && blue < blueThreshold) {
    return "Vermelho";
  }

  // Verifica se a cor detectada é verde
  if (red < redThreshold && green > greenThreshold && blue < blueThreshold) {
    return "Verde";
  }

  // Verifica se a cor detectada é azul
  if (red < redThreshold && green < greenThreshold && blue > blueThreshold) {
    return "Azul";
  }

  // Se nenhuma das condições acima for satisfeita, retorna "Desconhecida"
  return "Desconhecida";
}

/*
    Função: setMotors
    Descrição: Configuração dos pinos dos motores ao iniciar o Arduino.
*/
void setMotors() {
  // Define os pinos dos Motores como Saída
  pinMode(motor_A_IN, OUTPUT);
  pinMode(motor_A_IP, OUTPUT);
  pinMode(motor_B_IN, OUTPUT);
  pinMode(motor_B_IP, OUTPUT);
}

/*
    Função: stopMotors
    Descrição: Para os motores imediatamente.
*/
void stopMotors() {
  // Parar o Motor A e Motor B
  analogWrite(motor_A_IN, 0);
  analogWrite(motor_A_IP, 0);
  analogWrite(motor_B_IN, 0);
  analogWrite(motor_B_IP, 0);
}

/*
    Função: stopCar
    Descrição: Para os motores e adiciona um delay no final.
*/
void stopCar(unsigned int timeMotor) {
  stopMotors(); // Chama a função stopMotors para parar os motores
  delay(timeMotor); // Delay para esperar o carro parar
}

/*
    Função: moveBackward
    Descrição: Move o carro para trás por um tempo determinado.
*/
void moveBackward(unsigned int timeMotor) {
  // Mover carro para trás
  analogWrite(motor_A_IN, velocityMotorA);
  analogWrite(motor_A_IP, 0);
  analogWrite(motor_B_IN, velocityMotorB);
  analogWrite(motor_B_IP, 0);
  // Delay para controlar o tempo de movimento
  delay(timeMotor);
}

/*
    Função: moveForward
    Descrição: Move o carro para frente por um tempo determinado.
*/
void moveForward(unsigned int timeMotor) {
  // Mover carro para frente
  analogWrite(motor_A_IN, 0);
  analogWrite(motor_A_IP, velocityMotorA);
  analogWrite(motor_B_IN, 0);
  analogWrite(motor_B_IP, velocityMotorB);
  // Delay para controlar o tempo de movimento
  delay(timeMotor);
}

/*
    Função: moveFrente
    Descrição: Move o carro para frente continuamente.
*/
void moveFrente() {
  // Mover carro para frente
  analogWrite(motor_A_IN, 0);
  analogWrite(motor_A_IP, velocityMotorA);
  analogWrite(motor_B_IN, 0);
  analogWrite(motor_B_IP, velocityMotorB);
}

/*
    Função: turnRight
    Descrição: Gira o carro para a direita por um tempo determinado.
*/
void turnRight(unsigned int timeMotor) {
  // Girar carro para a direita
  analogWrite(motor_A_IN, velocityMotorA);
  analogWrite(motor_A_IP, 0);
  analogWrite(motor_B_IN, velocityTurnMotorB);
  analogWrite(motor_B_IP, 0);
  // Delay para controlar o tempo de giro
  delay(timeMotor);
}

/*
    Função: turnLeft
    Descrição: Gira o carro para a esquerda por um tempo determinado.
*/
void turnLeft(unsigned int timeMotor) {
  // Girar carro para a esquerda
  analogWrite(motor_A_IN, velocityTurnMotorA);
  analogWrite(motor_A_IP, 0);
  analogWrite(motor_B_IN, velocityMotorB);
  analogWrite(motor_B_IP, 0);
  // Delay para controlar o tempo de giro
  delay(timeMotor);
}

/*
    Função: setup
    Descrição: Inicia o código apenas uma vez ao ligar o microcontrolador
*/
void setup() {
    Serial.begin(9600); // Inicializa a comunicação serial
    checkSensorColor();
    setMotors();
    setServos();
}

void loop() {
    readTcsF();
    readTcsL();
    readTcsR();

    static bool pertoDaLinha = false;
    static bool nalinha = false;
    
    if (!pertoDaLinha) {
        moveFrente(); // Move o carro para frente continuamente
        
        // Verifica se o sensor da direita detecta branco
        if (colorR == "Branco") {
            // Continua a movimentação até detectar preto
            while (colorR == "Branco") {
                colorR = readTcsR();
            }

            // Agora que o sensor da direita detectou preto, entra no estado perto da linha
            pertoDaLinha = true;
            stopCar(1000); // Para o carro e aguarda por um momento
            Serial.println("Estado de alerta ativado!");
        }
    } else {
        // No estado de alerta
        if (colorR == "Branco") {
            // Verifica as cores dos sensores da frente e da esquerda
            if (colorF == "Preto" && colorL == "Branco") {
                // Continua movimentação para frente
                moveFrente();
                nalinha = true;
                pertoDaLinha = false;
            }
        }
    }
    if(nalinha == true){
        moveForward()
    }
    
}
