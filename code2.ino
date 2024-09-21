// ===================================
//            Bibliotecas
// ===================================

#include <Arduino.h> //Inclui a biblioteca do arduino
//#include <analogWrite.h>
#include <ServoEasing.hpp> //Inclui a biblioteca dos Servos
#include <Adafruit_NeoPixel.h> // Adiciona a biblioteca para controle de LEDs endereçáveis da Adafruit
#include <Adafruit_TCS34725.h> // Biblioteca dos Sensores de Cor
#include <Wire.h>


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
#define qtdServos 4 //Informa a quantidade de Servos que serão ligados em cascata

/*Definição do arduino em geral*/
#define D_in 16 //Nomeia o pino 6 do Arduino

// ===================================
//            Constantes
// ===================================

/*Constantes dos servos*/
const byte pinServos[qtdServos] = {servo_Pino_A, servo_Pino_B, servo_Pino_C, servo_Pino_D};

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
const byte motor_A_IN = 27; //Motor A; // Pino de entrada negativo do Motor A (Faz a roda girar no sentido anti-horário)
const byte motor_A_IP = 13; //Motor A; // Pino de entrada positiva do Motor A (Faz a roda girar no sentido horário)

//Motor B;
const byte motor_B_IN = 4;  // Pino de entrada negativa do Motor B (Faz a roda girar no sentido anti-horário)
const byte motor_B_IP = 2;  //Pino de entrada positiva do Motor B (Faz a roda girar no sentido horário)

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

String colorF; // Variável para armazenar a cor lida pelo sensor da frente
String colorL; // Variável para armazenar a cor lida pelo sensor esquerdo
String colorR; // Variável para armazenar a cor lida pelo sensor direito

/*Variáveis sobre servo*/
ServoEasing servos[qtdServos], servo_A, servo_B, servo_C, servo_D;

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
    servos[x].write(90);//Move o servo para a posição inicial de 90 graus.
    delay(timeXV);//Tempo de espera de 15 segundos
    }
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
    Função: checkSensorColor
    Descrição: Verifica se o sensor de cor foi encontrado
*/
void checkSensorColor() {

  
  // Inicializa e verifica cada sensor

  if (!tcsF.begin()) {
    Serial.println("Não foi possível encontrar o sensor ColorFrente.");
    delay(10);
    //while (1); // Para o programa se o sensor não for encontrado
  } else {
    Serial.println("Sensor da frente encontrado");
  }
  
  if (!tcsL.begin()) {
    Serial.println("Não foi possível encontrar o sensor ColorLeft.");
    delay(10);
    //while (1); // Para o programa se o sensor não for encontrado
  }else {
    Serial.println("Sensor da esquerda encontrado");
  }
  
  if (!tcsR.begin()) {
    Serial.println("Não foi possível encontrar o sensor ColorRight.");
    delay(10);
    //while (1); // Para o programa se o sensor não for encontrado
  } else {
    Serial.println("Sensor da direita encontrado");
  }
  
  //Serial.println("Todos os sensores foram inicializados com sucesso.");
}

/*
    Função: identifyColor
    Descrição: Identifica as cores com base nos valores RGB.
*/
String identifyColor(uint16_t red, uint16_t green, uint16_t blue) {
  // Limiares para identificação das cores
    const uint16_t blackThreshold = 50;    // Limite para considerar a cor como preto
    const uint16_t whiteThreshold = 200;   // Limite para considerar a cor como branco
    const uint16_t redThreshold = 200;     // Limite para considerar a cor como vermelha
    const uint16_t greenThreshold = 200;   // Limite para considerar a cor como verde
    const uint16_t blueThreshold = 200;    // Limite para considerar a cor como azul


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
    Função: readTcsF
    Descrição: Lê os dados de cor do sensor tcsF e identifica a cor.
*/
void readTcsF() {
    uint16_t redF, greenF, blueF, clearF; // Declara variáveis para armazenar os valores de cor e intensidade
    tcsF.getRawData(&redF, &greenF, &blueF, &clearF); // Lê os dados brutos de cor do sensor

    Serial.print("Red: "); Serial.println(redF);
    Serial.print("Green: "); Serial.println(greenF);
    Serial.print("Blue: "); Serial.println(blueF);
    Serial.print("Clear: "); Serial.println(clearF);

    if (clearF > 50) { // Ajuste o valor conforme necessário
        colorF = identifyColor(redF, greenF, blueF); // Identifica a cor com base nos valores lidos
        Serial.print("Cor na frente: "); // Imprime a mensagem para indicar que a cor está sendo mostrada
        Serial.println(colorF); // Exibe a cor identificada no console
    } else {
        Serial.println("Leitura não confiável (clearF baixo).");
    }

    delay(100); // Adiciona um atraso entre as leituras
}
/*
    Função: readTcsL
    Descrição: Lê os dados de cor do sensor tcsL e identifica a cor.
*/
void readTcsL() {
    uint16_t redL, greenL, blueL, clearL; // Declara variáveis para armazenar os valores de cor e intensidade
    tcsL.getRawData(&redL, &greenL, &blueL, &clearL); // Lê os dados brutos de cor do sensor na esquerda
    colorL = identifyColor(redL, greenL, blueL); // Identifica a cor com base nos valores lidos
    // Serial.print("Cor na esquerda: "); // (Opcional) Imprime a mensagem para indicar que a cor está sendo mostrada
    // Serial.println(colorL); // (Opcional) Exibe a cor identificada no console
}


/*
    Função: readTcsR
    Descrição: Lê os dados de cor do sensor tcsR e identifica a cor.
*/
void readTcsR() {
    uint16_t redR, greenR, blueR, clearR; // Declara variáveis para armazenar os valores de cor e intensidade
    tcsR.getRawData(&redR, &greenR, &blueR, &clearR); // Lê os dados brutos de cor do sensor à direita
    colorR = identifyColor(redR, greenR, blueR); // Identifica a cor com base nos valores lidos e armazena na variável global colorR
    // Serial.print("Cor na direita: "); // (Opcional) Imprime a mensagem para indicar que a cor está sendo mostrada
    // Serial.println(colorR); // (Opcional) Exibe a cor identificada no console
}



/*
    Função: setMotors
    Descrição: Configura os pinos dos motores como saída.
*/
void setMotors() {
  pinMode(motor_A_IN, OUTPUT); // Configura o pino de controle positivo do Motor A como saída
  pinMode(motor_A_IP, OUTPUT); // Configura o pino de controle negativo do Motor A como saída
  pinMode(motor_B_IN, OUTPUT);  // Configura o pino de controle positivo do Motor B como saída
  pinMode(motor_B_IP, OUTPUT);  // Configura o pino de controle negativo do Motor B como saída
}

/*
    Função: stopMotors
    Descrição: Para os motores imediatamente.
*/
void stopMotors() {
  // Parar o Carro
  analogWrite(motor_A_IN, 0); // Define a saída do pino positivo do Motor A para 0 (para o motor)
  analogWrite(motor_A_IP, 0); // Define a saída do pino negativo do Motor A para 0 (para o motor)
  analogWrite(motor_B_IN, 0);  // Define a saída do pino positivo do Motor B para 0 (para o motor)
  analogWrite(motor_B_IP, 0);  // Define a saída do pino negativo do Motor B para 0 (para o motor)
}

/*
    Função: stopCar
    Descrição: Para os motores e aguarda um tempo definido.
*/
void stopCar(unsigned int timeMotor) {
  // Parar o Carro
  analogWrite(motor_A_IN, 0);  // Define a saída do pino positivo do Motor A para 0, parando o motor
  analogWrite(motor_A_IP, 0);  // Define a saída do pino negativo do Motor A para 0, parando o motor
  analogWrite(motor_B_IN, 0);  // Define a saída do pino positivo do Motor B para 0, parando o motor
  analogWrite(motor_B_IP, 0);  // Define a saída do pino negativo do Motor B para 0, parando o motor
  delay(timeMotor); // Aguarda pelo tempo especificado (timeMotor) antes de continuar
}

/*
    Função: moveBackward
    Descrição: Move o carro para trás por um tempo determinado.
*/
void moveBackward(unsigned int timeMotor) {
  // Mover carro para trás
  analogWrite(motor_A_IN, velocityMotorA); // Ativa o motor A no sentido ant-horário com a velocidade definida
  analogWrite(motor_A_IP, 0); // Desativa o motor A no sentido horário
  analogWrite(motor_B_IN, velocityMotorB); // Ativa o motor B no sentido ant-horário com a velocidade definida
  analogWrite(motor_B_IP, 0); // Desativa o motor B no sentido horário
  delay(timeMotor); // Aguarda pelo tempo especificado (timeMotor) antes de continuar
}

/*
    Função: moveForward
    Descrição: Move o carro para frente por um tempo determinado.
*/
void moveForward(unsigned int timeMotor) {
  // Mover carro para frente
  analogWrite(motor_A_IN, 0); // Desativa o motor A no sentido anti-horário
  analogWrite(motor_A_IP, velocityMotorA); // Ativa o motor A no sentido horário com a velocidade definida
  analogWrite(motor_B_IN, 0); // Desativa o motor B no sentido anti-horário
  analogWrite(motor_B_IP, velocityMotorB); // Ativa o motor B no sentido horário com a velocidade definida
  delay(timeMotor); // Aguarda pelo tempo especificado (timeMotor) antes de continuar
}

/*
    Função: turnRight
    Descrição: Vira o carro para a direita.
*/
void turnRight(unsigned int timeMotor) {
  // Mover carro para direita
  analogWrite(motor_A_IN, velocityMotorA); // Ativa o motor A no sentido anti-horário com a velocidade definida
  analogWrite(motor_A_IP, 0); // Desativa o motor A no sentido horário
  analogWrite(motor_B_IN, 0); // Desativa o motor B no sentido anti-horário
  analogWrite(motor_B_IP, velocityTurnMotorB); // Ativa o motor B no sentido horário com a velocidade de giro
  delay(timeMotor); // Aguarda pelo tempo especificado (timeMotor) antes de continuar
}

/*
    Função: turnLeft
    Descrição: Vira o carro para a esquerda.
*/
void turnLeft(unsigned int timeMotor) {
  // Mover carro para esquerda
  analogWrite(motor_A_IN, velocityTurnMotorA); // Ativa o motor A no sentido anti-horário com a velocidade de giro
  analogWrite(motor_A_IP, 0); // Desativa o motor A no sentido horário
  analogWrite(motor_B_IN, 0); // Desativa o motor B no sentido anti-horário
  analogWrite(motor_B_IP, velocityMotorB); // Ativa o motor B no sentido horário com a velocidade definida
  delay(timeMotor); // Aguarda pelo tempo especificado (timeMotor) antes de continuar
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
  //setServos(); //configura os servos
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(1000);
  checkSensorColor(); //Inicializa e verifica se os sensores de cor estão corretos
  //stopCar(1000); //para o robo por 1 segundo
  delay(1000); //espera 1 segundo para tudo ser carregado
}

/*
    Função: loop
    Descrição: Inicia o código e fica em loop
*/
void loop() {
  readTcsF();
  delay(1000);
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