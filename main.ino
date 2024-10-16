//Incluindo Bibliotecas
#include "Adafruit_TCS34725.h" //Biblioteca para o sensor de cor

// Desligar o WIFI e Bluetooth

const int tempo = 10; // Definição da variável tempo com o pino 10

// Definição dos pinos dos motores do carrinho que são 4 motores resultando em 2 pinos para cada motor

// Motores traseiros direito
const int MotorBackRightA = 1; // Pino para o controle do motor traseiro direito, direção A
const int MotorBackRightB = 1; // Pino para o controle do motor traseiro direito, direção B

// Motores traseiros esquerdo
const int MotorBackLeftA = 1; // Pino para o controle do motor traseiro esquerdo, direção A
const int MotorBackLeftB = 1; // Pino para o controle do motor traseiro esquerdo, direção B

// Motores frontais direito
const int MotorFrontRightA = 1; // Pino para o controle do motor frontal direito, direção A
const int MotorFrontRightB = 1; // Pino para o controle do motor frontal direito, direção B

// Motores frontais esquerdo
const int MotorFrontLeftA = 1; // Pino para o controle do motor frontal esquerdo, direção A
const int MotorFrontLeftB = 1; // Pino para o controle do motor frontal esquerdo, direção B

const int triggerPin = 2; // Definindo o pino trigger(disparo) do sensor ultrassônico

const int echoPin = 7; // Definindo o pino de eco do sensor ultrassônico, usado para medir o tempo que a onda sonora leva para retornar ao sensor.

// Definição de uma variável inteira para o sensor ultrassonico
int cm = 0;

//Definição de uma variável inteira para controlar a velocidade

int vel = 0;

//Definição de uma constante inteira para determinar a velociade máxima
const int maxVel = 150;

// Definir os pinos do sensor de cor

/* Inicializa com valores específicos de int time e ganho */
Adafruit_TCS34725 tcsF = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X); //Definição do primeiro sensor (Sensor da Frete)

Adafruit_TCS34725 tcsE = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X); //Definição do segundo sensor (Sensor da esquerda)

Adafruit_TCS34725 tcsD = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X); //Definição do terceiro sensor (Sensor da direita)

Adafruit_TCS34725 tcsT = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X); //Definição do quarto sensor (Sensor de trás)

// Definir como os motores vão andar através do resultado do sensor de cor

// Como o sensor ultrassônico vai interferir na andança definida pelo sensor de cor

// Cria uma função que retorna um valor do tipo long resultante da leitura do sensor ultrassônico
long readUltrasonicDistance()
{
    digitalWrite(triggerPin, LOW); // Clear the trigger
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH); // Sets the trigger pin to HIGH state for 10 microseconds
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW); // Clear the trigger
    return pulseIn(echoPin, HIGH); // Reads the echo pin, and returns the sound wave travel time in microseconds
}

// Definição de uma função para dar pinMode nos pinos

// Função para configurar os pinos
void PinOut()
{
    // Configura os pinos dos motores como saídas
    pinMode(MotorBackRightA, OUTPUT);
    pinMode(MotorBackRightB, OUTPUT);
    pinMode(MotorBackLeftA, OUTPUT);
    pinMode(MotorBackLeftB, OUTPUT);
    pinMode(MotorFrontRightA, OUTPUT);
    pinMode(MotorFrontRightB, OUTPUT);
    pinMode(MotorFrontLeftA, OUTPUT);
    pinMode(MotorFrontLeftB, OUTPUT);

    // Configura os pinos do sensor ultrassônico
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);

    delay(1000); // Aguarda 1 segundo para estabilizar
}

void setup()
{
    Serial.begin(115200); // Inicializa a comunicação serial
    PinOut(); // Configura os pinos
}

void loop()
{
    cm = 0.01723 * readUltrasonicDistance();
    delay(10); // Wait for 10 millisecond(s)
    if (cm <= 15)
    {
        Serial.println("Objeto próximo!");
    }
    else
    {
        Serial.println("Objeto longe!");
    }

    //Declara quatro variáveis inteiras de 16 bits, sem sinal, que são usadas para armazenar valores de cores (vermelho, verde, azul) e a intensidade da luz.
    uint16_t r, g, b, c;

    //Pega os valores "crus" do sensor referentes ao Vermelho(r), Verde(g), Azul(b) e da Claridade(c).
    tcsT.getRawData(&r, &g, &b, &c);
    tcsD.getRawData(&r, &g, &b, &c);
    tcsE.getRawData(&r, &g, &b, &c);
    tcsF.getRawData(&r, &g, &b, &c);
}


/* Para entender as seguintes funções do código você precisará saber sobre:
    Ponte h: Onde o motor será contralado por duas etapas.
    A entrada referida para sentido horário será a A.
    A entrada referida para o sentido ant-horário será a B.
*/

//Função que fará o carrinho andar para frente
void Frente () {

    acelerar();

    analogWrite(MotorBackRightA, vel); //Liga o motor com uma velocidade x, no sentido horário
    analogWrite(MotorBackRightB, 0); //Desliga o sentido ant-horário do motor

    analogWrite(MotorBackLeftA, vel); //Liga o motor com uma velocidade x, no sentido horário
    analogWrite(MotorBackLeftB, 0); //Desliga o sentido ant-horário do motor

    analogWrite(MotorFrontRightA, vel); //Liga o motor com uma velocidade x, no sentido horário
    analogWrite(MotorFrontRightB, 0); //Desliga o sentido ant-horário do motor

    analogWrite(MotorFrontLeftA, vel); //Liga o motor com uma velocidade x, no sentido horário
    analogWrite(MotorFrontLeftB, 0); //Desliga o sentido ant-horário do motor
}

//Função que fará o carrinho andar para Tras
void Tras () {

    acelerar();
    
    analogWrite(MotorBackRightA, 0); //Desliga o sentido horário do motor
    analogWrite(MotorBackRightB, vel); //Liga o motor com uma velocidade x, no sentido ant-horário

    analogWrite(MotorBackLeftA, 0); //Desliga o sentido horário do motor
    analogWrite(MotorBackLeftB, vel); //Liga o motor com uma velocidade x, no sentido ant-horário

    analogWrite(MotorFrontRightA, 0); //Desliga o sentido horário do motor
    analogWrite(MotorFrontRightB, vel); //Liga o motor com uma velocidade x, no sentido ant-horário

    analogWrite(MotorFrontLeftA, 0); //Desliga o sentido horário do motor
    analogWrite(MotorFrontLeftB, vel); //Liga o motor com uma velocidade x, no sentido ant-horário
}

//Função que fará o carrinho andar para Direita
void Direita () {
// MotorFrontLeft - MotorBackRight (Mesma direção) Horário

// MotorFrontRight	- MotorBackLeft (Mesma direção) Ant-Horário

    acelerar(); //Chama a função para acelerar o carrinho

    analogWrite(MotorFrontLeftA, vel); //Liga o motor com uma velocidade x, no sentido horário
    analogWrite(MotorFrontLeftB, 0); //Desliga o sentido ant-horário do motor

    analogWrite(MotorBackRightA, vel); //Liga o motor com uma velocidade x, no sentido horário
    analogWrite(MotorBackRightB, 0); //Desliga o sentido ant-horário do motor

    analogWrite(MotorFrontRightA, 0); //Desliga o sentido horário do motor
    analogWrite(MotorFrontRightB, vel); //Liga o motor com uma velocidade x, no sentido ant-horário

    analogWrite(MotorBackLeftA, 0); //Desliga o sentido horário do motor
    analogWrite(MotorBackLeftB, vel); //Liga o motor com uma velocidade x, no sentido ant-horário

}

//Função que fará o carrinho andar para Esquerda
void Esquerda () {
// MotorFrontLeft - MotorBackRight (Mesma direção) Ant-Horário

// MotorFrontRight	- MotorBackLeft (Mesma direção) Horário

    acelerar(); //Chama a função para acelerar o carrinho

    analogWrite(MotorFrontLeftA, 0); //Liga o motor com uma velocidade x, no sentido horário
    analogWrite(MotorFrontLeftB, vel); //Desliga o sentido ant-horário do motor

    analogWrite(MotorBackRightA, 0); //Liga o motor com uma velocidade x, no sentido horário
    analogWrite(MotorBackRightB, vel); //Desliga o sentido ant-horário do motor

    analogWrite(MotorFrontRightA, vel); //Desliga o sentido horário do motor
    analogWrite(MotorFrontRightB, 0); //Liga o motor com uma velocidade x, no sentido ant-horário

    analogWrite(MotorBackLeftA, vel); //Desliga o sentido horário do motor
    analogWrite(MotorBackLeftB, 0); //Liga o motor com uma velocidade x, no sentido ant-horário
}

//Função que fará o carrinho andar para Sudoeste
void So(){
    // Diagonal SO:

// MotorFrontLeft - MotorBackRight (Mesma direção) Ant-Horário

    acelerar(); //Chama a função que acelera o carrinho

    analogWrite(MotorFrontLeftA, 0); //Liga o motor com uma velocidade x, no sentido horário
    analogWrite(MotorFrontLeftB, vel); //Desliga o sentido ant-horário do motor

    analogWrite(MotorBackRightA, 0); //Liga o motor com uma velocidade x, no sentido horário
    analogWrite(MotorBackRightB, vel); //Desliga o sentido ant-horário do motor
}

//Função que fará o carrinho andar para Nordeste
void Ne(){

    // Diagonal NE:

    // MotorFrontLeft - MotorBackRight (Mesma direção) Horário

    acelerar(); //Chama a função que acelera o carrinho

    analogWrite(MotorFrontLeftA, vel); //Liga o motor com uma velocidade x, no sentido horário
    analogWrite(MotorFrontLeftB, 0); //Desliga o sentido ant-horário do motor

    analogWrite(MotorBackRightA, vel); //Liga o motor com uma velocidade x, no sentido horário
    analogWrite(MotorBackRightB, 0); //Desliga o sentido ant-horário do motor
}

//Função que fará o carrinho andar para Sudeste
void Se(){

    // Diagonal SE: 

    // MotorFrontRight	- MotorBackLeft (Mesma direção) Ant-Horário

    acelerar();

    analogWrite(MotorFrontRightA, 0); //Desliga o sentido horário do motor
    analogWrite(MotorFrontRightB, vel); //Liga o motor com uma velocidade x, no sentido ant-horário

    analogWrite(MotorBackLeftA, 0); //Desliga o sentido horário do motor
    analogWrite(MotorBackLeftB, vel); //Liga o motor com uma velocidade x, no sentido ant-horário

}

//Função que fará o carrinho andar para Noroeste
void No(){
    // Diagonal NO: 

    // MotorFrontRight	- MotorBackLeft (Mesma direção) Horário

    acelerar();

    analogWrite(MotorFrontRightA, vel); //Desliga o sentido horário do motor
    analogWrite(MotorFrontRightB, 0); //Liga o motor com uma velocidade x, no sentido ant-horário

    analogWrite(MotorBackLeftA, vel); //Desliga o sentido horário do motor
    analogWrite(MotorBackLeftB, 0); //Liga o motor com uma velocidade x, no sentido ant-horário
}

void GirarD(){
    
    // Volta em relação ao eixo:

    // Girara as rodas da direita na direção x e girara as rodas da esquerda na direção contrária

    acelerar();

    analogWrite(MotorBackRightA, 0); //Desliga o sentido horário do motor
    analogWrite(MotorBackRightB, vel); //Liga o motor com uma velocidade x, no sentido ant-horário

    analogWrite(MotorFrontRightA, 0); //Desliga o sentido horário do motor
    analogWrite(MotorFrontRightB, vel); //Liga o motor com uma velocidade x, no sentido ant-horário

    analogWrite(MotorBackLeftA, vel); //Desliga o sentido horário do motor
    analogWrite(MotorBackLeftB, 0); //Liga o motor com uma velocidade x, no sentido ant-horário

    analogWrite(MotorFrontLeftA, vel); //Desliga o sentido horário do motor
    analogWrite(MotorFrontLeftB, 0); //Liga o motor com uma velocidade x, no sentido ant-horário
}

void GirarE(){
    
    // Volta em relação ao eixo:

    // Girara as rodas da direita na direção x e girara as rodas da esquerda na direção contrária

    acelerar();

    analogWrite(MotorBackRightA, vel); //Desliga o sentido horário do motor
    analogWrite(MotorBackRightB, 0); //Liga o motor com uma velocidade x, no sentido ant-horário

    analogWrite(MotorFrontRightA, vel); //Desliga o sentido horário do motor
    analogWrite(MotorFrontRightB, 0); //Liga o motor com uma velocidade x, no sentido ant-horário

    analogWrite(MotorBackLeftA, vel); //Desliga o sentido horário do motor
    analogWrite(MotorBackLeftB, 0); //Liga o motor com uma velocidade x, no sentido ant-horário

    analogWrite(MotorFrontLeftA, vel); //Desliga o sentido horário do motor
    analogWrite(MotorFrontLeftB, 0); //Liga o motor com uma velocidade x, no sentido ant-horário
}

// Função para acelerar a velocidade
void acelerar() {
    if (vel < maxVel) {
        vel += 5;
        if (vel > maxVel) vel = maxVel;
    }
  delay(50); //Delay para podermos ver o aumento de velocidade gradativo.
}


void desacelerar() {
  if (vel > 0) {
    vel -= 10;
    if (vel < 0) vel = 0; // Garantir que a velocidade não seja negativa
    Serial.println("Velocidade desacelerada: " + String(vel));
  }
}