#include "Wire.h"
#include "Adafruit_TCS34725.h"

// Cria instâncias do SoftWire para cada sensor

// Cria instâncias dos sensores usando os barramentos SoftWire
//Adafruit_TCS34725softi2c  colorSensor = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X, &wire1);
//Adafruit_TCS34725softi2c  tcs2 = Adafruit_TCS34725softi2c(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X, &wire2);

// #include <Wire.h>
// #include <Adafruit_TCS34725.h>
// #include <cmath> // Adicione esta linha para usar sqrt

Adafruit_TCS34725 colorSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X); //240 1x

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

void checkSensorsColor() {
    if (colorSensor.begin()) {
        Serial.println("Sensor frente encontrado");
    } else {
        Serial.println("Sensor frente não encontrado");
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

    colorSensor.getRawData(&r, &g, &b, &c);

    if (c < 750){
      colorSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_60MS, TCS34725_GAIN_4X); //240 1x
    }


    int r10 = 0, g10 = 0, b10 = 0, c10 = 0;

    for (int i = 0; i < 10; i++) {
        colorSensor.getRawData(&r, &g, &b, &c);
        r10 += r;
        g10 += g;
        b10 += b;
        c10 += c;
    }

    r = r10/10;
    g = g10/10;
    b = b10/10;
    c = c10/10;
       
    // Serial.print("Não Calibradas: R=");
    // Serial.print(r);
    // Serial.print(", G=");
    // Serial.print(g);
    // Serial.print(", B=");
    // Serial.print(b);
    // Serial.print(", C=");
    // Serial.print(c);
  
    delay(3000);
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
    colorSensor.getRawData(&r, &g, &b, &c);
    identifyColor(r, g, b, c, ColorTcs);

    Serial.print("Cores de leitura R=");
    Serial.print(r);
    Serial.print(", G=");
    Serial.print(g);
    Serial.print(", B=");
    Serial.print(b);
    Serial.print(", C=");
    Serial.print(c);

    Serial.println("Cor detectada: " + ColorTcs);
}

void setup() {
    Serial.begin(9600);
    checkSensorsColor();
    calibratorColors(); // Calibrar as cores na inicialização
}

void loop() {
    readTcs();
    delay(1000);
}