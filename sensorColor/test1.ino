#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <cmath> // Adicione esta linha para usar sqrt

Adafruit_TCS34725 colorSensor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X); // Sensor na esquerda

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

int margem = 25;

String ColorTcs;

void checkSensorsColor() {
    if (colorSensor.begin()) {
        Serial.println("Sensor encontrado");
    }
}

void calibratorColors() {
    delay(3000);
    uint16_t r, g, b, c;
    colorSensor.getRawData(&r, &g, &b, &c);
       
    Serial.print("Não Calibradas: R=");
    Serial.print(r);
    Serial.print(", G=");
    Serial.print(g);
    Serial.print(", B=");
    Serial.print(b);
    Serial.print(", C=");
    Serial.print(c);
  
    delay(3000);
    calibR = r / 2;
    calibG = g / 2;
    calibB = b / 2;
    calibBR = std::sqrt(r) * 4.6;
    calibBG = std::sqrt(g) * 4.6;
    calibBB = std::sqrt(b) * 4.6;
    calibBB = std::sqrt(c) * 4.6;
    calibWR = r / 1.178;
    calibWG = g / 1.178;
    calibWB = b / 1.178;
    calibW = (r + b + g) / 1.178;

    Serial.println("Cores calibradas.");
    Serial.print("Calibrações: R=");
    Serial.print(calibR);
    Serial.print(", G=");
    Serial.print(calibG);
    Serial.print(", B=");
    Serial.print(calibB);
    Serial.print(", BR=");
    Serial.print(calibBR);
    Serial.print(", BG=");
    Serial.print(calibBG);
    Serial.print(", BB=");
    Serial.print(calibBB);
    Serial.print(", WR=");
    Serial.print(calibWR);
    Serial.print(", WG=");
    Serial.print(calibWG);
    Serial.print(", WB=");
    Serial.println(calibWB);
    Serial.print(", W=");
    Serial.println(calibW);
}

// void normalizeRGB(uint16_t r, uint16_t g, uint16_t b, uint8_t *rgb) {
//     delay(1);
//     uint16_t maxValue = max(r, max(g, b));
//     if (maxValue > 0) {
//         rgb[0] = (r * 255) / maxValue; // Normaliza R
//         rgb[1] = (g * 255) / maxValue; // Normaliza G
//         rgb[2] = (b * 255) / maxValue; // Normaliza B
//     } else {
//         rgb[0] = rgb[1] = rgb[2] = 0; // Se todos os valores forem zero
//     }
//     delay(1);
// }

void identifyColor(uint16_t rp, uint16_t gp, uint16_t bp, uint16_t cp, String &cor) {
    delay(1);

    if (bp <= (calibBB) &&
        gp <= (calibBG) &&
        rp <= (calibBR)) {
        cor = "Preto"; // Cor detectada: Preto
    } else if (rp >= calibR && gp <= (calibG + margem) && bp <= (calibB + margem)) {
        cor = "Vermelho"; // Cor detectada: Vermelho
    } else if (gp >= calibG && rp <= (calibR + margem) && bp <= (calibB + margem)) {
        cor = "Verde"; // Cor detectada: Verde
    } else if (bp >= calibB && rp <= (calibR + margem) && gp <= (calibG + margem)) {
        cor = "Azul"; // Cor detectada: Azul
    } else if (rp >= calibWR && gp >= calibWG && bp >= calibWB && cp >= calibW) {
        cor = "Branco"; // Cor detectada: Branco
    } else {
        cor = "Desconhecida"; // Cor desconhecida
    }

    //     if (bp <= (calibBB + margem) &&
    //     gp <= (calibBG + margem) &&
    //     rp <= (calibBR + margem)) {
    //     cor = "Preto"; // Cor detectada: Preto
    // } else if (rp >= (calibR + margem) && gp <= (calibG + margem) && bp <= (calibB + margem)) {
    //     cor = "Vermelho"; // Cor detectada: Vermelho
    // } else if (gp >= (calibG + margem) && rp <= (calibR + margem) && bp <= (calibB + margem)) {
    //     cor = "Verde"; // Cor detectada: Verde
    // } else if (bp >= (calibB + margem) && rp <= (calibR + margem) && gp <= (calibG + margem)) {
    //     cor = "Azul"; // Cor detectada: Azul
    // } else if (rp >= calibWR && bp >= calibWG && bp >= calibWB && cp >= calibW) {
    //     cor = "Branco"; // Cor detectada: Azul
    // } else {
    //     cor = "Desconhecida"; // Cor desconhecida
    // }

    //     if (cp < (calibBL + margem) && bp <= (calibB + margem) && gp <= (calibG + margem) && abs(rp - gp) < 35) {
    //     cor = "PRETO"; // Cor detectada: Preto
    // } else if (cp > (calibC + margem) && gp >= (calibG + margem) && rp >= (calibR + margem)) {
    //     cor = "BRANCO"; // Cor detectada: Branco
    // } else if (rp > (calibR + margem) && gp <= (calibG + margem) && abs(rp - gp) >= 25) {
    //     cor = "VERMELHO"; // Cor detectada: Vermelho
    // } else if (gp > (calibG + margem) && bp <= (calibB + margem)) {
    //     cor = "VERDE"; // Cor detectada: Verde
    // } else if (bp > (calibB + margem) && rp <= (calibR + margem)) {
    //     cor = "AZUL"; // Cor detectada: Azul
    // }
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
