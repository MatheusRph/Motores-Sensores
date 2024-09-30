#include <Adafruit_TCS34725.h> // Biblioteca dos Sensores de Cor
#include <Wire.h>

float calibR = 0.29;
float calibB = 0.35;
float calibG = 0.31;

#define margem 0.01 // Removido o ponto e vírgula

Adafruit_TCS34725 tcsF = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X); // Sensor na frente

void setup(){
    Serial.begin(115200);
    if (!tcsF.begin()) {
        Serial.println("Não foi possível encontrar o sensor ColorFrente.");
        delay(10);
    } else {
        Serial.println("Sensor da frente encontrado");
    }
}

void loop(){
    uint16_t r, g, b, c, colorTemp, lux;
    float rp, gp, bp;

    tcsF.getRawData(&r, &g, &b, &c);

    colorTemp = tcsF.calculateColorTemperature(r, g, b, c);
    lux = tcsF.calculateLux(r, g, b); // Corrigido: tcs para tcsF

    rp = (float)r / (float)c;
    gp = (float)g / (float)c;
    bp = (float)b / (float)c;

    int cor = 0;

    if (rp > (calibR + margem) && gp <= (calibG + margem)) {
        cor = 1; // Cor detectada: Vermelho
    } else if (gp > (calibG + margem) && bp <= (calibB + margem)) {
        cor = 2; // Cor detectada: Verde
    } else if (bp > (calibB + margem) && rp <= (calibR + margem)) {
        cor = 3; // Cor detectada: Azul
    } else {
        cor = 0; // Cor desconhecida
    }

    Serial.print("Cor detectada: ");
    Serial.println(cor); // Adicionado para mostrar a cor detectada
}
