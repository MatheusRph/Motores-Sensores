#include <Adafruit_TCS34725.h> // Biblioteca dos Sensores de Cor
#include <Wire.h>
#include <cmath> // Adicione esta linha para usar sqrt

#define D_in 16 //Nomeia o pino 16 do Arduino
#define CONFIG_SPIRAM_USE_MALLOC 1

float calibR = 0.291;
float calibB = 0.304;
float calibG = 0.294;
float calibW = 0;
float calibBL = 0;

#define margem 0.01 // Removido o ponto e vírgula

Adafruit_TCS34725 tcsF = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X); // Sensor na frente

void luz() {
  uint16_t r, g, b, c;
  Serial.println("Definindo claridade:");
  delay(5000);
  tcsF.getRawData(&r, &g, &b, &c);
  
  calibW = ((float)c / 1000) - 0.152;
  calibBL = (sqrt((float)c / 1000) - 0.401) / 2; // Cálculo direto em calibBL
}

void setup(){
    pinMode(D_in, OUTPUT); //Configura o pino 16 como saída
    digitalWrite(16, HIGH); // Desliga o LED RGB
    Serial.begin(115200);
    if (!tcsF.begin()) {
        Serial.println("Não foi possível encontrar o sensor ColorFrente.");
        delay(10);
    } else {
        Serial.println("Sensor da frente encontrado");
    }
    delay(1000);
    luz();
}

void loop(){
    uint16_t r, g, b, c, colorTemp, lux;
    float rp, gp, bp, cp;

    tcsF.getRawData(&r, &g, &b, &c);

    colorTemp = tcsF.calculateColorTemperature(r, g, b);

    lux = tcsF.calculateLux(r, g, b);

    rp = (float)r / (float)c;
    gp = (float)g / (float)c;
    bp = (float)b / (float)c;
    cp = (float)c / 1000;
    

    String cor = "Desconhecida";

    // Serial print para lux e clear
    Serial.print("Branco: ");
    Serial.println(cp); // Mostrando o valor de clear
    Serial.print("Red: ");
    Serial.println(rp); // Mostrando o valor de vermelho
    Serial.print("Blue: ");
    Serial.println(bp); // Mostrando o valor de azul
    Serial.print("Green: ");
    Serial.println(gp); // Mostrando o valor de verde

    if (cp < (calibBL + margem) && bp <= (calibB + margem) && gp <= (calibG + margem) && abs(r - g) < 35) {
        cor = "PRETO"; // Cor detectada: Preto
    } else if (cp > (calibW + margem) && gp >= (calibG + margem) && rp >= (calibR + margem)) {
        cor = "BRANCO"; // Cor detectada: Branco
    } else if (rp > (calibR + margem) && gp <= (calibG + margem) && abs(r - g) >= 25) {
        cor = "VERMELHO"; // Cor detectada: Vermelho
    } else if (gp > (calibG + margem) && bp <= (calibB + margem)) {
        cor = "VERDE"; // Cor detectada: Verde
    } else if (bp > (calibB + margem) && rp <= (calibR + margem)) {
        cor = "AZUL"; // Cor detectada: Azul
    } else {
        cor = "Desconhecida"; // Cor des"Green: ");
    }

    Serial.print("Cor detectada: ");
    Serial.println(cor); // Adicionado para mostrar a cor detectada
}
