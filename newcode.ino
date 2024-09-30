#include <Adafruit_TCS34725.h> // Biblioteca dos Sensores de Cor

Adafruit_TCS34725 tcsF = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X); // Sensor na frente
Adafruit_TCS34725 tcsR = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X); // Sensor na direita
Adafruit_TCS34725 tcsL = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_1X); // Sensor na esquerda

// Declare suas variáveis de cor globalmente
String colorF, colorL, colorR; // Ou o tipo que você estiver usando para armazenar as cores

void checkSensorColor() {
    // Inicializa e verifica cada sensor
    if (!tcsF.begin()) {
        Serial.println("Não foi possível encontrar o sensor ColorFrente.");
        delay(10);
    } else {
        Serial.println("Sensor da frente encontrado");
    }

    if (!tcsL.begin()) {
        Serial.println("Não foi possível encontrar o sensor ColorLeft.");
        delay(10);
    } else {
        Serial.println("Sensor da esquerda encontrado");
    }

    if (!tcsR.begin()) {
        Serial.println("Não foi possível encontrar o sensor ColorRight.");
        delay(10);
    } else {
        Serial.println("Sensor da direita encontrado");
    }
}

/*
    Função: identifyColor
    Descrição: Identifica as cores com base nos valores RGB.
*/
String identifyColor(uint16_t red, uint16_t green, uint16_t blue, uint16_t clear) {
    uint32_t sum = clear;
    uint16_t cThreshold = 2300;
    uint8_t redThreshold = 140;
    uint8_t blueThreshold = 130;
    uint8_t greenThreshold = 120;
    uint8_t black = 90;

    if (sum > 0) {
        uint8_t r = (red * 255) / sum;
        uint8_t g = (green * 255) / sum;
        uint8_t b = (blue * 255) / sum;

        Serial.print("Red: "); Serial.println(r);
        Serial.print("Green: "); Serial.println(g);
        Serial.print("Blue: "); Serial.println(b);
        Serial.print("Clear: "); Serial.println(clear);

        // Identifica a cor com base nos valores normalizados
        if (r < black && g < black && b < black && clear < cThreshold){
            return "Preto";
        }
        else if (clear > r && clear > b && clear > g && clear >= cThreshold){
            return "Branco";
        } else if (r > g && r > b && r > redThreshold) {
            return "Vermelho";
        } else if (g > r && g > b && g > greenThreshold) {
            return "Verde";
        } else if (b > r && b > g && b > blueThreshold) {
            return "Azul";
        } else {
            return "Indefinido"; // Para cores que não se encaixam
        }
    } else {
        Serial.println("Valores de cor não disponíveis (soma zero).");
        return "Invalido"; // Retorna um valor padrão se a soma for zero
    }
}

/*
    Função: readTcsF
    Descrição: Lê os dados de cor do sensor tcsF e identifica a cor.
*/
void readTcsF() {
    uint16_t redF, greenF, blueF, clearF; // Declara variáveis para armazenar os valores de cor e intensidade
    tcsF.getRawData(&redF, &greenF, &blueF, &clearF); // Lê os dados brutos de cor do sensor

    colorF = identifyColor(redF, greenF, blueF, clearF); // Identifica a cor com base nos valores lidos

    Serial.print("Cor da frente: "); Serial.println(colorF); // Adiciona a impressão da cor
    delay(100); // Adiciona um atraso entre as leituras
}

/*
    Função: readTcsL
    Descrição: Lê os dados de cor do sensor tcsL e identifica a cor.
*/
void readTcsL() {
    uint16_t redL, greenL, blueL, clearL; // Declara variáveis para armazenar os valores de cor e intensidade
    tcsL.getRawData(&redL, &greenL, &blueL, &clearL); // Lê os dados brutos de cor do sensor na esquerda
    colorL = identifyColor(redL, greenL, blueL, clearL); // Identifica a cor com base nos valores lidos
    Serial.print("Cor da esquerda: "); Serial.println(colorL); // Adiciona a impressão da cor
}

/*
    Função: readTcsR
    Descrição: Lê os dados de cor do sensor tcsR e identifica a cor.
*/
void readTcsR() {
    uint16_t redR, greenR, blueR, clearR; // Declara variáveis para armazenar os valores de cor e intensidade
    tcsR.getRawData(&redR, &greenR, &blueR, &clearR); // Lê os dados brutos de cor do sensor à direita
    colorR = identifyColor(redR, greenR, blueR, clearR); // Identifica a cor com base nos valores lidos e armazena na variável global colorR
    Serial.print("Cor da direita: "); Serial.println(colorR); // Adiciona a impressão da cor
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    checkSensorColor(); // Inicializa e verifica se os sensores de cor estão corretos
    delay(1000); // Espera 1 segundo para tudo ser carregado
}

void loop() {
    readTcsF();
    delay(1500); // Adiciona um atraso de 3 segundos entre as leituras
}
