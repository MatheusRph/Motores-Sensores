/* Definição dos Pinos */
#define pinLine1 36 // Pino do seguidor de linha A
#define pinLine2 35 // Pino do seguidor de linha B
#define pinLine3 39 // Pino do seguidor de linha C
#define qtdLed 3    // Informa a quantidade de sensores preto e branco serão usados

const byte pinLine[qtdLed] = {pinLine1, pinLine2, pinLine3}; // Matriz de pinos

unsigned int timeXV = 15000; // 15 segundos em milissegundos

void setLeds() {
    for (int x = 0; x < qtdLed; x++) {
        pinMode(pinLine[x], INPUT); // Define pino como entrada
        delay(timeXV); // Tempo de espera
    }
}

void setup() {
    Serial.begin(9600); 
    delay(10); 
    setLeds();
    delay(10);
    Serial.println("Loop Iniciado");
}

int readPin(int pino) { // Change return type to int
    return digitalRead(pinLine[pino]);
    delay(10);
}

void loop() {
  delay(50);
    if (readPin(2) == LOW) { // Lê o primeiro pino da matriz
        Serial.println("Preto");
    } else {
        Serial.println("Branco");
    }    
}
