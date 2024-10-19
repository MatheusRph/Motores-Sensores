/* Definição dos Pinos */
#define pinLDR1 36 // Pino do seguidor de linha A
#define pinLDR2 35 // Pino do seguidor de linha B
#define pinLDR3 39 // Pino do seguidor de linha C
#define qtdLed 3    // Informa a quantidade de sensores preto e branco serão usados

int valorLDR1 = 0;
int valorLDR2 = 0;
int valorLDR3 = 0;

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
    pinMode(pinLDR1, INPUT);
    pinMode(pinLDR2, INPUT);
    pinMode(pinLDR3, INPUT);
}

/*int readPin(int pino) { // Change return type to int
    return digitalRead(pinLine[pino]);
    delay(10);  
}*/   //codigo do seguidor, ver se da pra reutilizar

void loop() {
  valorLDR1=analogRead(pinLDR1);
  valorLDR2=analogRead(pinLDR2);
  valorLDR3=analogRead(pinLDR3);
  delay(50);
    if (valorLDR1 == 0/*colocar o valor certinho*/) { // Lê o primeiro pino da matriz
        Serial.println("Preto");
    }
if (valorLDR2 == 0/*colocar o valor certinho*/) { // com certeza tem um jeito melhor de fazer isso
        Serial.println("Preto");
    }
if (valorLDR3 == 0/*colocar o valor certinho*/) { // mas não to a fim de descobrir como
        Serial.println("Preto");
    }

        Serial.println("Branco");
    }    
}
