#define triggerPin 19  // Pino trigger compartilhado
#define echoPin1 18    // Pino echo do primeiro sensor
#define echoPin2 36    // Pino echo do segundo sensor

void setup() {
    Serial.begin(115200); // Inicializa a comunicação serial
    setUltra(); // Configura os sensores ultrassônicos
}

void loop() {
    // Lê a distância do primeiro sensor
    float distance1 = readUltrasonicDistance(echoPin1);
    Serial.print("Distância Sensor 1: ");
    Serial.print(distance1);
    Serial.println(" cm");

    // Lê a distância do segundo sensor
    float distance2 = readUltrasonicDistance(echoPin2);
    Serial.print("Distância Sensor 2: ");
    Serial.print(distance2);
    Serial.println(" cm");

    delay(1000); // Espera 1 segundo entre as leituras
}

void setUltra() {
    pinMode(triggerPin, OUTPUT); // Configura o pino do trigger como saída
    pinMode(echoPin1, INPUT);    // Configura o pino do echo do primeiro sensor como entrada
    pinMode(echoPin2, INPUT);    // Configura o pino do echo do segundo sensor como entrada
}

float readUltrasonicDistance(int echoPin) {
    // Ativa o trigger
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);

    // Lê a duração do pulso no echo
    long duration = pulseIn(echoPin, HIGH);
    // Converte a duração para distância em cm
    float distance = duration * 0.01723;

    return distance; // Retorna a distância em cm
}
