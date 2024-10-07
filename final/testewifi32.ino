// Servo Sweep example for the ESP32
// https://wokwi.com/arduino/projects/323706614646309460
// Criado por PionersTech
// Professor Thales Augusto Cargdoso dos Santos
// Controle para motores, Servos e RGB Endereçável de um Robô
//inclusão das bibliotecas
#include <Arduino.h>
//#include <analogWrite.h>
#include <ServoEasing.hpp>
#include <Adafruit_NeoPixel.h>  //Adiciona a biblioteca
#include <WiFi.h>
#include <ESPmDNS.h>

#define NUM_SERVOS 4  //Informa a quantidade de Servos que serão ligados em cascata
#define D_in 16       //Nomeia o pino 6 do Arduino
#define NUM_LEDS 4    //Informa a quantidade de LEDs que serão ligados em cascata

#define NUM_MOTORS 4
#define motorA_L 27
#define motorA_R 13
#define motorB_L 4
#define motorB_R 2

#define servo_Pino_A 32
#define servo_Pino_B 33
#define servo_Pino_C 25
#define servo_Pino_D 26

Adafruit_NeoPixel pixels(NUM_LEDS, D_in, NEO_GRB + NEO_KHZ800);  //Instancia o objeto "pixels", informando a quantidade e o pino de sinal

//uint8_t R = 255, G = 255, B = 255;

uint8_t red = 0, green = 0, blue = 0;  // Variables for storing RGB values
unsigned int delayMs = 10;             // Time between LED updates in milliseconds

//unsigned int timeX = 10;
//M1 27 13
//M2 4 2
//M3 17 12
//M4 15 14
// variaveis para definição da velocidade dos motores
byte velocityMotorA = 0x7f;  // 127
byte velocityMotorB = 0x7f;  // 127
// variaveis para definição da velocidade dos motores
byte velocityTurnMotorA = 0x32;  // 50
byte velocityTurnMotorB = 0x32;  // 50
// variaveis para definição da velocidade dos motores em rampa de aceleração e desaceleração
int rampTime = 1000;  // Tempo de rampa em milissegundos (ajuste conforme a necessidade)
int maxSpeed = 255;   // Velocidade máxima do motor (ajuste de acordo com seu motor)

unsigned int timeMotor = 1000;  //Tempo de ligar o Motor

/*
//Motor A;
const byte motorA_L = 27;
const byte motorA_R = 13;
//Motor B;
const byte motorB_L = 4;
const byte motorB_R = 2;
*/


const char* ssid = "koilight";
const char* password = "koil1234";
IPAddress local_IP(192, 168, 4, 1);  // IP fixo
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);


WiFiServer server(80);

// Function to send the HTML form with updated input values
void sendHTML(WiFiClient client, uint8_t r, uint8_t g, uint8_t b) {
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println();
  client.println("<!DOCTYPE html><html lang=\"pt-br\">");
  client.println("<meta charset=\"UTF-8\"><meta http-equiv=\"X-UA-Compatible\" content=\"IE=edge\"><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"><head><title>Control RGB LED</title></head><body>");
  client.println("<form method='get' action='/'>");
  // // Button for moving forward
  // client.println("<button type='submit' name='MF'>Move Forward</button><br>");

  // // Button for moving backward
  // client.println("<button type='submit' name='MB'>Move Backward</button><br>");

  // // Button for moving left
  // client.println("<button type='submit' name='ML'>Move Left</button><br>");

  // // Button for moving right
  // client.println("<button type='submit' name='MR'>Move Right</button><br>");

  // client.println("Red  : <input type='range' id='red' name='red' min='0' max='255' value='" + String(r) + "' oninput='updateValue(this.value, \"redValue\")'> <span id='redValue'>" + String(r) + "</span><br>");
  // client.println("Green: <input type='range' id='green' name='green' min='0' max='255' value='" + String(g) + "' oninput='updateValue(this.value, \"greenValue\")'> <span id='greenValue'>" + String(g) + "</span><br>");
  // client.println("Blue : <input type='range' id='blue' name='blue' min='0' max='255' value='" + String(b) + "' oninput='updateValue(this.value, \"blueValue\")'> <span id='blueValue'>" + String(b) + "</span><br>");
  // client.println("<input type='submit' value=' Set Color'>");
  // client.println("</form></body></html>");
  
}

// Function to process HTTP requests
void processRequest(WiFiClient client) {
  String request = client.readStringUntil('\r');
  client.println();  // Send HTTP response header


  // Extract RGB values from the request (if form submitted)
  if (request.indexOf("?") > -1) {  // Check for query parameters
    int redIndex = request.indexOf("red=") + 4;
    int greenIndex = request.indexOf("green=") + 6;
    int blueIndex = request.indexOf("blue=") + 5;

    if (redIndex > -1 && greenIndex > -1 && blueIndex > -1) {
      red = request.substring(redIndex).toInt();
      green = request.substring(greenIndex).toInt();
      blue = request.substring(blueIndex).toInt();
    }
  }
/*
  if (client.available() > 0) {
    if (client.readStringUntil('=') == "MF") {
      // Move forward function called
      moveForwardRamp(timeMotor);
      Serial.println("Moving Forward...");
    } else if (client.readStringUntil('=') == "MB") {
      // Move backward function called
      moveBackwardRamp(timeMotor);
      Serial.println("Moving Backward...");
    } else if (client.readStringUntil('=') == "ML") {
      // Move left function called
      servoEsquerda(1);
      //turnLeft(timeMotor);
      Serial.println("turning Left...");
    } else if (client.readStringUntil('=') == "MR") {
      // Move backward function called
      servoDireita(1);
      //turnRight(timeMotor);
      Serial.println("turning Right...");
    }
  }
*/
  // Update and show NeoPixel colors
  showRGB(red, green, blue);
  delay(delayMs);

  // Send the HTML form back with updated input values
  sendHTML(client, red, green, blue);
}

void handleClient() {
  WiFiClient client = server.available();
  if (!client) {
    return;
  }
  Serial.println("New client connected.");
  processRequest(client);
  client.stop();  // Close the connection
  Serial.println("Client disconnected.");
}

// Function to set and display NeoPixel colors
void showRGB(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_LEDS; i++) {
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
  pixels.show();
  delay(delayMs);
}

const byte pinMotors[NUM_MOTORS] = { motorA_L, motorA_R, motorB_L, motorB_R };

ServoEasing servos[NUM_SERVOS], servo_A, servo_B, servo_C, servo_D;

const byte pinServos[NUM_SERVOS] = { servo_Pino_A, servo_Pino_B, servo_Pino_C, servo_Pino_D };

void setServos() {
  for (int x = 0; x < NUM_SERVOS; x++) {
    pinMode(pinServos[x], OUTPUT);
    servos[x].attach(pinServos[x]);
    servos[x].setEasingType(EASE_CUBIC_IN_OUT);
    servos[x].setSpeed(70);
    servos[x].write(90);
    delay(delayMs);
  }
}

// Função para acionar um servomotor específico
void acionarServo(int numeroServo, int angulo) {
  Serial.println("Movendo Servo :" + String(numeroServo));
  if (numeroServo >= 0 && numeroServo < NUM_SERVOS) {
    servos[numeroServo].write(angulo);
  } else {
    Serial.println("Número de servo inválido!");
  }
}

void servoEsquerda(int numeroServo) {  // função criada para girar servo para esquerda
  Serial.println("Movendo Servo :" + String(numeroServo) + " Esquerda!");
  delay(10);
  if (numeroServo >= 0 && numeroServo < NUM_SERVOS) {
    for (int pos = 90; pos <= 180; pos += 1) {
      servos[numeroServo].write(pos);
      delay(delayMs);  // Wait for 15 millisecond(s)
    }
    for (int pos = 180; pos >= 90; pos -= 1) {
      servos[numeroServo].write(pos);
      delay(delayMs);  // Wait for 15 millisecond(s)
    }
  } else {
    Serial.println("Número de servo inválido!");
    delay(10);
  }
}

void servoDireita(int numeroServo) {  // função criada para girar servo para direita
  if (numeroServo >= 0 && numeroServo < NUM_SERVOS) {
    Serial.println("Movendo Servo :" + String(numeroServo) + " Direita!");
    for (int pos = 90; pos >= 0; pos -= 1) {
      servos[numeroServo].write(pos);
      delay(delayMs);  // Wait for 15 millisecond(s)
    }
    for (int pos = 0; pos <= 90; pos += 1) {
      servos[numeroServo].write(pos);
      delay(delayMs);  // Wait for 15 millisecond(s)
    }
  } else {
    Serial.println("Número de servo inválido!");
  }
}

void setMotors() {
  for (int j = 0; j < NUM_MOTORS; j++) {
    pinMode(pinMotors[j], OUTPUT);
    delay(delayMs);
  }
}

void stopMotors() {
  // Parar o Carro
  for (int j = 0; j < NUM_MOTORS; j++) {
    analogWrite(pinMotors[j], 0);
  }
  delay(delayMs);
}

void stopCar(unsigned int timeMotor) {
  // Parar o Carro
  stopMotors();
  delay(timeMotor);
}

void moveForwardRamp(unsigned int timeMotor) {
  // Aceleração gradual para frente
  for (int speed = 0; speed <= maxSpeed; speed++) {
    analogWrite(motorA_L, speed);
    analogWrite(motorA_R, 0);
    analogWrite(motorB_L, speed);
    analogWrite(motorB_R, 0);
    delay(rampTime / maxSpeed);  // Ajuste a temporização por etapa
  }
  delay(timeMotor);  // Tempo de movimento na velocidade máxima

  // Desaceleração gradual para parar
  for (int speed = maxSpeed; speed >= 0; speed--) {
    analogWrite(motorA_L, speed);
    analogWrite(motorA_R, 0);
    analogWrite(motorB_L, speed);
    analogWrite(motorB_R, 0);
    delay(rampTime / maxSpeed);
  }
}

void moveBackwardRamp(unsigned int timeMotor) {
  // Aceleração gradual para trás (inverte lógica)
  for (int speed = 0; speed < maxSpeed; speed++) {
    analogWrite(motorA_L, 0);
    analogWrite(motorA_R, speed);
    analogWrite(motorB_L, 0);
    analogWrite(motorB_R, speed);
    delay(rampTime / maxSpeed);
  }
  delay(timeMotor);  // Tempo de movimento na velocidade máxima

  // Desaceleração gradual para parar
  for (int speed = maxSpeed; speed >= 0; speed--) {
    analogWrite(motorA_L, 0);
    analogWrite(motorA_R, speed);
    analogWrite(motorB_L, 0);
    analogWrite(motorB_R, speed);
    delay(rampTime / maxSpeed);
  }
}

void moveBackward(unsigned int timeMotor) {
  // Mover carro para frente
  analogWrite(motorA_L, velocityMotorA);
  analogWrite(motorA_R, 0);
  analogWrite(motorB_L, velocityMotorB);
  analogWrite(motorB_R, 0);
  delay(timeMotor);
}

void moveForward(unsigned int timeMotor) {
  // Mover carro para tras
  analogWrite(motorA_L, 0);
  analogWrite(motorA_R, velocityMotorA);
  analogWrite(motorB_L, 0);
  analogWrite(motorB_R, velocityMotorB);
  delay(timeMotor);
}

void turnRight(unsigned int timeMotor) {
  // Mover carro para direita
  analogWrite(motorA_L, velocityMotorA);
  analogWrite(motorA_R, 0);
  analogWrite(motorB_L, velocityTurnMotorB);
  analogWrite(motorB_R, 0);
  delay(timeMotor);
}

void turnLeft(unsigned int timeMotor) {
  // Mover carro para esquerda
  analogWrite(motorA_L, velocityTurnMotorA);
  analogWrite(motorA_R, 0);
  analogWrite(motorB_L, velocityMotorB);
  analogWrite(motorB_R, 0);
  delay(timeMotor);
}

void setup() {
  pinMode(D_in, OUTPUT);  //Configura o pino 16 como saída
  pixels.begin();         //Inicia o objeto "pixels"
  pixels.clear();         //desliga todos os LEDs
  setMotors();
  setServos();
  stopCar(1000);
  Serial.begin(115200);
  delay(1000);

  // Configura o modo Access Point
  // Configure Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  Serial.println("WiFi configured as Access Point");
  Serial.print("SSID: ");
  Serial.println(ssid);
  Serial.print("Password: ");
  Serial.println(password);
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());

  server.begin();
  // Inicia o servidor mDNS
  MDNS.begin(ssid);
  MDNS.addService("http", "tcp", 80);

  showRGB(255, 0, 0);  // Initial red color
  stopCar(1);
}

void loop() {

  handleClient();
  delay(10);
  //timeX = random(50)+1;
  //showColors(27, 28, 30);
  //showColors(255, 0, 0);
  //showColors(0, 255, 0);
  ///showColors(0, 0, 255);
  //showColors(255, 255, 0);
  //showColors(255, 0, 255);
  //showColors(0, 255, 255);
  //showColors(255, 0, 0);
  //showColors(255, 255, 255);
  //showColors(150, 0, 75);
  //servoDireita(1);
  //servoEsquerda(1);
  //servoDireita(2);
  //servoEsquerda(2);
  //servoDireita(3);
  //servoEsquerda(3);
  //servoDireita(0);
  //servoEsquerda(0);
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