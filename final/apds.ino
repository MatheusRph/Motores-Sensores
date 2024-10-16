// Adicao da biblioteca
#include <SparkFun_APDS9960.h>

// Criacao do objeto para leitura do sensor
SparkFun_APDS9960 apds = SparkFun_APDS9960();

// Declaracao das variaveis que armazenarao as leituras de luminosidade
uint16_t luz_ambiente = 0;
uint16_t luz_vermelha = 0;
uint16_t luz_verde = 0;
uint16_t luz_azul = 0;

void setup() {
  
  // Inicializacao do monitor serial
  Serial.begin(9600);

  // Inicializacao da comunicacao I2C com o sensor
  if ( apds.init() ) {
    Serial.println("Sensor APDS-9960 iniciado com sucesso!");
  } else {
    Serial.println("Falha na inicializacao do sensor...");
  }
  
  // Start running the APDS-9960 light sensor (no interrupts)
  if ( apds.enableLightSensor(false) ) {
    Serial.println("Sensor configurado para modo de leitura de luz!");
  } else {
    Serial.println("Falha na configuracao de modo de operacao do sensor...");
  }
  
  Serial.println("Aproxime objetos coloridos e tampe o sensor...");

}

void loop() {
  
  // Verifica se ha leitura de luz ambiente disponivel
  if ( apds.readAmbientLight(luz_ambiente) ){
    // Exibe o valor no monitor serial
    Serial.print("Ambiente: ");
    Serial.print(luz_ambiente);
  }
  // Verifica se ha leitura de luz vermelha disponivel
  if ( apds.readRedLight(luz_vermelha) ){
    // Exibe o valor no monitor serial
    Serial.print(" | Vermelho: ");
    Serial.print(luz_vermelha);
  }
  // Verifica se ha leitura de luz verde disponivel
  if ( apds.readGreenLight(luz_verde) ){
    // Exibe o valor no monitor serial
    Serial.print(" | Verde: ");
    Serial.print(luz_verde);
  }
  // Verifica se ha leitura de luz azul disponivel
  if ( apds.readBlueLight(luz_azul) ){
    // Exibe o valor no monitor serial
    Serial.print(" | Azul: ");
    Serial.println(luz_azul);
  }
  
  delay(100);
  
}