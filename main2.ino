#include <Adafruit_NeoPixel.h> //Adiciona a biblioteca
#define D_in 16 //Nomeia o pino 6 do Arduino
#define qtdeLeds 4 //Informa a quantidade de LEDs que serão ligados em cascata
Adafruit_NeoPixel pixels(qtdeLeds, D_in); //Instancia o objeto "pixels", informando a quantidade e o pino de sinal

uint8_t R = 255, G = 255, B = 255;

unsigned int timeX = 1000;

void showColors(uint8_t R, uint8_t G, uint8_t B) {
  for (int i = 0; i < qtdeLeds; i++) {                                             //para i = 0, se i menor que a quantidade de leds, incremente 1 em i
    pixels.setPixelColor(i, pixels.Color(R, G, B));                                //liga LED com uma cor espcífica passada por parametros
    //pixels.setPixelColor(i, pixels.Color(random(255), random(255), random(255)));  //liga LED com uma cor aleatória
    pixels.show();                                                                 //executa os parâmetros do comando acima
    delay(timeX);                                                                  //Aguarda 10 milissegundos
    pixels.clear();                                                                //desliga todos os LEDs
    //pixels.setPixelColor(i, pixels.Color(0, 0, 0));                                //desliga LEDS RGB
    //pixels.show();                                                                 //executa os parâmetros do comando acima
    delay(timeX);                                                                  //Aguarda 10 milissegundos
  }
}

void setup() {
 pinMode(D_in, OUTPUT); //Configura o pino 16 como saída
 pixels.begin(); //Inicia o objeto "pixels"
 pixels.clear(); //desliga todos os LEDs
}
void loop() {
  timeX = random(50)+1;
  //showColors(27, 28, 30);
  showColors(255, 0, 0);
  showColors(0, 255, 0);
  showColors(0, 0, 255);
  showColors(0, 255, 255);
  showColors(255, 255, 0);
  showColors(255, 0, 255);
  //showColors(255, 0, 0);
  showColors(255, 255, 255);
  showColors(150, 0, 75);
}