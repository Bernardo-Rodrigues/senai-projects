#include <VirtualWire.h>
#include <NewPing.h>

#define TRIGGER  10  
#define ECHO     11 
#define MAX_DISTANCE 450 
#define pinRF  12

NewPing sonar(TRIGGER, ECHO, MAX_DISTANCE); 

struct Dados {
  int distancia;
};

Dados Sensor; 

void setup() {

  vw_set_tx_pin(pinRF);
  vw_set_ptt_inverted(true);
  vw_setup(2000);

  Serial.begin(9600);
}

void loop() {
  
  int  distanciaCm = sonar.ping_cm();
  
  Sensor.distancia = distanciaCm;
  
  Serial.print(Sensor.distancia);
  Serial.println(" cm  ");
  
  vw_send((uint8_t *)&Sensor, sizeof(Sensor));
  vw_wait_tx(); 
  
  delay(1000);
}
