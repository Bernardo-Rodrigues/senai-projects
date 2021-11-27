#include <VirtualWire.h>

#define pinRF  12


struct Dados {
  int distancia;
};

Dados Sensor; 

uint8_t buf[sizeof(Sensor)];
uint8_t buflen = sizeof(Sensor);

void setup() { 

  vw_set_rx_pin(pinRF);
  vw_setup(2000);   
  vw_rx_start();

  Serial.begin(9600);
}

void loop() {

  if ( vw_have_message() ) {
    vw_get_message(buf, &buflen);
    memcpy(&Sensor,&buf,buflen);


   Serial.print(Sensor.distancia);
   Serial.println(" cm ");

  }  
}
