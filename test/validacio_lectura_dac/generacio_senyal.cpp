#include <Arduino.h>


const int samplesPerSecond = 250;
const int delay_us = 1000000 / samplesPerSecond;


const float offset = 1650.0; // 1.65 V per centrar la senyal al voltant de 0
const float baseline_noise = 0.005; // soroll de fons en mV
const float emg_ampl = 50.0; // amplitud màxima d'activació
const float active_duration = 1.0; 
const float rest_duration = 3.0;   


bool activat = false;
unsigned long last_switch = 0;
unsigned long durada = 0;


void setup() {
  Serial.begin(115200);
  randomSeed(analogRead(0)); // Inicialitza aleatorietat
}


void loop() {
  unsigned long now = millis();


  // Alternem entre contracció i descans
  if (now - last_switch > durada * 1000) {
    activat = !activat;
    last_switch = now;
    durada = activat ? active_duration : rest_duration;
  }


  // Simula senyal EMG com soroll aleatori centrat a offset
  float valor_mV = offset;
  if (activat) {
    valor_mV += (random(-1000, 1000) / 1000.0) * emg_ampl;
  } else {
    valor_mV += (random(-1000, 1000) / 1000.0) * baseline_noise;
  }


  // Limita a 0–3.3V per DAC (en mV)
  valor_mV = constrain(valor_mV, 0, 3300);
  uint8_t dac_val = round(valor_mV * 255.0 / 3300.0);


  dacWrite(25, dac_val); // GPIO25 → INxP
  dacWrite(26, 255 - dac_val); // INxN (oposat)


  delayMicroseconds(delay_us);
}
