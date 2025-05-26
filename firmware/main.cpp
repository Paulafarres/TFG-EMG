//LLIBRERIES PER SPI, BLE, FILTRAT DIGITAL
#include <Arduino.h>
#include <SPI.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <math.h>        
#include <Filters.h>
#include <Filters/Notch.hpp> 

//UUIDs i gestió BLE 
#define SERVICE_UUID "0af56af5-502d-4244-ae1d-262a9d72945d"
#define CHARACTERISTIC_UUID "4fc085b6-f4a6-4142-ac13-ced0cf3ad0e3"
BLECharacteristic* pCharacteristic;
bool deviceConnected = false;

//Pins i comandes per ADS1294
#define MOSI 23
#define MISO 19
#define SCLK 18
#define SS 5
#define CLKSEL 33
#define DRDY 26
#define START 27
#define RESET 14
#define PWDN 12

#define CMD_READ_REG 0x20
#define CMD_WRITE_REG 0x40
#define CMD_RESET 0x06
#define CMD_SDATAC 0x11
#define CMD_RDATAC 0x10

//Registres de configuració ADS1294
#define CONFIG1_REG 0x01
#define CONFIG1_VALUE 0xC6
#define CONFIG2_REG 0x02
#define CONFIG2_VALUE 0x40
#define CONFIG3_REG 0x03
#define CONFIG3_VALUE 0xE0
#define LOFF_REG 0x04
#define LOFF_VALUE 0x00
#define CH1SET_REG 0x05
#define CH1SET_VALUE 0x60
#define CH2SET_REG 0x06
#define CH2SET_VALUE 0x60
#define CH3SET_REG 0x07
#define CH3SET_VALUE 0x60
#define CH4SET_REG 0x08
#define CH4SET_VALUE 0x00
#define RLD_SENSP_REG 0x0D
#define RLD_SENSP_VALUE 0x07
#define RLD_SENSN_REG 0x0E
#define RLD_SENSN_VALUE 0x07
#define GPIO_REG_ADS 0x14
#define GPIO_VALUE_ADS 0x0F
#define CONFIG4_REG 0x17
#define CONFIG4_VALUE 0x00
#define WCT1_REG 0x18
#define WCT1_VALUE 0x00
#define WCT2_REG 0x19
#define WCT2_VALUE 0x00        

//VARIABLES GLOBALS 

//Filtrat digital Notch (50Hz)
const float fs = 500.0;      // Sampling frequency (Hz)
const float f0 = 50.0;       // Notch frequency (Hz)
const float fn = 2 * f0 / fs; // Normalized frequency
// Filtre FIR Notch per 50 Hz
auto notchFilter1 = simpleNotchFIR(fn);
auto notchFilter2 = simpleNotchFIR(fn);
auto notchFilter3 = simpleNotchFIR(fn);

bool isRunning = true;
volatile bool novaMostra = false;

//Buffers i finestres mòbils
#define N_CANALS 3
#define RMS_WINDOW 30
#define OFFSET_WINDOW 50
#define MOVING_AVG_WINDOW 5

//offset_buffer: elimina l'offset del senyal amb mitjana mòbil
float offset_buffer[N_CANALS][OFFSET_WINDOW] = {0}; 
int offset_index[N_CANALS] = {0};
float offset_sum[N_CANALS] = {0};

//moving_avg_buffer: suavitza el senyal (filtrat passa-baix)
float moving_avg_buffer[N_CANALS][MOVING_AVG_WINDOW] = {0};
int moving_avg_index[N_CANALS] = {0};

//rms_buffer: càlcul de la RMS amb finestra desplaçada 30 mostres
float rms_buffer[N_CANALS][RMS_WINDOW] = {0};
int rms_index[N_CANALS] = {0};
bool rms_plena[N_CANALS] = {false};
float max_rms[N_CANALS] = {0};

unsigned long lastSent = 0;
const unsigned long BLE_INTERVAL = 100; // mil·lisegons

void stopProgram(){
  isRunning=false;
}

//BLE: connexió i callbacks
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("BLE connectat");
  }
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("BLE desconnectat");
  }
};

//Funcions d'enviament SPI
void ads1294_writeCommand(uint8_t cmd) {
  digitalWrite(SS, LOW); delayMicroseconds(2);
  SPI.transfer(cmd);
  delayMicroseconds(2); digitalWrite(SS, HIGH); delay(1);
}

void ads1294_writeRegister(uint8_t reg_addr, uint8_t value) {
  digitalWrite(SS, LOW); delayMicroseconds(2);
  SPI.transfer(CMD_WRITE_REG | reg_addr);
  SPI.transfer(0x00); SPI.transfer(value);
  delayMicroseconds(2); digitalWrite(SS, HIGH); delay(1);
}

//Configuració completa dels registres
void configRegisters() {
  digitalWrite(SS, LOW); delayMicroseconds(2);
  ads1294_writeRegister(CONFIG1_REG, CONFIG1_VALUE);
  ads1294_writeRegister(CONFIG2_REG, CONFIG2_VALUE);
  ads1294_writeRegister(CONFIG3_REG, CONFIG3_VALUE);
  ads1294_writeRegister(LOFF_REG, LOFF_VALUE);
  ads1294_writeRegister(CH1SET_REG, CH1SET_VALUE);
  ads1294_writeRegister(CH2SET_REG, CH2SET_VALUE);
  ads1294_writeRegister(CH3SET_REG, CH3SET_VALUE);
  ads1294_writeRegister(CH4SET_REG, CH4SET_VALUE);
  ads1294_writeRegister(RLD_SENSP_REG, RLD_SENSP_VALUE);
  ads1294_writeRegister(RLD_SENSN_REG, RLD_SENSN_VALUE);
  ads1294_writeRegister(GPIO_REG_ADS, GPIO_VALUE_ADS);
  ads1294_writeRegister(CONFIG4_REG, CONFIG4_VALUE);
  ads1294_writeRegister(WCT1_REG, WCT1_VALUE);
  ads1294_writeRegister(WCT2_REG, WCT2_VALUE);
  delayMicroseconds(2); digitalWrite(SS, HIGH);
}

//Conversió i processament
float convertToMillivolts(int32_t valor, float vref = 2.4, int gain = 1) {
  const float FS = 8388607.0;
  return (valor * vref * 1000.0) / (FS * gain);
}

float removeOffset(int canal, float x) {
  offset_sum[canal] -= offset_buffer[canal][offset_index[canal]];
  offset_buffer[canal][offset_index[canal]] = x;
  offset_sum[canal] += x;
  offset_index[canal] = (offset_index[canal] + 1) % OFFSET_WINDOW;
  return x - (offset_sum[canal] / OFFSET_WINDOW);
}

float moving_average(int canal, float nova) {
  moving_avg_buffer[canal][moving_avg_index[canal]] = nova;
  moving_avg_index[canal] = (moving_avg_index[canal] + 1) % MOVING_AVG_WINDOW;
  float suma = 0;
  for (int j = 0; j < MOVING_AVG_WINDOW; j++) suma += moving_avg_buffer[canal][j];
  return suma / MOVING_AVG_WINDOW;
}

float calcularRMS(int canal, float nova) {
  rms_buffer[canal][rms_index[canal]] = nova;
  rms_index[canal] = (rms_index[canal] + 1) % RMS_WINDOW;
  if (rms_index[canal] == 0) rms_plena[canal] = true;
  int N = rms_plena[canal] ? RMS_WINDOW : rms_index[canal];
  float suma = 0;
  for (int i = 0; i < N; i++) suma += rms_buffer[canal][i] * rms_buffer[canal][i];
  float rms= sqrt(suma / N);
  if (rms > max_rms[canal]) max_rms[canal] = rms;
  return rms;
}

//Interrupció DRDY
void IRAM_ATTR drdyISR() {
  novaMostra = true;
}

//SETUP() Inicialització
void setup() {
  //Configura l'ADS1294
  Serial.begin(115200);
  pinMode(MISO, INPUT); pinMode(MOSI, OUTPUT);
  pinMode(SCLK, OUTPUT); pinMode(SS, OUTPUT);
  pinMode(DRDY, INPUT); pinMode(RESET, OUTPUT);
  pinMode(PWDN, OUTPUT); pinMode(START, OUTPUT);
  pinMode(CLKSEL, OUTPUT); digitalWrite(CLKSEL, HIGH);

  digitalWrite(PWDN, LOW); delay(1);
  digitalWrite(PWDN, HIGH); delay(10);
  digitalWrite(RESET, LOW); delay(1);
  digitalWrite(RESET, HIGH); delay(10);

  SPI.begin(SCLK, MISO, MOSI, SS);
  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE1));
  ads1294_writeCommand(CMD_RESET); delay(100);
  ads1294_writeCommand(CMD_SDATAC); delay(1);
  configRegisters(); delay(1);
  digitalWrite(START, HIGH); delay(1);
  ads1294_writeCommand(CMD_RDATAC);
  attachInterrupt(digitalPinToInterrupt(DRDY), drdyISR, FALLING);

  //Inicialitza BLE
  BLEDevice::init("TFG EMG");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  BLE2902* desc = new BLE2902();
  desc->setNotifications(true);
  pCharacteristic->addDescriptor(desc);
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();
}


//LOOP(): Si hi ha nova mostra es fa: lectura via SPI, conversió a mV, filtartge notch, eliminació offset, filtratge amb
//mitjana mòbil, càlcul RMS. Si han passat més de 100ms i hi ha connexió BLE: calcula percentatge per canal i envia dades BLE
// (RMS1,RMS2,RMS3 + % d'activació)
void loop() {
  if(!isRunning) return;
  if (novaMostra) {
    novaMostra = false;
    uint8_t dades[12];
    digitalWrite(SS, LOW);
    for (int i = 0; i < 12; i++) dades[i] = SPI.transfer(0x00);
    digitalWrite(SS, HIGH);

    float rms_vals[N_CANALS];
    for (int canal = 0; canal < N_CANALS; canal++) {
      int base = 3 + canal * 3;
      int32_t valor = ((uint32_t)dades[base] << 16) | ((uint32_t)dades[base + 1] << 8) | dades[base + 2];
      if (valor & 0x800000) valor |= 0xFF000000;
      float mv = convertToMillivolts(valor);         
      float mv_filtrat;
      if (canal == 0) mv_filtrat = notchFilter1(mv);
      else if (canal == 1) mv_filtrat = notchFilter2(mv);
      else if (canal == 2) mv_filtrat = notchFilter3(mv);
      float x = removeOffset(canal, mv_filtrat);  
      x = moving_average(canal, x);
      rms_vals[canal] = calcularRMS(canal, x);
    }

    if (deviceConnected && millis() - lastSent >= BLE_INTERVAL) {
      lastSent = millis();
      
      float perc_vals[N_CANALS];
      float total_rms = rms_vals[0] + rms_vals[1] + rms_vals[2];
      for (int i = 0; i < N_CANALS; i++) {
      perc_vals[i] = (total_rms > 0) ? (rms_vals[i] / total_rms) * 100.0 : 0.0;
      }

      char buffer[100];
      //Format enviament ble: paquet de text amb 4 línies
      // Línia 1: RMS dels 3 canals separats per comes
      //Línies 2-4: Percentatges (1 per línia)
      snprintf(buffer, sizeof(buffer), "%.2f,%.2f,%.2f\n%.2f\n%.2f\n%.2f\n", rms_vals[0], rms_vals[1], rms_vals[2], perc_vals[0], perc_vals[1], perc_vals[2]);
      pCharacteristic->setValue(buffer);
      pCharacteristic->notify();
    }
  }
}
