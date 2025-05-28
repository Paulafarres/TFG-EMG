#include <Arduino.h>
#include <SPI.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

using namespace std;
volatile bool novesdades = false;

#define SERVICE_UUID "0af56af5-502d-4244-ae1d-262a9d72945d"
#define CHARACTERISTIC_UUID "4fc085b6-f4a6-4142-ac13-ced0cf3ad0e3"

BLECharacteristic* pCharacteristic;
bool deviceConnected = false;

//Definició de pins i constants
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
#define CMD_STOP 0x0A
#define CMD_SDATAC 0x11
#define CMD_RDATAC 0x10
#define CMD_RESET 0x06

#define CONFIG1_REG 0x01
#define CONFIG1_VALUE 0xC6
#define CONFIG2_REG 0x02
#define CONFIG2_VALUE 0x40
#define CONFIG3_REG 0x03
#define CONFIG3_VALUE 0xE0
#define LOFF_REG 0x04
#define LOFF_VALUE 0x00
#define CH1SET_REG 0x05
#define CH1SET_VALUE 0x50
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
#define LOFF_SENSP_REG 0x0F
#define LOFF_SENSP_VALUE 0x00
#define LOFF_SENSN_REG 0x10
#define LOFF_SENSN_VALUE 0x00
#define LOFF_FLIP_REG 0x11
#define LOFF_FLIP_VALUE 0x00
#define LOFF_STATP_REG 0x12
#define LOFF_STATP_VALUE 0x00
#define LOFF_STATN_REG 0x13
#define LOFF_STATN_VALUE 0x00
#define GPIO_REG_ADS 0x14
#define GPIO_VALUE_ADS 0x0F
#define PACE_REG 0x15
#define PACE_VALUE 0x00
#define RESP_REG 0x16
#define RESP_VALUE 0x00
#define CONFIG4_REG 0x17
#define CONFIG4_VALUE 0x00
#define WCT1_REG 0x18
#define WCT1_VALUE 0x00
#define WCT2_REG 0x19
#define WCT2_VALUE 0x00

volatile bool novaMostra = false;
void ads1294_writeCommand(uint8_t cmd) {
    digitalWrite(SS, LOW);
    delayMicroseconds(2);
    SPI.transfer(cmd);
    delayMicroseconds(2);
    digitalWrite(SS, HIGH);
    delay(1);
}

uint8_t ads1294_readRegister(uint8_t reg_addr) {
  digitalWrite(SS, LOW);
  delayMicroseconds(2);
  SPI.transfer(CMD_READ_REG | reg_addr);
  SPI.transfer(0x00);
  uint8_t value = SPI.transfer(0x00);
  delayMicroseconds(2);
  digitalWrite(SS, HIGH);
  delay(1);
  return value;
}

void ads1294_writeRegister(uint8_t reg_addr, uint8_t value) {
    digitalWrite(SS, LOW);
    delayMicroseconds(2);
    SPI.transfer(CMD_WRITE_REG | reg_addr);
    SPI.transfer(0x00);
    SPI.transfer(value);
    delayMicroseconds(2);
    digitalWrite(SS, HIGH);
    delay(1);
}

void configRegisters(){
    digitalWrite(SS,LOW);
    delayMicroseconds(2);
    ads1294_writeRegister(CONFIG1_REG, CONFIG1_VALUE);
    ads1294_writeRegister(CONFIG2_REG, CONFIG2_VALUE);
    ads1294_writeRegister(CONFIG3_REG,CONFIG3_VALUE);
    ads1294_writeRegister(LOFF_REG, LOFF_VALUE);
    ads1294_writeRegister(CH1SET_REG,CH1SET_VALUE);
    ads1294_writeRegister(CH2SET_REG,CH2SET_VALUE);
    ads1294_writeRegister(CH3SET_REG,CH3SET_VALUE);
    ads1294_writeRegister(CH4SET_REG,CH4SET_VALUE);
    ads1294_writeRegister(RLD_SENSP_REG,RLD_SENSP_VALUE);
    ads1294_writeRegister(RLD_SENSN_REG,RLD_SENSN_VALUE);
    ads1294_writeRegister(LOFF_SENSP_REG,LOFF_SENSP_VALUE);
    ads1294_writeRegister(LOFF_SENSN_REG,LOFF_SENSN_VALUE);
    ads1294_writeRegister(LOFF_FLIP_REG,LOFF_FLIP_VALUE);
    ads1294_writeRegister(LOFF_STATP_REG,LOFF_STATP_VALUE);
    ads1294_writeRegister(LOFF_STATN_REG,LOFF_STATN_VALUE);
    ads1294_writeRegister(GPIO_REG_ADS,GPIO_VALUE_ADS);
    ads1294_writeRegister(PACE_REG,PACE_VALUE);
    ads1294_writeRegister(RESP_REG,RESP_VALUE);
    ads1294_writeRegister(CONFIG4_REG,CONFIG4_VALUE);
    ads1294_writeRegister(WCT1_REG,WCT1_VALUE);
    ads1294_writeRegister(WCT2_REG,WCT2_VALUE);
    delayMicroseconds(2);
    digitalWrite(SS,HIGH);
}

// --- Conversió a mil·livolts ---
float convertToMillivolts(int32_t valor, float vref = 2.4, int gain = 1) {
  const float FS = 8388607.0; // 2^23-1
  return (valor * vref * 1000.0) / (FS * gain);
}

#define OFFSET_WINDOW 50
float offset_buffer[OFFSET_WINDOW];
int offset_index = 0;
float offset_sum = 0;

float removeOffset(float x) {
  offset_sum -= offset_buffer[offset_index];
  offset_buffer[offset_index] = x;
  offset_sum += x;
  offset_index = (offset_index + 1) % OFFSET_WINDOW;
  return x - (offset_sum / OFFSET_WINDOW);
}


#define N 5
float buffer[N];
int i = 0;

float moving_average(float nova) {
  buffer[i] = nova;
  i = (i + 1) % N;
  float suma = 0;
  for (int j = 0; j < N; j++) suma += buffer[j];
  return suma / N;
}


// --- ISR per DRDY ---
void IRAM_ATTR drdyISR() {
  novaMostra = true;
}

void setup() {
  Serial.begin(115200);

  pinMode(MISO, INPUT);
  pinMode(MOSI, OUTPUT);
  pinMode(SCLK, OUTPUT);
  pinMode(SS, OUTPUT);
  pinMode(DRDY, INPUT);
  pinMode(RESET, OUTPUT);
  pinMode(PWDN, OUTPUT);
  pinMode(START, OUTPUT);
  pinMode(CLKSEL, OUTPUT);
  digitalWrite(CLKSEL, HIGH); // usa cristall intern

  digitalWrite(PWDN, LOW); delay(1);
  digitalWrite(PWDN, HIGH); delay(10);
  digitalWrite(RESET, LOW); delay(1);
  digitalWrite(RESET, HIGH); delay(10);

  SPI.begin(SCLK, MISO, MOSI, SS);
  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE1));

  ads1294_writeCommand(CMD_RESET);
  delay(100);
  configRegisters();

  digitalWrite(START, HIGH);
  delay(1);

  attachInterrupt(digitalPinToInterrupt(DRDY), drdyISR, FALLING);
}

void loop() {
  if (novaMostra) {
    novaMostra = false;

    uint8_t status[3], canal1[3];

    digitalWrite(SS, LOW);
    for (int i = 0; i < 3; i++) SPI.transfer(0x00);  // STATUS
    for (int i = 0; i < 3; i++) canal1[i] = SPI.transfer(0x00);
    digitalWrite(SS, HIGH);

    int32_t valor = ((uint32_t)canal1[0] << 16) | ((uint32_t)canal1[1] << 8) | canal1[2];
    if (valor & 0x800000) valor |= 0xFF000000; // Signe
    float mv = convertToMillivolts(valor);
float mv_final = removeOffset(mv);
float mv_suau = moving_average(mv_final);
Serial.println(mv_suau);

  }
}
