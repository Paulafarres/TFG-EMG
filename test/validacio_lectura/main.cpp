//Inclusió de llibreries per comunicació SPI i BLE
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

#define WINDOW 10
float emg1_buffer[WINDOW];
float emg2_buffer[WINDOW];
float emg3_buffer[WINDOW];

int index_buffer=0;
bool buffer_full = false;
float emg1_mv=0.0;
float emg2_mv=0.0;
float emg3_mv=0.0;
int32_t emg1_value=0;
int32_t emg2_value=0;
int32_t emg3_value=0;

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

float convertToMillivolts(long rawValue, float vRef = 2.4, int gain = 1) {
  const float fullScale = 8388607.0;
  float voltage = 1000 * (rawValue / fullScale) * (vRef / gain);
  return voltage;
}

void readADS1294Data(){
    digitalWrite(SS,LOW);
    uint8_t status[3];
    uint8_t emg1[3], emg2[3], emg3[3];
    for (int i = 0; i < 3; i++) status[i] = SPI.transfer(0x00);
    for (int i = 0; i < 3; i++) emg1[i] = SPI.transfer(0x00);
    for (int i = 0; i < 3; i++) emg2[i] = SPI.transfer(0x00);
    for (int i = 0; i < 3; i++) emg3[i] = SPI.transfer(0x00);
    digitalWrite(SS,HIGH);

    emg1_value = (emg1[0]<<16) | (emg1[1]<<8) |emg1[2];
    emg2_value = (emg2[0]<<16) | (emg2[1]<<8) |emg2[2];
    emg3_value = (emg3[0]<<16) | (emg3[1]<<8) |emg3[2];

    if(emg1_value & 0x800000) emg1_value |= 0xFF000000;
    if(emg2_value & 0x800000) emg2_value |= 0xFF000000;
    if(emg3_value & 0x800000) emg3_value |= 0xFF000000;

    emg1_mv = convertToMillivolts(emg1_value);
    emg2_mv = convertToMillivolts(emg2_value);
    emg3_mv = convertToMillivolts(emg3_value);

    emg1_buffer[index_buffer] = emg1_mv;
    emg2_buffer[index_buffer] = emg2_mv;
    emg3_buffer[index_buffer] = emg3_mv;

    index_buffer++;

    if (index_buffer >= WINDOW) {
        index_buffer = 0;
        if (deviceConnected) {
            String emg1_str = "", emg2_str = "", emg3_str = "";
            for (int i = 0; i < WINDOW; i++) {
                emg1_str += String(emg1_buffer[i], 2);
                emg2_str += String(emg2_buffer[i], 2);
                emg3_str += String(emg3_buffer[i], 2);
                if (i < WINDOW - 1) {
                    emg1_str += ",";
                    emg2_str += ",";
                    emg3_str += ",";
                }
            }
            String paquet = emg1_str + "\n" + emg2_str + "\n" + emg3_str + "\n";
            pCharacteristic->setValue(paquet.c_str());
            pCharacteristic->notify();
            delay(2);
        }
    }
}

void IRAM_ATTR drdy_isr() {
  novesdades = true;
}
void llegirTotsElsRegistres() {
  Serial.println("Llegeixo registres 0x00 a 0x19:");
  for (uint8_t reg = 0x00; reg <= 0x19; reg++) {
    uint8_t val = ads1294_readRegister(reg);
    Serial.print("Registre 0x");
    if (reg < 0x10) Serial.print("0"); // format hex
    Serial.print(reg, HEX);
    Serial.print(" = 0x");
    if (val < 0x10) Serial.print("0");
    Serial.print(val, HEX);
    Serial.print(" (bin: ");
    Serial.print(val, BIN);
    Serial.println(")");
    delay(10); // per veure millor a Serial
  }
}

void setup() {
    Serial.begin(115200);
    pinMode(MISO, INPUT);
    pinMode(MOSI, OUTPUT);
    pinMode(SS, OUTPUT);
    pinMode(SCLK, OUTPUT);
    pinMode(CLKSEL, OUTPUT);
    pinMode(DRDY, INPUT);
    pinMode(START, OUTPUT);
    pinMode(RESET, OUTPUT);
    pinMode(PWDN, OUTPUT);
    digitalWrite(CLKSEL, HIGH);
    delay(10);

    digitalWrite(SS, HIGH);
    SPI.begin(SCLK, MISO, MOSI, SS);
    SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE1));
    Serial.println("SPI inicialitzat i comunicació iniciada");

    attachInterrupt(digitalPinToInterrupt(DRDY), drdy_isr, FALLING);

    digitalWrite(PWDN, LOW);
    digitalWrite(RESET, LOW);
    delay(1);
    digitalWrite(PWDN, HIGH);
    digitalWrite(RESET, HIGH);
    delay(10);
    delay(50);

    ads1294_writeCommand(CMD_RESET);
    delay(100);
    ads1294_writeCommand(CMD_SDATAC);
    delay(1);
    configRegisters();
    delay(1);
    llegirTotsElsRegistres();
    digitalWrite(START, HIGH);
    delay(1);
    ads1294_writeCommand(CMD_RDATAC);
    Serial.println("Inicialització completada!");

    BLEDevice::init("TFG EMG");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();
    Serial.println("Esperant connexió BLE...");
}

void loop() {
    if (novesdades) {
        readADS1294Data();
        novesdades = false;
    }
}
