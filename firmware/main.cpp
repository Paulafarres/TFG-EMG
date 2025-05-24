/*
  Sistema de lectura EMG amb ESP32 + ADS1294 + BLE

  Aquest codi:
  - Inicialitza l’ADS1294 via SPI.
  - Llegeix 3 canals EMG a 24 bits.
  - Converteix les dades a mV i calcula percentatges d’activació.
  - Envia els valors per BLE cada N mostres.
  - Permet controlar l’estat de lectura i exportar resultats des de la interfície web.

*/

//Inclusió de llibreries per comunicació SPI i BLE
#include <Arduino.h>
#include <SPI.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

using namespace std;

// Variable de control per indicar que hi ha noves dades disponibles des de l'interruptor DRDY
volatile bool novesdades = false;

// Definició dels UUIDs del servei i característica BLE
#define SERVICE_UUID "0af56af5-502d-4244-ae1d-262a9d72945d"
#define CHARACTERISTIC_UUID "4fc085b6-f4a6-4142-ac13-ced0cf3ad0e3"

BLECharacteristic* pCharacteristic;
bool deviceConnected = false;

// Pins utilitzats per la comunicació amb el xip ADS1294
#define MOSI 23
#define MISO 19
#define SCLK 18
#define SS 5

// Pins de control del xip ADS1294
#define CLKSEL 33
#define DRDY 26
#define START 27
#define RESET 14
#define PWDN 12

// Comandes del xip ADS1294
#define CMD_READ_REG 0x20
#define CMD_WRITE_REG 0x40
#define CMD_STOP 0x0A
#define CMD_SDATAC 0x11
#define CMD_RDATAC 0x10
#define CMD_RESET 0x06

// Valors de configuració dels registres del xip ADS1294
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

// Constants de finestra per càlcul i buffers circulars
#define WINDOW 10
float emg1_buffer[WINDOW], emg2_buffer[WINDOW], emg3_buffer[WINDOW];
int index_buffer=0;
bool buffer_full = false;

// Variables per emmagatzemar valors en mV i valors crus
float emg1_mv=0.0, emg2_mv=0.0, emg3_mv=0.0;
int32_t emg1_value=0, emg2_value=0, emg3_value=0;

float max1 = -9999;  // Màxim per al quàdriceps
float max2 = -9999;  // Màxim per als isquios
float max3 = -9999;  // Màxim per al gluti
float perc1, perc2, perc3;  // Percentatges per cada canal

//Control d'estat isRunning , per aturar el bucle de processament si cal, des de fora
bool isRunning = true;

// Classe per callbacks BLE
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

// Funcions per escriure/comunicar amb l'ADS1294
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

//Actualització de màxims i càlculs de percentatges respecte del màxim total
void actualitzarMaxValues(float emg1_value, float emg2_value, float emg3_value) {

  emg1_value = fabs(emg1_value);
  emg2_value = fabs(emg2_value);
  emg3_value = fabs(emg3_value);

  if ( emg1_value > max1) max1 = emg1_value;
  if( emg2_value > max2) max2 = emg2_value;
  if (emg3_value > max3) max3= emg3_value;
  float maxTotal = max1 + max2 + max3;
  if (maxTotal > 0) {
    perc1 = (max1 / maxTotal) * 100;
    perc2 = (max2 / maxTotal) * 100;
    perc3 = (max3 / maxTotal) * 100;
  } else {
    perc1 = perc2 = perc3 = 0;
  }

}

// Funció principal que llegeix dades de l’ADS1294, les processa i les envia per BLE

void readADS1294Data(){

    // Inici de comunicació SPI amb el xip ADS1294
    digitalWrite(SS,LOW);

    // Variables per llegir dades SPI: 3 bytes d’estat + 3x3 bytes (3 canals EMG)
    uint8_t status[3];
    uint8_t emg1[3], emg2[3], emg3[3];
    
    // Llegeix 3 bytes d'estat (no utilitzats aquí però cal fer-ho)
    for (int i = 0; i < 3; i++) status[i] = SPI.transfer(0x00);

    // Llegeix les dades EMG per als tres canals (24 bits = 3 bytes per canal)
    for (int i = 0; i < 3; i++) emg1[i] = SPI.transfer(0x00);
    for (int i = 0; i < 3; i++) emg2[i] = SPI.transfer(0x00);
    for (int i = 0; i < 3; i++) emg3[i] = SPI.transfer(0x00);

    // Fi de la comunicació SPI
    digitalWrite(SS,HIGH);
    
    // Converteix els 3 bytes de cada canal en un valor de 24 bits amb signe
    emg1_value = (emg1[0]<<16) | (emg1[1]<<8) |emg1[2];
    emg2_value = (emg2[0]<<16) | (emg2[1]<<8) |emg2[2];
    emg3_value = (emg3[0]<<16) | (emg3[1]<<8) |emg3[2];

    // Sign extension: si el bit més significatiu és 1 → valor negatiu → completa a 32 bits
    if(emg1_value & 0x800000) emg1_value |= 0xFF000000;
    if(emg2_value & 0x800000) emg2_value |= 0xFF000000;
    if(emg3_value & 0x800000) emg3_value |= 0xFF000000;

    // Conversió dels valors a mil·livolts
    emg1_mv = convertToMillivolts(emg1_value);
    emg2_mv = convertToMillivolts(emg2_value);
    emg3_mv = convertToMillivolts(emg3_value);

    // Actualitza el màxim de cada canal i calcula el percentatge d’activació
    actualitzarMaxValues(emg1_mv,emg2_mv,emg3_mv);

    // Desa els valors als buffers circulars per RMS i transmissió
    if(index_buffer < WINDOW){
      emg1_buffer[index_buffer] = emg1_mv;
      emg2_buffer[index_buffer] = emg2_mv;
      emg3_buffer[index_buffer] = emg3_mv;
      index_buffer++;
    }

    // Si hi ha un dispositiu BLE connectat, envia les dades
    if (deviceConnected) {
      String emg1_str = "", emg2_str = "", emg3_str = "";
      float sum1=0, sum2=0, sum3=0;
      
      // Construeix les cadenes de text amb els valors EMG separats per comes
      for (int i = 0; i < WINDOW; i++) {
        emg1_str += String(emg1_buffer[i], 2);
        emg2_str += String(emg2_buffer[i], 2);
        emg3_str += String(emg3_buffer[i], 2);
        sum1+=emg1_buffer[i];
        sum2+=emg2_buffer[i];
        sum3+=emg3_buffer[i];
        
        if (i < WINDOW - 1) {
          emg1_str += ",";
          emg2_str += ",";
          emg3_str += ",";
        }
      }
    // Prepara el paquet de dades que s’enviarà per BLE
    String paquet = emg1_str + "\n" + emg2_str + "\n" + emg3_str + "\n";
    paquet += String(perc1,2) + "\n" + String(perc2,2) + "\n" + String(perc3,2);

    // Envia el paquet com a notificació BLE
    pCharacteristic->setValue(paquet.c_str());
    pCharacteristic->notify();
    
    // Reinicia l’índex per tornar a omplir el buffer
    index_buffer=0;

  }
}

// Atura l'execució del bucle principal de lectura.
// Aquesta funció pot ser cridada externament.
void stopProgram(){
  isRunning=false;
}

// Interrupció de baix nivell (ISR) vinculada al senyal DRDY (Data Ready) del xip ADS1294.
// Quan arriba una nova mostra (DRDY fa un flanc de baixada), s'activa aquesta ISR.
// La variable 'novesdades' es posa a true per indicar que cal llegir dades.
void IRAM_ATTR drdy_isr() {
  novesdades = true;
}

// Llegeix i imprimeix el valor de tots els registres per debugging
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

    // Inicialització de pins i SPI
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

    // Reinicialització del xip
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

    // Inicialització BLE
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
  // Si s'ha aturat el programa externament (per exemple, des de la web), no fem res
  if(!isRunning) return;

  // Comprova si hi ha noves dades disponibles (interrupció DRDY activa)
  if (novesdades) {
    readADS1294Data(); // Llegeix les dades del xip ADS1294 i actualitza buffers, percentatges i envia per BLE
    novesdades = false;  // Reinicia la bandera per esperar la següent interrupció
  }
}
