//OBJECTIU:
 // - Llegeix senyals EMG de tres músculs mitjançant l'ADS1294 per SPI
 // - Processa el senyal (conversió mV, càlcul del màxim i percentatge d'activació)
 // - Envia dades per BLE a una app web 
 
//ESTRUCTURA FUNCIONAL
 // - Configuració i control del xip ADS1294: definició i inicialització dels pins SPI i control,
      // definició dels registres (high resolution, 3 canals, rellotge intern...). Reset inicial i 
      // es comprova connexió llegint l'ID
 // - Lectura de dades: cada 2ms comprova si hi ha dades disponibles (DRDY), llegeix 3 canals de 3 bytes cadascun
      // es fa la conversió a valors amb signe i es passa de digital a mV. S'actualitzen els màxims i es calcula el
      // % d'activitat relativa de cada múscul. Els valors EMG es guarden en buffers circulars 
 // - Processament de màxims i percentatges. la funció actualitzarMaxValues manté el valor màxim de cada múscul.
      // calcula quin percentatge d'activitat total correspon a cada canal
 // - Comunicació BLE: es configura el servidor amb el nom TFG EMG. Quan el navegador web es connecta s'envien notificacions 
      // cada 100ms amb: 20 valors EMG de cada canal, percentatge d'activació dels 3 músculs. Tot queda concatenat en una cadena
      // de text com: val1,val2,...val50\n
      //              val1,val2,...val50\n
      //              val1,val2,...val50\n
      //              perc1\n
      //              perc2\n
      //              perc3
 //- Loop: cada cicle comprova: si toca llegir dades (cada 2ms), si toca enviar dades per BLE (cada 200ms i només si el buffer té prou mostres)
      // si el sistema està aturat (isRunning=false), s'atura el processament

//Inclusió de llibreries per comunicació SPI i BLE
#include <Arduino.h>
#include <SPI.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

using namespace std;

//Definició de pins i constants

// Assignació de pins SPI per a l'ESP32
#define MOSI 23
#define MISO 19
#define SCLK 18
#define SS 5

// Pins de control
#define CLKSEL 33
#define DRDY 26
#define START 27
#define RESET 14
#define PWDN 12

// Comandes de l'ADS1294
#define CMD_READ_REG 0x20 //0010 0000
#define CMD_WRITE_REG 0x40 //0100 0000
#define CMD_STOP 0x0A //0000 1010
#define CMD_SDATAC 0x11 //0001 0001 stop mode contínu
#define CMD_RDATAC 0x10 //0001 0000 mode contínu
#define CMD_RESET 0x06 //0000 011


// Registres i valors predefinits
#define ID_REG 0x00
#define ID_VALUE 0x90 //1001 0000 (100=ADS129x family, 10, 000 4-channel)

#define CONFIG1_REG 0x01
#define CONFIG1_VALUE 0xC4 //1100 0100 (1=HR mode, 1=multiple readback, 0=oscillator clock output disabled,0, 100=2kSPS fmod/256)

#define CONFIG2_REG 0x02
#define CONFIG2_VALUE 0x40 //0100 0000 RESET value

#define CONFIG3_REG 0x03
#define CONFIG3_VALUE 0xCC //1100 1100 (1=internal ref. buffer on, 1, 0=vrefp 2.4V, 0=no mesurar senyal rld en un canal, 1=rldref internally on, 1=rld buffer on, 0=rld sense disabled, 0=rld is connected)

#define LOFF_REG 0x04
#define LOFF_VALUE 0x00 //0000 0000 RESET value

#define CH1SET_REG 0x05
#define CH1SET_VALUE 0x60 //0110 0000 (0=Normal operation, 110=pga c1 12, 0, 000=normal electrode input)

#define CH2SET_REG 0x06
#define CH2SET_VALUE 0x60 //0110 0000 (0=Normal operation, 110=pga c2 12, 0, 000=normal electrode input)

#define CH3SET_REG 0x07
#define CH3SET_VALUE 0x60 //0110 0000 (0=Normal operation, 110=pga c3 12, 0, 000=normal electrode input)

#define CH4SET_REG 0x08
#define CH4SET_VALUE 0x00 //reset value

#define RLD_SENSP_REG 0x0D
#define RLD_SENSP_VALUE 0x07 //0000 0111 (0000=canals 5/6/7/8, 0=canal 4 disabled, 111= canal 1/2/3 enabled)

#define RLD_SENSN_REG 0x0E
#define RLD_SENSN_VALUE 0x07 //0000 0111 (0000=canals 5/6/7/8, 0=canal 4 disabled, 111= canal 1/2/3 enabled)

#define LOFF_SENSP_REG 0x0F
#define LOFF_SENSP_VALUE 0x00 //reset value

#define LOFF_SENSN_REG 0x10
#define LOFF_SENSN_VALUE 0x00 //reset value

#define LOFF_FLIP_REG 0x11
#define LOFF_FLIP_VALUE 0x00 //reset value

#define LOFF_STATP_REG 0x12
#define LOFF_STATP_VALUE 0x00 //reset value

#define LOFF_STATN_REG 0x13
#define LOFF_STATN_VALUE 0x00 //reset value

#define GPIO_REG 0x14
#define GPIO_VALUE 0x0F //Reset value

#define PACE_REG 0x15
#define PACE_VALUE 0x00 //Reset value

#define RESP_REG 0x16
#define RESP_VALUE 0x00 //Reset value

#define CONFIG4_REG 0x17
#define CONFIG4_VALUE 0x00

#define WCT1_REG 0x18
#define WCT1_VALUE 0x00 //Reset value

#define WCT2_REG 0x19
#define WCT2_VALUE 0x00 //Reset value

//BLE UUIDs
#define SERVICE_UUID "0af56af5-502d-4244-ae1d-262a9d72945d"
#define CHARACTERISTIC_UUID "4fc085b6-f4a6-4142-ac13-ced0cf3ad0e3"

//Buffers
const int BUFFER = 20;
float emg1_buffer[BUFFER];
float emg2_buffer[BUFFER];
float emg3_buffer[BUFFER];
int index_buffer = 0;

//Variables de temps
unsigned long ultimalectura = 0;
unsigned long ultimenvio = 0;
const unsigned long lecturadades = 2;
const unsigned long enviodades = 100;

float max1 = -9999;  // Màxim per al quàdriceps
float max2 = -9999;  // Màxim per als isquios
float max3 = -9999;  // Màxim per al gluti
float perc1, perc2, perc3;  // Percentatges per cada canal

//Control d'estat isRunning , per aturar el bucle de processament si cal, des de fora
bool isRunning = true;

//Estat i BLE
BLEServer* pServer = NULL;
BLECharacteristic * pCharacteristic;

bool deviceConnected = false;
bool oldDeviceConnected = false;

//Mostra quan està connectat / desconnectat per Serial
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("Connectat");
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("Desconnectat");
  }
};

//FUNCIONS DE CONTROL DE L'ADS1294

// Escriure una comanda a l'ADS1294
void ads1294_writeCommand(uint8_t cmd) {
    digitalWrite(SS, LOW);
    delayMicroseconds(2);
    SPI.transfer(cmd);
    delayMicroseconds(2);
    digitalWrite(SS, HIGH);
    delay(1);
}

// Llegir un registre de l'ADS1294
uint8_t ads1294_readRegister(uint8_t reg_addr) {
  digitalWrite(SS, LOW);
  delayMicroseconds(2);
  SPI.transfer(CMD_READ_REG | reg_addr);
  SPI.transfer(0x00);  // Nombre de registres a llegir -1 (0 = 1 registre)
  uint8_t value = SPI.transfer(0x00);
  delayMicroseconds(2);
  digitalWrite(SS, HIGH);
  delay(1);
  return value;
}

// Escriure a un registre de l'ADS1294
void ads1294_writeRegister(uint8_t reg_addr, uint8_t value) {
    digitalWrite(SS, LOW);
    delayMicroseconds(2);
    SPI.transfer(CMD_WRITE_REG | reg_addr);  // Comanda per escriure
    SPI.transfer(0x00);  // Nombre de registres a escriure -1 (0 = 1 registre)
    SPI.transfer(value);
    delayMicroseconds(2);
    digitalWrite(SS, HIGH);
    delay(1);
}

//Configura el xip ADS1294
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
    ads1294_writeRegister(GPIO_REG,GPIO_VALUE);
    ads1294_writeRegister(PACE_REG,PACE_VALUE);
    ads1294_writeRegister(RESP_REG,RESP_VALUE);
    ads1294_writeRegister(CONFIG4_REG,CONFIG4_VALUE);
    ads1294_writeRegister(WCT1_REG,WCT1_VALUE);
    ads1294_writeRegister(WCT2_REG,WCT2_VALUE);
    delayMicroseconds(2);
    digitalWrite(SS,HIGH);
}

//Comprova la connexió del xip ADS1294
void checkConnection(){
  for (int i = 0; i < 10; i++) {
    uint8_t id = ads1294_readRegister(ID_REG);
    Serial.print("ID raw [");
    Serial.print(i);
    Serial.print("]: 0x");
    Serial.print(id, HEX);
    Serial.print(" - bin: ");
    Serial.println(id, BIN);
    delay(100);
  }
}

//Converteix de digital a mV
float convertToMillivolts(long rawValue, float vRef = 2.4, int gain = 12) {
  const float fullScale = 8388607.0;  // 2^23 - 1 (màxim valor en 24 bits)
  float voltage = (rawValue / fullScale) * (vRef/ gain);  // en Volts
  return voltage * 1000.0;  // en mV
}

//FUNCIONS DE PROCESSAMENT DE SENYAL

//Actualització de màxims i càlculs de percentatges respecte del màxim total
void actualitzarMaxValues(float emg1_value, float emg2_value, float emg3_value) {

  emg1_value = fabs(emg1_value);
  emg2_value = fabs(emg2_value);
  emg3_value = fabs(emg3_value);

  if ( emg1_value > max1) max1 = emg1_value;
  if( emg2_value > max2) max2 = emg2_value;
  if (emg3_value > max3) max3= emg3_value;

  float maxTotal= max1 + max2 + max3;

  perc1 = (max1/maxTotal)*100;
  perc2 = (max2/maxTotal)*100;
  perc3 = (max3/maxTotal)*100;

}

//Llegir dades de l'ADS1294
void readADS1294Data(){
    while (digitalRead(DRDY)==HIGH);

    digitalWrite(SS,LOW);

    uint8_t status[3]; //3 bytes d'estat
    uint8_t emg1[3], emg2[3], emg3[3]; //3 bytes per cada canal

    //Llegir els bytes ESTAT
    for (int i =0; i<3; i++){
        status[i]= SPI.transfer(0x00);
    }
    //Llegir 3 canals REALS
    for (int i=0; i<3; i++){ emg1[i]= SPI.transfer(0x00);}
    for (int i=0; i<3; i++){ emg2[i]= SPI.transfer(0x00);}
    for (int i=0; i<3; i++){ emg3[i]= SPI.transfer(0x00);}
    digitalWrite(SS,HIGH);
  
    //Convertim 3 bytes en valors de 24 bits
    int32_t emg1_value = (emg1[0]<<16) | (emg1[1]<<8) |emg1[2];
    int32_t emg2_value = (emg2[0]<<16) | (emg2[1]<<8) |emg2[2];
    int32_t emg3_value = (emg3[0]<<16) | (emg3[1]<<8) |emg3[2];

    //Assegurar que si el num de 24 bits era negatiu, tb sigui en 32 bits
    if(emg1_value & 0x800000) emg1_value|=0xFF000000; //si el MSB està a 1= negatiu. si es nagtiu omplim 8 bits restants amb 1
    //800000=1000 0000 0000 0000 0000 0000   i FF000000=1111 1111 0000 0000 0000 0000 0000 0000
    if (emg2_value & 0x800000) emg2_value |= 0xFF000000;
    if (emg3_value & 0x800000) emg3_value |= 0xFF000000;

    //PROVAR DINS ANIRA EMG1_VALUE...
    float emg1_mv = convertToMillivolts(emg1_value); 
    float emg2_mv = convertToMillivolts(emg2_value);
    float emg3_mv = convertToMillivolts(emg3_value);

    actualitzarMaxValues(emg1_mv,emg2_mv,emg3_mv);

    // Guardem els valors al buffer
    if(index_buffer < BUFFER){
      emg1_buffer[index_buffer] = emg1_mv;
      emg2_buffer[index_buffer] = emg2_mv;
      emg3_buffer[index_buffer] = emg3_mv;
      index_buffer++;
    }    
}

//FUNCIONS DE COMUNICACIÓ BLE

//Notifica dades per BLE
void enviardades(){
  unsigned long temps = millis();
  float seconds = temps/1000.0;
  String emg1_str="", emg2_str="", emg3_str="";
  float sum1=0, sum2=0, sum3=0;

  for (int i =0; i<BUFFER; i++) {
    emg1_str += String(emg1_buffer[i],2);
    emg2_str += String(emg2_buffer[i],2);
    emg3_str += String(emg3_buffer[i],2);
    sum1+=emg1_buffer[i];
    sum2+=emg2_buffer[i];
    sum3+=emg3_buffer[i];

    if (i<BUFFER-1){
      emg1_str += ",";
      emg2_str += ",";
      emg3_str += ",";
    }
  }
  String paquet =  emg1_str + "\n" + emg2_str + "\n" +emg3_str + "\n";
  paquet += String(perc1,2) + "\n" + String(perc2,2) + "\n" + String(perc3,2);

  pCharacteristic->setValue(paquet.c_str());
  pCharacteristic->notify();
  index_buffer=0;
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


//CONTROL D'ESTAT

void stopPorgram(){
  isRunning=false;
}

//SETUP() I LOOP()

void setup() {
    Serial.begin(115200);

    // Configuració dels pins SPI
    pinMode(MISO,INPUT);
    pinMode(MOSI,OUTPUT);
    pinMode(SS, OUTPUT);
    pinMode(SCLK,OUTPUT);

    //Cofgiuració dels pins de control
    pinMode(CLKSEL, OUTPUT);
    pinMode(DRDY, INPUT);
    pinMode(START, OUTPUT);
    pinMode(RESET, OUTPUT);
    pinMode(PWDN, OUTPUT);
  
  
    digitalWrite(CLKSEL, HIGH);
    delay(10);

    //Inicialitzem SPI
    digitalWrite(SS, HIGH); //SS alt 
    SPI.begin(SCLK, MISO, MOSI, SS);  // Iniciar SPI amb els pins correctes
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST,SPI_MODE1));
    Serial.println("SPI inicialitzat i comunicació iniciada");


    digitalWrite(PWDN, LOW);
    digitalWrite(RESET, LOW);
    delay(1);
    digitalWrite(PWDN, HIGH);
    digitalWrite(RESET, HIGH);
    delay(10);
   
    // Esperar que les fonts vcap1 vcap2 estiguin estabilitzades
    delay(50);

    // Reset del xip
    ads1294_writeCommand(CMD_RESET);
    delay(100);

    //Desactivar la lectura contínua abans de configurar registres
    ads1294_writeCommand(CMD_SDATAC);
    delay(1);

     // Comprovar ID per validar la connexió
    checkConnection();
    
    //Configurar registres
    configRegisters();
    delay(1);
    
    llegirTotsElsRegistres();

    // Activar senyal START
    digitalWrite(START, HIGH);
    delay(1);

    //Activar mode lectura contínua
    ads1294_writeCommand(CMD_RDATAC);
    Serial.println("Inicialització completada!");

    //BLE
    Serial.println("Iniciant BLE...");
    BLEDevice::init("TFG EMG");
    pServer=BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);
    pCharacteristic=pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_NOTIFY |
      BLECharacteristic::PROPERTY_INDICATE
    );
    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();

    //ADVERTISING
    BLEAdvertising *pAdvertising=BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    BLEDevice::startAdvertising();
    Serial.println("BLE iniciat i esperant connexió");
}

void loop() {
  unsigned long now = millis();

  if(!isRunning){
    return;
  }

  if(now - ultimalectura >= lecturadades){ //Lectura de dades cada 2ms
    ultimalectura = now;
    readADS1294Data();
  }

  if((deviceConnected) && now - ultimenvio >= enviodades && index_buffer >= BUFFER){ //Envio dades si està connectat, cada 200ms i si tenim 20 mostres
    ultimenvio = now;
    enviardades();
  }

}




