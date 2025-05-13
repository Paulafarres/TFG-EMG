//Inclusió de llibreries per comunicació SPI i BLE
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

using namespace std;

//BLE UUIDs
#define SERVICE_UUID "0af56af5-502d-4244-ae1d-262a9d72945d"
#define CHARACTERISTIC_UUID "4fc085b6-f4a6-4142-ac13-ced0cf3ad0e3"

//Buffers i variables d'adquisició
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

//Màxims i percentatges
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

//Mostra quan es connecta/desconnecta per Serial
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


//FUNCIONS DE PROCESSAMENT DE SENYAL
//Actualització de màxims i càlculs de percentatges
void actualitzarMaxValues(float emg1,float emg2, float emg3){
  emg1_value = fabs(emg1_value);
  emg2_value = fabs(emg2_value);
  emg3_value = fabs(emg3_value);
 //Actualitzar els maxims
 if (emg1_value > max1) max1 = emg1_value;
 if (emg2_value > max2) max2 = emg2_value;
 if (emg3_value > max3) max3= emg3_value;
 float maxTotal = max1 + max2 + max3;
 perc1 = (max1/maxTotal)*100;
 perc2= (max2/maxTotal)*100;
 perc3 = (max3/maxTotal)*100;
}


void readADS1294Data() {
  unsigned long t = millis();

  float noise1 = ((float)random(-100, 100)) / 1000.0;  // soroll petit
  float noise2 = ((float)random(-100, 100)) / 1000.0;
  float noise3 = ((float)random(-100, 100)) / 1000.0;

  // Simulació d’un EMG: sinusoide + soroll + activació temporal (com un burst)
  float emg1 = 0.1 * sin(2 * PI * 50 * t / 1000.0) + noise1; //ALERTA
  float emg2 = 0.7 * sin(2 * PI * 60 * t / 1000.0) + noise2;
  float emg3 = 1.0 * sin(2 * PI * 40 * t / 1000.0) + noise3;

  // Activació intermitent per simular contraccions
  if ((t / 1000) % 4 < 2) emg1 *= 1.5;
  if ((t / 1000) % 5 < 3) emg2 *= 1.5;
  if ((t / 1000) % 6 < 2) emg3 *= 1.5;

  actualitzarMaxValues( emg1, emg2, emg3);

  if (index_buffer < BUFFER) {
    emg1_buffer[index_buffer] = emg1;
    emg2_buffer[index_buffer] = emg2;
    emg3_buffer[index_buffer] = emg3;
    index_buffer++;
  }
}

//FUNCIONS DE COMUNICACIÓ BLE
//Notifica dades per BLE
void enviarDades(){
  String emg1_str="", emg2_str="", emg3_str="";
  float sum1=0, sum2=0, sum3=0;
  unsigned long t = millis();
  float seconds = t/1000.0;

  for (int i =0; i<BUFFER; i++){
    emg1_str += String(emg1_buffer[i],2);
    emg2_str += String(emg2_buffer[i],2);
    emg3_str += String(emg3_buffer[i],2);
    sum1+=emg1_buffer[i];
    sum2+=emg2_buffer[i];
    sum3+=emg3_buffer[i];

    if(i<BUFFER-1){
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

//CONTROL D'ESTAT
void stopPorgram(){
  isRunning=false;
}

//SETUP() I LOOP()

void setup() {

    Serial.begin(115200);

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

  if (now - ultimalectura >= lecturadades) {
    ultimalectura = now;
    readADS1294Data();
  }

  if ((deviceConnected) && now - ultimenvio >= enviodades && index_buffer >= BUFFER) {
    ultimenvio = now;
    enviarDades();
  }

}
 
