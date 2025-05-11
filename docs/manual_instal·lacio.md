# Manual d’instal·lació

Aquest document explica com instal·lar i configurar el sistema EMG al teu ordinador per programar l’ESP32 i visualitzar les dades.

## 1. Requisits de maquinari

- ESP32 DevKitC-32D
- Xip ADS1294
- PCB dissenyada pel projecte (Annex D)
- 7 elèctrodes de superfície
- Cables i bateria o alimentació USB

## 2. Instal·lació del firmware (ESP32)

1. Instal·la **Visual Studio Code** i el plugin **PlatformIO**.
2. Obre el directori `firmware/` des de PlatformIO.
3. Connecta l’ESP32 per USB.
4. Carrega el codi (`main.cpp`) a l’ESP32 fent clic a la fletxa de "Upload".

## 3. Executar la interfície web

1. Obre el fitxer `webapp/index.html` amb **Google Chrome** o **Microsoft Edge**.
2. Prem **“START”** i selecciona el dispositiu `TFG EMG`.

## 4. Compatibilitat

- La connexió BLE només funciona en navegadors compatibles com **Chrome** (Windows, macOS, Android) i **Edge**.
- No és compatible amb Firefox o Safari.
