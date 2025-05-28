# TFG-EMG: Sistema d’adquisició i visualització d’electromiografia de superfície

Aquest projecte forma part del Treball de Final de Grau en Enginyeria Biomèdica. L’objectiu és desenvolupar un sistema portàtil capaç de captar, processar i visualitzar senyals EMG durant exercicis de força, com l'extensió de genoll en pacients amb pròtesi total de genoll. El sistema permet comparar l'activació muscular entre la cama sana i la operada.

## Contingut del repositori

- `firmware/`: Codi C++ per l’ESP32. Llegeix senyals EMG de 3 canals mitjançant l’ADS1294, aplica filtratge digital (notch, offset, mitjana mòbil), calcula la RMS i transmet els valors per Bluetooth Low Energy (BLE).
- `webapp/`: Interfície HTML/JavaScript per connectar-se via BLE, visualitzar gràfics en temps real i exportar dades.
- `data/`: Fitxers de prova o resultats en format CSV.
- `docs/`: Manual d’usuari i instal·lació.
- `README.md`: Aquest document.

## Requisits

### Hardware
- ESP32 DevKitC-32D
- ADS1294 (xip d’adquisició EMG)
- 7 elèctrodes de superfície
- PCB pròpia (veure annex D del TFG)
- Bateria o USB per alimentació

### Programari
- PlatformIO (VS Code) per al codi de l’ESP32
- Navegador compatible amb Web Bluetooth API (Chrome o Edge en Windows/Linux/Android)


## Instal·lació

1. **ESP32:**
   - Obre el directori `firmware/` amb PlatformIO.
   - Carrega el codi a l’ESP32 via USB.
   - Connecta la PCB i encén el sistema.


2. **Interfície web:**
   - Obre `webapp/index.html` amb un navegador compatible (ex:Chrome o Edge).
   - Prem *Connectar* i selecciona el dispositiu BLE `TFG EMG`.
   - Visualitza en temps real les dades EMG i percentatges d’activació.
   - Prem *STOP* per exportar les dades a .csv.
     
## Manual d’usuari

1. Introdueix el nom del pacient i selecciona si es tracta de la cama sana o operada.
2. Durant l’exercici, observa el gràfic EMG i percentatge d’activació.
3. Prem *STOP* per aturar la sessió.
4. El sistema exportarà automàticament un fitxer `.csv` amb dades RMS.
5. A la pestanya de *comparació*, pots carregar dos fitxers .csv per analitzar diferències entre cama operada i sana.

## Dades

Els fitxers de dades s’exporten automàticament des de la interfície web. També es poden afegir manualment a la carpeta `data/`.

## Autoria

Paula Farrés Sorroche

Grau en Enginyeria Biomèdica

Universitat de Girona

Curs 2024–2025

Tutor: Carles Pous Sabadí

## Llicència

Aquest projecte s’allibera sota llicència MIT. Pots reutilitzar-lo amb finalitats docents o de recerca, citant l’autora.

---


