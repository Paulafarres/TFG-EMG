# TFG-EMG: Sistema d’adquisició i visualització d’electromiografia de superfície

Aquest projecte forma part del Treball de Final de Grau en Enginyeria Biomèdica. L’objectiu és desenvolupar un sistema portàtil capaç de captar, processar i visualitzar senyals EMG durant exercicis de força, com la sentadilla, en pacients amb pròtesi total de genoll.

## Contingut del repositori

- `firmware/`: Codi C++ per l’ESP32 amb adquisició de 3 canals EMG via ADS1294 i transmissió per BLE.
- `webapp/`: Interfície HTML/JavaScript per connectar-se via BLE, visualitzar gràfics en temps real i exportar dades.
- `data/`: Fitxers de prova o resultats en format CSV.
- `README.md`: Aquest document.
- `docs/`: Manual d’usuari i instal·lació.

## Requisits

### Hardware
- ESP32 DevKitC-32D
- ADS1294 (xip d’adquisició EMG)
- 7 elèctrodes de superfície
- PCB pròpia (veure annex D del TFG)
- Bateria o USB per alimentació

### Programari
- PlatformIO (VS Code) per al codi de l’ESP32
- Navegador web compatible amb BLE (ex. Chrome) per visualització i comparació de dades


## Instal·lació

1. **ESP32:**
   - Obre el directori `firmware/` amb PlatformIO.
   - Carrega el codi a l’ESP32 via USB.
   - Connecta el sistema EMG i encén-lo.

2. **Interfície web:**
   - Obre `webapp/index.html` amb Chrome o Edge.
   - Prem *Connectar* i selecciona el dispositiu BLE `TFG EMG`.
   - Comença a rebre i visualitzar dades.

## Manual d’usuari

1. Escriu el nom
2. Selecciona la cama
2. Durant l’exercici, observa el gràfic EMG i percentatge d’activació.
3. Prem *STOP* per aturar la sessió.
4. El sistema exportarà automàticament un fitxer `.csv` amb dades reals i % d’activitat.
5. A la pestanya de comparació, pots carregar dos fitxers per analitzar diferències entre cama operada i sana.

## Dades

Els fitxers de dades s’exporten automàticament des de la interfície web. També es poden afegir manualment a la carpeta `data/`.

## Llicència

Aquest projecte s’allibera sota llicència MIT. Pots reutilitzar-lo amb finalitats docents o de recerca, citant l’autora.

---


