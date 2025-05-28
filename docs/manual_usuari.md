# Manual d’usuari

Aquest manual explica com utilitzar el sistema d’electromiografia (EMG) desenvolupat per aquest projecte, destinat a analitzar l’activitat muscular durant exercicis com la sentadilla.

## 1. Connexió i inici

1. Assegura’t que el sistema EMG està encès i l’ESP32 alimentat per USB o bateria.
2. Obre el fitxer `index.html` situat dins la carpeta `webapp/` amb el navegador **Google Chrome** o **Microsoft Edge**.
3. Introducció de dades: escriu el nom del pacient a l'inici de la pàgina.
4. Fes clic a **“START”** i selecciona el dispositiu BLE anomenat `TFG EMG`.
5. Un cop connectat, començaràs a veure les gràfiques EMG en temps real.

## 2. Monitorització d’un exercici

1. Selecciona la cama : **sana** o **operada**.
2. Executa l’exercici mentre observes l’activitat muscular en temps real i el percentatge d’activació per cada múscul.
Durant la sessió es mostrarà:
  - El senyal EMG de 3 músculs en forma de gràfic de línies.
  - El percentatge d'activació de cada múscul en un gràfic de barres.
    
## 3. Finalització i exportació

1. Prem **STOP** un cop finalitzada la sessió.
2. El sistema guardarà automàticament un fitxer `.csv` amb:
   - Totes les mostres EMG.
   - Els valors RMS finals de cada múscul.
   - El valor RMS total de la sessió.

## 4. Comparació de sessions

1. A la secció de comparació, selecciona dos fitxers `.csv`.:
   - Un corresponent a la cama operada.
   - Un a la cama sana.
2. Prem **Comparar RMS** per obtenir:
  - Una taula amb RMS de cada múscul per cama.
  - Diferències percentuals.
  - Detecció automàtoca de possibles dèficits si la diferència és significativa.

---

Per qualsevol dubte o error en la connexió BLE, assegura’t que el dispositiu està encès i que s’anomena `TFG EMG`.
