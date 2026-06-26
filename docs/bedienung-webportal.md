# AutoSat – Bedienung & Kalibrierung über das Web-Portal

## Verbindung herstellen

1. ESP32-C3 einschalten (USB oder Bordstrom)
2. Mit dem WLAN-Netz **AutoSat** verbinden (kein Passwort)
3. Browser öffnen → `192.168.4.1`

---

## Tabs im Überblick

| Tab | Funktion |
|---|---|
| **Status** | Tracking starten/stoppen, Süd-Referenz setzen |
| **Elevation** | Servo steuern, Elevation kalibrieren, Korrekturachse wählen |
| **GPS/Sat** | GPS-Koordinaten eingeben, Satelliten-Längengrad festlegen |
| **Motor** | Manuelle Azimut-Schritte, Step-Limits setzen |
| **WiFi** | Optionaler WLAN-Client für MQTT/Internet |
| **Debug** | Live-Rohdaten: IMU, Heading, Fehlerwinkel, Loop-Hz |

---

## Erstinbetriebnahme (einmalig)

### Schritt 1 – GPS-Koordinaten eingeben

Tab **GPS/Sat** → eigene Breite/Länge eintragen → **Speichern**.  
Der Satellit (Standard: Astra 19.2°E) wird automatisch berechnet.

### Schritt 2 – Azimut-Südkalibrierung

1. Plattform physisch nach **Süden** drehen (Kompass-App oder Markierung am Gerät)
2. Tab **Status** → **„Süd-Referenz setzen"**
3. Das System setzt `currentSteps = 0`, berechnet den Montageoffset aus dem aktuellen Kompasswert und speichert alles dauerhaft im NVM

> Diese Kalibrierung gilt für alle Neustarts. Nur wiederholen, wenn die Plattform physisch umgebaut wurde.

### Schritt 3 – Elevation-Servo kalibrieren

1. Tab **Elevation** → **„Servo aktivieren"**  
   Der Servo fährt auf den zuletzt gespeicherten Winkel.
2. Mit **+1° / −1°** (Feinabstimmung) oder **+10° / −10°** (grob) auf Satellitensignal ausrichten  
   → Empfang prüfen am TV oder Receiver (maximale Signalstärke)
3. **„Kalibrieren & speichern"** – dieser Servo-Winkel ist jetzt die Referenz für den Tracking-Betrieb

### Schritt 4 – Korrekturachse wählen

1. Tab **Status** → **„Tracking starten"**
2. Boot seitlich kippen – die Schüssel muss aktiv gegensteuern
3. Falls sie in die falsche Richtung korrigiert:  
   Tab **Elevation** → anderen Achsen-Button wählen: **+Pitch / −Pitch / +Roll / −Roll**
4. Solange wechseln, bis die Korrektur stimmt

---

## Normaler Betrieb (nach jedem Neustart)

1. Mit WLAN **AutoSat** verbinden, Portal öffnen
2. Tab **Elevation** → **„Servo aktivieren"**  
   (Servo ist nach Neustart deaktiviert – Sicherheitsmaßnahme)
3. Tab **Status** → **„Tracking starten"**  
   Das System kalibriert den Gyro-Offset für ca. 2,5 Sekunden, dann läuft das Tracking automatisch

> Der Azimut-Motor startet sofort an der gespeicherten Step-Position aus dem letzten Betrieb.

---

## Tracking manuell stoppen

Tab **Status** → **„Tracking stoppen"**  
Servo und Motor bleiben in ihrer aktuellen Position, reagieren aber nicht mehr auf Bewegungen.

---

## Motor manuell bewegen (Tab: Motor)

- **Schritte +/−**: Azimut manuell in Schritten verfahren (zum Testen oder Ausrichten)
- **Step-Limits setzen**: Soft-Limits für den Azimut-Bereich definieren (verhindert Kabelwicklung)
- **Motor-Test**: Kurzer Testlauf, um Motorstrom und Verbindung zu prüfen

---

## Debug-Tab – was bedeutet was?

| Wert | Bedeutung |
|---|---|
| **Heading** | Berechnetes Kompass-Heading (tilt-kompensiert), 0°=Nord |
| **Pitch / Roll** | Neigung des Boards in Grad |
| **dishAz** | Aktuelle berechnete Schüsselrichtung (absolut) |
| **Fehlerwinkel** | Differenz Ziel-Azimut − dishAz (was der Motor korrigieren muss) |
| **Loop-Hz** | Verarbeitungsgeschwindigkeit der Firmware-Hauptschleife |

---

## Häufige Probleme

**Tracking dreht in die falsche Richtung (Azimut)**  
→ Süd-Kalibrierung wiederholen. Plattform muss beim Setzen der Referenz exakt nach Süden zeigen.

**Elevation korrigiert falsch bei Schaukeln**  
→ Korrekturachse wechseln (Tab Elevation, +Pitch/−Pitch/+Roll/−Roll).

**Servo reagiert nicht**  
→ Servo nach jedem Neustart erst über „Servo aktivieren" freigeben.

**Kompass dreht beim Betrieb der Motoren**  
→ Magnetometer (GY-271) so weit wie möglich von Schrittmotor und Stromkabeln entfernt montieren.
