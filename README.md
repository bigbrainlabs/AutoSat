# AutoSat

Automatische Satelliten-Ausrichtung mit Echtzeit-Tracking für kleine Flachantennen/Schüsseln (35–40 cm) – hält den Satelliten auch während der Fahrt im Fokus. Für Boote, Wohnmobile, Caravans.

**Ziel:** DIY statt teure kommerzielle Systeme. Selbstgemacht ist wissen was drin ist und bezahlbar.

> Status: 🔧 Prototyp in Betrieb – Feinabstimmung läuft

---

## Konzept

Astra 19.2°E ist geostationär – steht fest am Himmel. Bewegt sich das Boot (dreht, schaukelt), muss die Schüssel nachgeführt werden – aktiv, auch während der Fahrt.

Die Plattform dreht sich auf einer Drehachse (Azimut), ein Servo kippt die Schüssel (Elevation). Ein IMU mit Kompass und Gyroskop misst die aktuelle Ausrichtung; die Firmware berechnet daraus den Fehler und korrigiert ihn.

**Tracking-Ansatz:** absolutes Azimut-Tracking über Kompassheading. Die Plattform wird einmalig nach Süden ausgerichtet (Kalibrierung), danach kennt das System jederzeit die absolute Schüsselrichtung:

```
Schüsselrichtung = Kompass-Heading + Motorposition + Montageoffset
Fehler = Ziel-Azimut − Schüsselrichtung
```

Elevation wird einmalig kalibriert (Servo-Winkel bei Satellitensignal gespeichert) und im Tracking per IMU-Achse für Boots-Neigung korrigiert.

---

## Hardware (aktueller Aufbau)

| # | Komponente | Funktion | Link |
|---|---|---|---|
| 1 | ESP32-C3 SuperMini | Steuerung, WiFi Access Point | [Amazon](https://amzn.to/4w9G6VN) |
| 2 | GY-91 (MPU-6500 + BMP280) | Gyroskop + Beschleunigung (Pitch/Roll/Yaw) | [Amazon](https://amzn.to/4wfitv6) |
| 3 | GY-271 (HMC5883L) | Magnetometer / Kompass | [Amazon](https://amzn.to/3QKusSe) |
| 4 | TMC2209 Schrittmotortreiber | Azimut-Motor, UART-Ansteuerung, StealthChop | [Amazon](https://amzn.to/3SqNw8E) |
| 5 | NEMA17 Schrittmotor | Azimut-Antrieb, Zahnradübersetzung 2:1 | [Amazon](https://amzn.to/4g7nUHM) |
| 6 | MG996R Servo | Elevation-Antrieb | [Amazon](https://amzn.to/4w8z7MJ) |
| 7 | GT2-Riemen + Umlenkrolle | Kraftübertragung Azimut-Antrieb | [Amazon](https://amzn.to/4oPDdqM) |
| 9 | Drehlager für Plattform | Zentrale Drehachse | [Amazon](https://amzn.to/4oMyjL0) |

> Spannungsversorgung und Kleinmaterial (Kabel, Stecker, Dupont) nach eigenem Bedarf.

### Pin-Belegung

| Pin | Funktion |
|---|---|
| GPIO2 | STEP (TMC2209) |
| GPIO4 | DIR (hardwärts verkabelt, ungenutzt – Richtung via UART) |
| GPIO5 | Servo (MG996R, PWM) |
| GPIO6 | I2C SDA (MPU-6500, HMC5883L) |
| GPIO7 | I2C SCL |
| GPIO8 | TMC2209 UART TX (= LED-Pin, flackert bei Richtungswechsel) |
| GND | TMC2209 ENABLE (fest LOW = immer aktiv) |

### Wichtige Hardware-Eigenheiten

- **ESP32-C3 ist Single-Core (RISC-V):** `noInterrupts()` blockiert auch den WiFi-Stack. Gelöst durch minimale Interrupt-Blackouts (nur Adresse 3 ansprechen, ~8 ms) + `yield()` danach.
- **TMC2209 UART-Adresse = 3:** Bei 1/16-Mikroschritt sind MS1=HIGH, MS2=HIGH → UART-Adresse = 2·MS2+MS1 = 3. Adresse 0 wäre falsch.
- **NEMA17 Resonanzzone:** 80–300 Vollschritte/s → bei 16× Mikroschritt Mindest-Delay 800 µs/Schritt (= 39 Vollschritte/s).
- **Richtungssteuerung per UART:** DIR-Pin wird vom IC ignoriert; Richtung läuft über GCONF.shaft-Bit (Register 0x00).

---

## Software-Architektur

### Firmware (`firmware/firmware.ino` + `firmware/webportal.h`)

**IMU-Filter:**
- Pitch/Roll: Komplementärfilter, α=0.98 (Gyro + Beschleunigungssensor)
- Yaw/Heading: dynamisches α – bei Drehung 0.97 (Gyro dominiert), bei Stillstand 0.85 (Kompass zieht Drift heraus)
- Tilt-kompensiertes Magnetometer-Heading (Pitch/Roll-Korrektur)

**Azimut-Tracking:**
- Absolut: `dishAz = heading_yaw + currentSteps/STEPS_PER_DEGREE + azMountOffset`
- `azMountOffset` wird einmalig bei Süd-Kalibrierung gesetzt und in NVM gespeichert
- Soft-Limits: ±180° × STEPS_PER_DEGREE (nach Kalibrierung dynamisch gesetzt)

**Elevation-Tracking:**
- Referenz-Servo-Winkel (`servoCalAngle`) einmalig kalibriert und in NVM gespeichert
- Korrektur per wählbarer IMU-Achse: +Pitch / +Roll / −Pitch / −Roll (über Portal einstellbar)
- Rampe: manuell 60 ms/° (~16°/s), Live-Tracking 12 ms/° (~83°/s)
- Servo nach Neustart deaktiviert – muss im Portal explizit freigegeben werden

**Wichtige Parameter:**

| Parameter | Wert | Bedeutung |
|---|---|---|
| STEPS_PER_DEGREE | 17,78 | 200 Schritte/U × 16× Mikro × 2:1 Übersetzung |
| STEP_DELAY_TRACK | 300 µs | Tracking-Schrittgeschwindigkeit |
| STEP_MAN_MIN | 800 µs | Minimaler manueller Delay (Resonanz vermeiden) |
| TMC IRUN | 25 % (IRUN=8) | Motorstrom Fahren |
| TMC IHOLD | 6 % (IHOLD=2) | Motorstrom Halten |

---

## Web-Portal

Der ESP32-C3 öffnet einen WiFi Access Point (`AutoSat`). Portal unter `192.168.4.1`.

**Tabs:**

| Tab | Inhalt |
|---|---|
| Status | Tracking starten/stoppen, Süd-Referenz setzen, Azimut-Kalibrierung |
| Elevation | Servo freigeben, manuell positionieren, MIN/MAX setzen, kalibrieren, Achse wählen |
| GPS/Sat | Koordinaten, Satelliten-Längengrad, Azimut/Elevation-Anzeige |
| Motor | Manuelle Azimut-Schritte, Motor-Test, Step-Limits setzen |
| WiFi | WLAN-Client konfigurieren (optional, für MQTT/Internet) |
| Debug | Live-Rohdaten IMU, Heading, Fehlerwinkel, Loop-Hz |

---

## Kalibrierung (Erstinbetriebnahme)

### 1. Azimut – Süd-Referenz setzen

1. Plattform physisch nach Süden drehen (Kompass-App oder Markierung)
2. Portal → Status → **„Süd-Referenz setzen"**
3. Firmware setzt: `currentSteps = 0`, Soft-Limits ±180°, `azMountOffset` aus aktuellem Kompasswert
4. Einmalig – wird in NVM gespeichert, gilt für alle Neustarts

### 2. Elevation – Servo kalibrieren

1. Portal → Elevation → **„Servo aktivieren"** (Servo fährt auf gespeicherte Position)
2. Servo manuell mit ±1°/±10°-Buttons auf Satellitensignal ausrichten
3. Portal → Elevation → **„Kalibrieren & speichern"**
4. Dieser Servo-Winkel wird als Referenz gespeichert

### 3. Elevation-Korrekturachse wählen

1. Tracking starten
2. Boot seitlich kippen → Schüssel muss in die richtige Richtung nachkorrigieren
3. Falls falsch: Portal → Elevation → anderen Achsen-Button (+Pitch / +Roll / −Pitch / −Roll) probieren

### 4. Tracking starten

1. Portal → Status → **„Tracking starten"** (kalibriert Gyro-Offset für ~2,5 s)
2. System hält Astra 19.2°E automatisch im Fokus

---

## Ablauf nach jedem Neustart

1. ESP32 startet, WiFi-AP `AutoSat` verfügbar
2. Servo ist **deaktiviert** – keine automatische Bewegung
3. Portal öffnen → Elevation → **„Servo aktivieren"** → Servo fährt auf kalibrierten Winkel
4. Portal → Status → **„Tracking starten"**

Azimut-Motor startet sofort an der gespeicherten Step-Position aus dem letzten Betrieb (NVM).

---

## Repo-Struktur

```
AutoSat/
├── README.md
├── firmware/
│   ├── firmware.ino               ← Haupt-Firmware (IMU, Motor, Tracking, NVM)
│   └── webportal.h                ← Web-Portal (HTML/CSS/JS + HTTP-Handler)
├── docs/
│   ├── bedienung-webportal.md     ← Bedienung & Kalibrierung über das Web-Portal
│   └── schaltplan.md              ← Schaltplan & Verdrahtung aller Komponenten
└── 3D-Print-Parts/
    ├── Autosat-Elevation_Mount.stl  ← Halterung Elevation-Servo
    └── Nema17-Ground_Mount.stl      ← Grundhalterung NEMA17 Azimut-Motor
```

**Dokumentation:**
- [Bedienung & Kalibrierung über das Web-Portal](docs/bedienung-webportal.md)
- [Schaltplan & Verdrahtung](docs/schaltplan.md)

---

## NVM-Schlüssel (Preferences, Namespace "autosat")

| Schlüssel | Typ | Bedeutung |
|---|---|---|
| `steps` | Long | Letzte Motor-Position |
| `azMntOff` | Float | Azimut-Montageoffset (Süd-Kalibrierung) |
| `azMin` / `azMax` | Long | Azimut Soft-Limits in Steps |
| `srvCalAng` | Int | Kalibrierter Servo-Winkel (Elevation) |
| `servoMin` / `servoMax` | Int | Servo-Endpunkte |
| `elvAxis` | Int | Korrektur-Achse (0=+Pitch, 1=+Roll, 2=−Pitch, 3=−Roll) |
| `gpsLat` / `gpsLon` | Double | GPS-Koordinaten |
| `satLon` | Float | Satelliten-Längengrad (Default: 19.2) |
| `azOffset` | Float | Azimut-Offset für Satellitenberechnung |
| `azMountOffset` | Float | (identisch mit azMntOff, älterer Schlüssel) |

---

## Offene Punkte

- [ ] Genauigkeit bei längerem Betrieb auf dem Wasser testen
- [ ] Kompass-Kalibrierung (Hard/Soft-Iron) implementieren
- [ ] Achsen-Transformation (cos/sin) bei bekannter Board-Montage aktivieren
- [ ] MQTT-Integration (optional)
- [ ] Gehäuse / wetterfeste Montage

---

---

## Projekt unterstützen

Wer das Projekt gut findet – ich schreibe die Buch-Serie **„Logbuch ohne Pose"** über das Leben auf dem Boot, ehrlich und ungeschönt. In **Band 7** wird AutoSat eine Rolle spielen.

👉 [Logbuch ohne Pose – bei Amazon](https://amzn.to/4w6gBo9)

Mit dem Kauf eines Bandes oder der ganzen Serie unterstützt du die Weiterentwicklung direkt. Danke! ⚓

---

## Lizenz

Dieses Projekt steht unter der **GNU General Public License v3.0 (GPL-3.0)**.

Das bedeutet: Du kannst den Code frei verwenden, verändern und weitergeben – solange Ableitungen ebenfalls unter der GPL v3 veröffentlicht werden und die Quelle angegeben wird.

Weitere Details: [https://www.gnu.org/licenses/gpl-3.0.html](https://www.gnu.org/licenses/gpl-3.0.html)

---

*"~50 € DIY. ~3.000 € kommerziell. Der Rest ist Physik."* ⚓

🐾 Buster ist nach wie vor skeptisch.
