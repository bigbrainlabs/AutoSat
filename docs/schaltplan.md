# AutoSat – Schaltplan & Verdrahtung

> **Hinweis:** Ein grafischer Schaltplan (z.B. als Fritzing- oder KiCad-Datei) wird demnächst ergänzt, um die Verdrahtung noch übersichtlicher darzustellen.

## Systemübersicht

```
                        ┌──────────────────────────┐
                        │     ESP32-C3 SuperMini    │
                        │                          │
          3,3V ────────►│ 3V3              GPIO2   ├──── STEP ────────────► TMC2209
           GND ────────►│ GND              GPIO4   ├──── DIR  ────────────► TMC2209 (per UART übersteuert)
                        │                  GPIO5   ├──── PWM  ────────────► MG996R Servo (Signal)
                        │                  GPIO6   ├──── SDA  ─────┬──────► GY-91 (MPU-6500)
                        │                  GPIO7   ├──── SCL  ─────┴──────► GY-271 (HMC5883L)
                        │                  GPIO8   ├──── UART TX ─────────► TMC2209 PDN_UART
                        └──────────────────────────┘
```

---

## Stromversorgung

```
Bordnetz 12 V
    │
    ├──► TMC2209  VM  (12 V Motorspannung)
    │
    └──► DC-DC Konverter 12 V → 5 V
              │
              ├──► MG996R Servo  VCC (5 V)
              │
              └──► ESP32-C3  5V-Pin (nicht USB – direkt auf 5V-Rail)
                        │
                        └── 3,3 V intern (für GY-91, GY-271, TMC2209 VIO)
```

> Den ESP32-C3 am 5V-Pin (nicht USB-C) anschließen, wenn der DC-DC Konverter die Versorgung übernimmt. Nie gleichzeitig USB und externen 5V anlegen.

---

## ESP32-C3 SuperMini – Pin-Belegung

| GPIO | Funktion | Ziel |
|------|----------|------|
| GPIO2 | STEP (Digital Out) | TMC2209 STEP |
| GPIO4 | DIR (Digital Out, ungenutzt) | TMC2209 DIR |
| GPIO5 | PWM (Servo) | MG996R Signal-Kabel |
| GPIO6 | I2C SDA | GY-91 SDA + GY-271 SDA |
| GPIO7 | I2C SCL | GY-91 SCL + GY-271 SCL |
| GPIO8 | UART TX (Software Serial) | TMC2209 PDN_UART |
| GND | Masse | Alle Komponenten |
| 3V3 | 3,3-V-Ausgang | GY-91 VCC, GY-271 VCC, TMC2209 VIO |

---

## TMC2209 Schrittmotortreiber

```
ESP32-C3 GPIO2  ──────────────────────► STEP
ESP32-C3 GPIO4  ──────────────────────► DIR
ESP32-C3 GPIO8  ──── 1 kΩ ────────────► PDN_UART  ◄──── 100 kΩ Pull-up ── 3,3 V
ESP32-C3 GND    ──────────────────────► EN   (fest LOW = immer aktiv)
ESP32-C3 3V3    ──────────────────────► VIO
Bordnetz 12 V   ──────────────────────► VM
GND             ──────────────────────► GND

MS1             ──────────────────────► 3,3 V   (HIGH)
MS2             ──────────────────────► 3,3 V   (HIGH)
                                       → UART-Adresse 3 (2×MS2 + MS1 = 2×1 + 1 = 3)

NEMA17 Spule A  ──────────────────────► 1A / 1B
NEMA17 Spule B  ──────────────────────► 2A / 2B
```

**PDN_UART-Beschaltung (Single-Wire UART):**

```
ESP32 GPIO8 ──┬── 1 kΩ ──► PDN_UART
              │
            (optional: 100 kΩ Pull-up nach 3,3 V direkt am PDN_UART-Pin)
```

> Richtungssteuerung läuft ausschließlich per UART (GCONF.shaft-Bit, Register 0x00). Der DIR-Pin wird vom IC ignoriert.

---

## GY-91 (MPU-6500 + BMP280) – Gyroskop & Beschleunigung

| GY-91 Pin | ESP32-C3 Pin |
|-----------|-------------|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO6 |
| SCL | GPIO7 |

> I2C-Adresse MPU-6500: **0x68** (AD0 auf GND)

---

## GY-271 (HMC5883L) – Magnetometer / Kompass

| GY-271 Pin | ESP32-C3 Pin |
|------------|-------------|
| VCC | 3V3 |
| GND | GND |
| SDA | GPIO6 (gemeinsamer I2C-Bus mit GY-91) |
| SCL | GPIO7 (gemeinsamer I2C-Bus mit GY-91) |

> I2C-Adresse HMC5883L: **0x1E** (fest)

### I2C-Bus Pull-ups

Beide Sensoren hängen am selben I2C-Bus. Falls keine Pull-ups auf den Modulen vorhanden sind (GY-91 und GY-271 haben sie meist onboard):

```
SDA ──── 4,7 kΩ ──── 3,3 V
SCL ──── 4,7 kΩ ──── 3,3 V
```

---

## MG996R Servo – Elevation-Antrieb

| Servo-Kabel | Anschluss |
|-------------|-----------|
| Signal (Orange/Gelb) | ESP32-C3 GPIO5 |
| VCC (Rot) | DC-DC Konverter 5 V |
| GND (Braun/Schwarz) | GND (gemeinsam mit ESP32) |

> Der Servo sitzt auf der rotierenden Plattform – Signal, VCC und GND werden über den 6-poligen Schleifring in der Drehachse geführt.

---

## NEMA17 Schrittmotor – Azimut-Antrieb

Die vier Motorleitungen werden an den TMC2209 angeschlossen. Typische Farbbelegung (je nach Hersteller prüfen!):

| Motorleitung | TMC2209 Pin | Typische Farbe |
|---|---|---|
| Spule A+ | 1A | Schwarz |
| Spule A− | 1B | Grün |
| Spule B+ | 2A | Rot |
| Spule B− | 2B | Blau |

> Spulenpaare mit Multimeter messen: beide Enden einer Spule zeigen Widerstand (~1–5 Ω). Kreuzpaarung dreht den Motor rückwärts – einfach ein Spulenpaar umpolen.

---

## Häufige Fehler

- **GND nicht gemeinsam:** Alle Komponenten müssen auf dem selben GND-Potential liegen. Servo-GND unbedingt mit ESP32-GND verbinden, sonst fehlerhaftes PWM-Signal.
- **TMC2209 UART-Adresse falsch:** MS1 und MS2 müssen beide HIGH (3,3 V) sein → Adresse 3. Adresse 0 (beide LOW) führt zu keiner Kommunikation.
- **Magnetometer zu nah am Motor:** HMC5883L muss möglichst weit weg von NEMA17 und den Motorstromkabeln montiert sein – starke Störfelder verfälschen den Kompasswert.
- **Servo direkt an 3,3 V:** Führt zu Brownouts des ESP32. Immer externe 5-V-Versorgung (DC-DC Konverter) verwenden.
