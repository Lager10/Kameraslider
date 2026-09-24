# Kameraslider

CAD-Daten, Stückliste und Arduino-Firmware für einen motorisierten Kameraslider
mit schwenkbarer Kameraachse.

## Struktur

```text
.
├── firmware/Kameraslider/   Arduino-Sketch
├── cad/                     komprimierter STEP-Export
├── docs/                    Stückliste als Markdown und Word-Datei
└── README.md                Projektübersicht
```

## Elektronik und Firmware

Der Sketch ist für einen Arduino Mega mit RAMPS 1.6 ausgelegt. Er steuert zwei
Schrittmotoren über TMC2208-Treiber und verwendet ein I²C-OLED sowie mechanische
Endschalter. Parameter werden im EEPROM gespeichert.

Benötigte Bibliotheken:

- Adafruit SSD1306
- AccelStepper
- die mit Arduino bereitgestellten Bibliotheken EEPROM und Wire

Der Sketch liegt unter
[`firmware/Kameraslider/Kameraslider.ino`](firmware/Kameraslider/Kameraslider.ino)
und besitzt damit wieder die vom Arduino-IDE-Projekt erwartete Dateiendung und
Ordnerstruktur.

> [!IMPORTANT]
> Die vorhandenen Unterlagen sind an zwei Stellen nicht deckungsgleich: Der
> Sketch ist für TMC2208-Treiber im Standalone-Betrieb und ein SSD1306-OLED
> geschrieben, während die ursprüngliche Stückliste TMC2130-Treiber und ein
> SH1106-OLED nennt. Vor dem Teilekauf beziehungsweise Aufbau muss entschieden
> werden, welche Hardware tatsächlich verwendet wird; bei TMC2130 oder SH1106
> ist die Firmware entsprechend anzupassen.

Vor dem ersten Motorbetrieb müssen Fahrtrichtung, Endschalterlogik,
Verfahrweg und Stromgrenzen der Treiber ohne montierte Kamera geprüft werden.

## CAD und Stückliste

[`cad/Kameraslider.step.zip`](cad/Kameraslider.step.zip) enthält den STEP-Export
`Kameraslider.step`. Die übersichtlich lesbare Stückliste befindet sich unter
[`docs/Stueckliste.md`](docs/Stueckliste.md). Die ursprüngliche Word-Fassung
liegt zusätzlich unter `docs/Stueckliste.docx`; ihre Autorenmetadaten wurden auf
den öffentlichen Projektnamen `Lager10` reduziert.
