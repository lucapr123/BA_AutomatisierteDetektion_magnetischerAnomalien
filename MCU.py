#include <Arduino.h>
#include <Wire.h>

// Definiere analoge Pins für das Arduino Mega 2560 Board
#define A0 54
#define A1 55
#define A2 56
#define A3 57
#define A4 58
#define A5 59

// Arduino MEGA R3
#define VREF 5.0  // Referenzspannung von 5V
#define ADC_RESOLUTION 1024.0  // 10-Bit Auflösung des ADC

// Sensor 1
const int analogPin11 = A0; // X-Achse von Sensor 1, verbunden mit Analog-Pin A0
const int analogPin21 = A1; // Y-Achse von Sensor 1, verbunden mit Analog-Pin A1
const int analogPin31 = A2; // Z-Achse von Sensor 1, verbunden mit Analog-Pin A2

// Sensor 2
const int analogPin12 = A3; // X-Achse von Sensor 2, verbunden mit Analog-Pin A3
const int analogPin22 = A4; // Y-Achse von Sensor 2, verbunden mit Analog-Pin A4
const int analogPin32 = A5; // Z-Achse von Sensor 2, verbunden mit Analog-Pin A5

// Arrays zur Speicherung der Spannungswerte für Sensor 1 und 2
float VoltagesSensor1[3];
float VoltagesSensor2[3];

// Gefilterte Spannungen der Sensoren
float filteredVoltagesSensor1[3] = {0, 0, 0};
float filteredVoltagesSensor2[3] = {0, 0, 0};

// Zeitvariablen für die Messintervalle
unsigned long lastMesszeit;
const unsigned long interval = 0.1; // Intervall zur Ausführung der printVoltages Funktion (10 ms)

// Konstante für den Tiefpassfilter
const float alpha = 0.4;  // Smoothing-Faktor für den Filter

// Tiefpassfilter-Funktion
float lowPassFilter(float currentValue, float previousFilteredValue) {
  return alpha * currentValue + (1.0 - alpha) * previousFilteredValue;  // Wendet den Tiefpassfilter an
}

// Funktion zum Lesen der Spannungen von Sensor 1
float* getVoltages1() {
  // Lese analoge Werte von den X-, Y- und Z-Achsen des Sensors
  int analogValue11 = analogRead(analogPin11);
  int analogValue21 = analogRead(analogPin21);
  int analogValue31 = analogRead(analogPin31);

  // Konvertiere die analogen Werte in Spannungen (Volt)
  VoltagesSensor1[0] = analogValue11 * (VREF / ADC_RESOLUTION);
  VoltagesSensor1[1] = analogValue21 * (VREF / ADC_RESOLUTION);
  VoltagesSensor1[2] = analogValue31 * (VREF / ADC_RESOLUTION);

  // Wende den Tiefpassfilter auf die Spannungen an
  for (int i = 0; i < 3; i++) {
    filteredVoltagesSensor1[i] = lowPassFilter(VoltagesSensor1[i], filteredVoltagesSensor1[i]);
  }
  
  return VoltagesSensor1;  // Rückgabe der Rohdaten
}

// Funktion zum Lesen der Spannungen von Sensor 2 (ähnlich wie für Sensor 1)
float* getVoltages2() {
  int analogValue12 = analogRead(analogPin12);
  int analogValue22 = analogRead(analogPin22);
  int analogValue32 = analogRead(analogPin32);

  VoltagesSensor2[0] = analogValue12 * (VREF / ADC_RESOLUTION);
  VoltagesSensor2[1] = analogValue22 * (VREF / ADC_RESOLUTION);
  VoltagesSensor2[2] = analogValue32 * (VREF / ADC_RESOLUTION);

  for (int i = 0; i < 3; i++) {
    filteredVoltagesSensor2[i] = lowPassFilter(VoltagesSensor2[i], filteredVoltagesSensor2[i]);
  }

  return VoltagesSensor2;  // Rückgabe der Rohdaten
}

// Funktion zum Ausdrucken der Spannungen über die serielle Schnittstelle
void printVoltages() {
  float* rawVoltagesSensor1 = getVoltages1();  // Rohdaten von Sensor 1 abrufen
  float* rawVoltagesSensor2 = getVoltages2();  // Rohdaten von Sensor 2 abrufen

  // Ausgabe der Zeit in Millisekunden
  Serial.print("Time (ms): "); Serial.print(millis()); Serial.print(", ");

  // Ausgabe der Rohspannungen von Sensor 1
  Serial.print("Raw Sensor 1: ");
  Serial.print(rawVoltagesSensor1[0], 4); Serial.print(", "); 
  Serial.print(rawVoltagesSensor1[1], 4); Serial.print(", "); 
  Serial.print(rawVoltagesSensor1[2], 4); Serial.print(" | ");

  // Ausgabe der gefilterten Spannungen von Sensor 1
  Serial.print("Filtered Sensor 1: ");
  Serial.print(filteredVoltagesSensor1[0], 4); Serial.print(", ");
  Serial.print(filteredVoltagesSensor1[1], 4); Serial.print(", ");
  Serial.print(filteredVoltagesSensor1[2], 4); Serial.print(" | ");

  // Ausgabe der Rohspannungen von Sensor 2
  Serial.print("Raw Sensor 2: ");
  Serial.print(rawVoltagesSensor2[0], 4); Serial.print(", "); 
  Serial.print(rawVoltagesSensor2[1], 4); Serial.print(", "); 
  Serial.print(rawVoltagesSensor2[2], 4); Serial.print(" | ");

  // Ausgabe der gefilterten Spannungen von Sensor 2
  Serial.print("Filtered Sensor 2: ");
  Serial.print(filteredVoltagesSensor2[0], 4); Serial.print(", ");
  Serial.print(filteredVoltagesSensor2[1], 4); Serial.print(", ");
  Serial.print(filteredVoltagesSensor2[2], 4); Serial.println();
}

// Initialisierung der Arduino-Umgebung
void setup() {
  Serial.begin(115200);  // Starte serielle Kommunikation mit 115200 Baud
  pinMode(analogPin11, INPUT);  // Setze den Pin für die X-Achse von Sensor 1 als Eingang
  pinMode(analogPin21, INPUT);  // Setze den Pin für die Y-Achse von Sensor 1 als Eingang
  pinMode(analogPin31, INPUT);  // Setze den Pin für die Z-Achse von Sensor 1 als Eingang
  pinMode(analogPin12, INPUT);  // Setze den Pin für die X-Achse von Sensor 2 als Eingang
  pinMode(analogPin22, INPUT);  // Setze den Pin für die Y-Achse von Sensor 2 als Eingang
  pinMode(analogPin32, INPUT);  // Setze den Pin für die Z-Achse von Sensor 2 als Eingang
  lastMesszeit = millis();  // Initialisiere die Zeitvariable für das erste Auslesen
}

// Hauptschleife des Programms
void loop() {
  if (millis() - lastMesszeit >= 20) {  // Messe alle 20 Millisekunden (entspricht 5 Hz)
    printVoltages();  // Spannungen auslesen und ausgeben
    lastMesszeit = millis();  // Aktualisiere die Zeitvariable
  }
}
