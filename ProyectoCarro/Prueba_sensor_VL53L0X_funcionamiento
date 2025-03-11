//                                                        Robots Covenant
//                                 Oscar Mauricio Cabezas, Juan José Devia, Jhon Alexander Cartagena

#include <Wire.h>
#include "Adafruit_VL53L0X.h"

//medida leída por el sensor de distancia
int Distancia;

// Instancias de los sensores
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Definición de pines para el puente H L9110

#define Motor_B1 2   
#define Motor_B2 4   
#define Motor_A1 18  
#define Motor_A2 19  

void setup() {

    Serial.begin(9600);

    // Configurar pines del motor
    pinMode(Motor_A1, OUTPUT);
    pinMode(Motor_A2, OUTPUT);
    pinMode(Motor_B1, OUTPUT);
    pinMode(Motor_B2, OUTPUT);
    
    Wire.begin();

    Serial.println("Iniciando sensor VL53L0X...");

    if (!lox.begin(0x29)) {
        Serial.println("Error al iniciar sensor");
    }
}

void loop() {

    Medir_VL53L0X();
    delay(100);
    
    adelante();
    delay(500);

    izquierda();
    delay(500);

    derecha();
    delay(500);

    atras();
    delay(500);

    detener();
    delay(500);

}

void Medir_VL53L0X() {
  VL53L0X_RangingMeasurementData_t medida;
  
  lox.rangingTest(&medida, false); // Obtener la medición

  if (medida.RangeStatus != 4) { // Si la medición es válida
    Serial.print("Distancia: ");
    Serial.print(medida.RangeMilliMeter);
    Serial.println(" mm");
  } else {
    Serial.println("Distancia fuera de rango");
  }
}

void adelante() {
    analogWrite(Motor_A1, HIGH);
    analogWrite(Motor_A2, LOW);
    analogWrite(Motor_B1, HIGH);
    analogWrite(Motor_B2, LOW);
}

void atras() {
    analogWrite(Motor_A1, LOW);
    analogWrite(Motor_A2, HIGH);
    analogWrite(Motor_B1, LOW);
    analogWrite(Motor_B2, HIGH);
}

void izquierda() {
    analogWrite(Motor_A1, HIGH);
    analogWrite(Motor_A2, LOW);
    analogWrite(Motor_B1, LOW);
    analogWrite(Motor_B2, HIGH);
}

void derecha() {
    analogWrite(Motor_A1, LOW);
    analogWrite(Motor_A2, HIGH);
    analogWrite(Motor_B1, HIGH);
    analogWrite(Motor_B2, LOW);
}

void detener() {
    analogWrite(Motor_A1, LOW);
    analogWrite(Motor_A2, LOW);
    analogWrite(Motor_B1, LOW);
    analogWrite(Motor_B2, LOW);
}