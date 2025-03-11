//                                                        Robots Covenant
//                                 Oscar Mauricio Cabezas, Juan José Devia, Jhon Alexander Cartagena

#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// Variables para los sensores
int DistanciaFrontal;
int DistanciaLateral;

// Instancias de los sensores
Adafruit_VL53L0X loxFrontal = Adafruit_VL53L0X();
Adafruit_VL53L0X loxLateral = Adafruit_VL53L0X();

// Definición de pines para el puente H L9110
#define Motor_B1 2   
#define Motor_B2 4   
#define Motor_A1 18  
#define Motor_A2 19  

// Pines de control de encendido de los sensores
#define XSHUT_FRONTAL 15
#define XSHUT_LATERAL 16

void setup() {

    Serial.begin(9600);
    Serial.println("Iniciando sensores VL53L0X...");

    // Configurar pines del motor
    pinMode(Motor_A1, OUTPUT);
    pinMode(Motor_A2, OUTPUT);
    pinMode(Motor_B1, OUTPUT);
    pinMode(Motor_B2, OUTPUT);
    
    // Configurar pines XSHUT para controlar la alimentación de los sensores
    pinMode(XSHUT_FRONTAL, OUTPUT);
    pinMode(XSHUT_LATERAL, OUTPUT);

    Wire.begin();

    // Inicializar sensores con direcciones distintas
    digitalWrite(XSHUT_FRONTAL, LOW);
    digitalWrite(XSHUT_LATERAL, LOW);
    
    delay(10);
    digitalWrite(XSHUT_FRONTAL, HIGH);
    delay(10);

    if (!loxFrontal.begin(0x30)) {
        Serial.println("Error al iniciar sensor frontal");
    }

    delay(10);
    digitalWrite(XSHUT_LATERAL, HIGH);
    delay(10);
    
    //Comentar el IF si solo se tiene un sensor
    if (!loxFrontal.begin(0x31)) {
        Serial.println("Error al iniciar sensor lateral");
    }
}

void loop() {

    DistanciaFrontal = Medir_VL53L0X(loxFrontal);
    DistanciaLateral = Medir_VL53L0X(loxLateral);//Comentar si solo se tiene un sensor

    Serial.print("Frontal: "); Serial.println(DistanciaFrontal);
    Serial.print("Lateral: "); Serial.println(DistanciaLateral);//Comentar si solo se tiene un sensor

    //lógica de funcionamiento para navegación (Sin tener en cuenta el segundo sensor)
    if (DistanciaFrontal > 100) {
        adelante();
        Serial.println("Avanzando");
    } else {
        izquierda();
        Serial.println("Girando");
    }
    
    delay(100);
}

float Medir_VL53L0X(Adafruit_VL53L0X &sensor) {
    VL53L0X_RangingMeasurementData_t medida;
    sensor.rangingTest(&medida, false);
    return (medida.RangeStatus != 4) ? medida.RangeMilliMeter : 9999; // 9999 indica fuera de rango
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