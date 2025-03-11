//                                                        Robots Covenant
//                                 Oscar Mauricio Cabezas, Juan José Devia, Jhon Alexander Cartagena

// Definición de pines para el puente H: L9110

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

}

void loop() {

    adelante();
    delay(500);

    izquierda();
    delay(500);
    
    derecha();
    delay(500);
    
    detener();
    delay(500);
    
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
    analogWrite(Motor_B2, LOW);
}