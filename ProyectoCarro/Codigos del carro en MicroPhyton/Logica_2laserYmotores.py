from machine import Pin, I2C, PWM
from vl53l0x import VL53L0X
import time
from time import ticks_ms

# Configuración de pines XSHUT para cada sensor
XSHUT_PINS = [15, 13]  # Pines XSHUT para el sensor frontal (15) y sensor izquierdo (13)
SENSOR_COUNT = len(XSHUT_PINS)

# Inicialización del bus I2C
i2c = I2C(0, sda=Pin(21), scl=Pin(22))

# Lista para almacenar los objetos de los sensores
sensors = []

# Función para reiniciar y configurar un sensor
def setup_sensor(xshut_pin, address):
    xshut = Pin(xshut_pin, Pin.OUT)
    xshut.value(0)
    time.sleep_ms(10)
    xshut.value(1)
    time.sleep_ms(10)
    sensor = VL53L0X(i2c, address=0x29)
    sensor.start()
    sensor.change_address(address)
    return sensor

# Configuración inicial de los sensores
for i in range(SENSOR_COUNT):
    address = 0x30 + i
    sensor = setup_sensor(XSHUT_PINS[i], address)
    sensors.append(sensor)

# Definición de pines para el puente H L9110
Motor_A1 = PWM(Pin(18))
Motor_A2 = PWM(Pin(19))
Motor_B1 = PWM(Pin(2))
Motor_B2 = PWM(Pin(4))

# Configurar la frecuencia PWM (1 kHz)
Motor_A1.freq(1000)
Motor_A2.freq(1000)
Motor_B1.freq(1000)
Motor_B2.freq(1000)

# Velocidades en escala de 0 a 1023
velocidad_inicial = 500
velocidad_final = 380
velocidad_giro = 450

# Función para mover los motores hacia adelante
def mover_adelante(velocidad):
    Motor_A1.duty(0)
    Motor_A2.duty(velocidad)
    Motor_B1.duty(velocidad)
    Motor_B2.duty(0)

# Función para girar a la derecha
def girar_derecha(velocidad):
    Motor_A1.duty(velocidad)
    Motor_A2.duty(0)
    Motor_B1.duty(velocidad)
    Motor_B2.duty(0)

# Función para girar a la izquierda
def girar_izquierda(velocidad):
    Motor_A1.duty(0)
    Motor_A2.duty(velocidad)
    Motor_B1.duty(0)
    Motor_B2.duty(velocidad)

# Función para detener los motores
def detener_motores():
    Motor_A1.duty(0)
    Motor_A2.duty(0)
    Motor_B1.duty(0)
    Motor_B2.duty(0)

# Bucle principal
try:
    mover_adelante(velocidad_inicial)
    print("Motores encendidos a velocidad inicial:", velocidad_inicial)
    time.sleep_ms(500)

    mover_adelante(velocidad_final)
    print("Motores reducidos a velocidad final:", velocidad_final)

    last_time = ticks_ms()

    while True:
        current_time = ticks_ms()
        if current_time - last_time >= 100:  # Ejecutar cada 100 ms
            last_time = current_time

            # Leer distancias de los sensores
            distancia_frontal = sensors[0].read()
            distancia_izquierda = sensors[1].read()

            print(f"Distancia frontal: {distancia_frontal} mm, Distancia izquierda: {distancia_izquierda} mm")

            # Lógica de navegación
            if distancia_frontal > 130:
                mover_adelante(velocidad_final)
            else:
                if distancia_izquierda > 130:
                    print("Obstáculo al frente. Girando a la izquierda...")
                    girar_izquierda(velocidad_giro)
                    time.sleep_ms(500)
                    detener_motores()
                else:
                    print("Obstáculo al frente y a la izquierda. Girando a la derecha...")
                    girar_derecha(velocidad_giro)
                    time.sleep_ms(500)
                    detener_motores()

except KeyboardInterrupt:
    detener_motores()
    for sensor in sensors:
        sensor.stop()
    print("Motores y sensores detenidos.")