from machine import Pin, PWM
import time

# Definición de pines para el puente H L9110
Motor_A1 = PWM(Pin(18))  # PWM
Motor_A2 = PWM(Pin(19))  # PWM
Motor_B1 = PWM(Pin(2))   # PWM
Motor_B2 = PWM(Pin(4))   # PWM

# Velocidades en escala de 0 a 1023
velocidad_inicial = 800  # Velocidad inicial (600 en escala de 0-1023)
velocidad_final = 480    # Velocidad final (280 en escala de 0-1023)

# Función para mover los motores hacia adelante
def mover_adelante(velocidad):
    Motor_A1.duty(0)               # Motor A: Giro hacia adelante
    Motor_A2.duty(velocidad)       # Motor A: Velocidad
    Motor_B1.duty(velocidad)       # Motor B: Giro hacia adelante
    Motor_B2.duty(0)               # Motor B: Velocidad

# Función para detener los motores
def detener_motores():
    Motor_A1.duty(0)
    Motor_A2.duty(0)
    Motor_B1.duty(0)
    Motor_B2.duty(0)

try:
    # Mover los motores a la velocidad inicial (600 en escala de 0-1023)
    mover_adelante(velocidad_inicial)
    print("Motores encendidos a velocidad inicial:", velocidad_inicial)

    # Esperar 500 ms
    time.sleep_ms(200)

    # Reducir la velocidad a la velocidad final (280 en escala de 0-1023)
    mover_adelante(velocidad_final)
    print("Motores reducidos a velocidad final:", velocidad_final)

    # Mantener los motores en movimiento (puedes agregar más lógica aquí)
    while True:
        time.sleep_ms(100)  # Mantener el programa en ejecución

except KeyboardInterrupt:
    # Detener los motores si se interrumpe el programa
    detener_motores()
    print("Motores detenidos.")