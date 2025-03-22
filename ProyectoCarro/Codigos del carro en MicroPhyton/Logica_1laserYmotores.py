from machine import Pin, PWM, I2C
from vl53l0x import VL53L0X
import time

# Inicialización del bus I2C y el sensor VL53L0X
i2c = I2C(0, sda=Pin(21), scl=Pin(22))
tof = VL53L0X(i2c)

# Habilitar el sensor
try:
    tof.start()
    print("Sensor VL53L0X inicializado correctamente.")
except OSError as e:
    print("Error al inicializar el sensor VL53L0X:", e)
    tof = None  # Desactivar el sensor si no se puede inicializar

# Definición de pines para el puente H L9110
Motor_A1 = PWM(Pin(18))  # PWM
Motor_A2 = PWM(Pin(19))  # PWM
Motor_B1 = PWM(Pin(2))   # PWM
Motor_B2 = PWM(Pin(4))   # PWM

# Velocidades en escala de 0 a 1023
velocidad_inicial = 800  # Velocidad inicial (600 en escala de 0-1023)
velocidad_final = 480    # Velocidad final (280 en escala de 0-1023)
velocidad_giro = 650     # Velocidad para giros (550 en escala de 0-1023)

# Función para mover los motores hacia adelante
def mover_adelante(velocidad):
    Motor_A1.duty(0)               # Motor A: Giro hacia adelante
    Motor_A2.duty(velocidad)       # Motor A: Velocidad
    Motor_B1.duty(velocidad)       # Motor B: Giro hacia adelante
    Motor_B2.duty(0)               # Motor B: Velocidad

# Función para girar a la derecha
def girar_derecha(velocidad):
    Motor_A1.duty(velocidad)       # Motor A: Giro hacia atrás
    Motor_A2.duty(0)               # Motor A: Velocidad
    Motor_B1.duty(velocidad)       # Motor B: Giro hacia adelante
    Motor_B2.duty(0)               # Motor B: Velocidad

# Función para detener los motores
def detener_motores():
    Motor_A1.duty(0)
    Motor_A2.duty(0)
    Motor_B1.duty(0)
    Motor_B2.duty(0)

# Función para medir la distancia con el sensor VL53L0X
def medir_distancia():
    if tof is None:
        print("Sensor no disponible.")
        return 9999  # Valor por defecto si el sensor no está disponible
    try:
        distancia = tof.read()
        return distancia
    except OSError as e:
        print("Error al leer el sensor:", e)
        return 9999  # Valor por defecto si hay un error

try:
    # Mover los motores a la velocidad inicial (600 en escala de 0-1023)
    mover_adelante(velocidad_inicial)
    print("Motores encendidos a velocidad inicial:", velocidad_inicial)

    # Esperar 500 ms
    time.sleep_ms(500)

    # Reducir la velocidad a la velocidad final (280 en escala de 0-1023)
    mover_adelante(velocidad_final)
    print("Motores reducidos a velocidad final:", velocidad_final)

    # Bucle principal
    while True:
        # Medir la distancia
        distancia = medir_distancia()
        print("Distancia medida:", distancia, "mm")

        # Lógica de navegación
        if distancia <= 100:  # Si hay un obstáculo a 100 mm o menos
            print("Obstáculo detectado. Girando a la derecha...")
            while distancia <= 100:  # Girar hasta que no haya obstáculo
                girar_derecha(velocidad_giro)
                time.sleep_ms(100)  # Esperar un poco antes de la siguiente medición
                distancia = medir_distancia()  # Medir la distancia nuevamente
                print("Distancia durante el giro:", distancia, "mm")
            detener_motores()  # Detener los motores después del giro
            print("Giro completado. Continuando...")
        else:
            # No hay obstáculo, seguir avanzando
            mover_adelante(velocidad_final)

        time.sleep_ms(100)  # Esperar 100 ms antes de la siguiente lectura

except KeyboardInterrupt:
    # Detener los motores si se interrumpe el programa
    detener_motores()
    print("Motores detenidos.")