# main.py
from MPU6050 import MPU6050
import time

# Crear una instancia del MPU6050
mpu = MPU6050()

# Calibrar el giroscopio (solo una vez, o cuando sea necesario)
mpu.calibrarGiroscopio()

# Bucle principal
try:
    while True:
        yaw = mpu.actualizarYaw()  # Obtener el valor de yaw
        print(f"Yaw: {yaw:.2f} grados")  # Imprimir el valor de yaw
        time.sleep_ms(100)  # Esperar 100 ms antes de la siguiente lectura

except KeyboardInterrupt:
    print("Programa detenido.")