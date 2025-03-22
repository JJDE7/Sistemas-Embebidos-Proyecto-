from machine import Pin, I2C
from vl53l0x import VL53L0X
from MPU6050 import MPU6050
import time

# Configuración de pines XSHUT para cada sensor
XSHUT_PINS = [15, 13]  # Pines XSHUT para el sensor 1 y sensor 2
SENSOR_COUNT = len(XSHUT_PINS)

# Inicialización del bus I2C
i2c = I2C(0, sda=Pin(21), scl=Pin(22))

# Lista para almacenar los objetos de los sensores
sensors = []

# Función para reiniciar y configurar un sensor
def setup_sensor(xshut_pin, address):
    # Apagar el sensor
    xshut = Pin(xshut_pin, Pin.OUT)
    xshut.value(0)
    time.sleep_ms(10)

    # Encender el sensor
    xshut.value(1)
    time.sleep_ms(10)

    # Inicializar el sensor con la dirección predeterminada (0x29)
    sensor = VL53L0X(i2c, address=0x29)
    sensor.start()

    # Cambiar la dirección I2C del sensor
    sensor.change_address(address)
    return sensor

# Configuración inicial de los sensores
for i in range(SENSOR_COUNT):
    address = 0x30 + i  # Direcciones únicas: 0x30, 0x31, etc.
    sensor = setup_sensor(XSHUT_PINS[i], address)
    sensors.append(sensor)

# Crear una instancia del MPU6050
mpu = MPU6050()

# Calibrar el giroscopio (solo una vez, o cuando sea necesario)
mpu.calibrarGiroscopio()

# Bucle principal
try:
    while True:
        # Leer y mostrar las distancias de todos los sensores
        for i in range(SENSOR_COUNT):
            try:
                distancia = sensors[i].read()
                print(f"Sensor {i}: {distancia} mm", end="\t")
            except OSError as e:
                print(f"Error en el sensor {i}: {e}")
            time.sleep_ms(10)  # Pequeño retraso entre lecturas de sensores
        
        # Obtener el valor de yaw
        try:
            yaw = mpu.actualizarYaw()
            print(f"Yaw: {yaw:.2f} grados")  # Imprimir el valor de yaw
        except OSError as e:
            print(f"Error en el MPU6050: {e}")
        
        time.sleep_ms(100)  # Esperar 100 ms antes de la siguiente lectura

except KeyboardInterrupt:
    # Detener los sensores si se interrumpe el programa
    for sensor in sensors:
        sensor.stop()
    print("Sensores detenidos.")
    print("Programa detenido.")