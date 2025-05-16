from machine import I2C, Pin
import time

class MPU6050:
    def __init__(self):
        self.i2c = I2C(0, sda=Pin(21), scl=Pin(22), freq=400000)  # Usar el mismo bus I2C que los sensores
        self.address = 0x68  # Dirección I2C del MPU6050

        # Verificar si el dispositivo está conectado
        devices = self.i2c.scan()
        if self.address not in devices:
            raise RuntimeError(f"MPU6050 no encontrado en la dirección {hex(self.address)}")

        # Configurar MPU6050
        self.i2c.writeto_mem(self.address, 0x6B, bytes([0]))  # Despertar
        self.i2c.writeto_mem(self.address, 0x1B, bytes([0x00]))  # Giro ±250°/s
        self.i2c.writeto_mem(self.address, 0x1C, bytes([0x00]))  # Acel ±2g

        self.yaw = 0.0
        self.last_time = time.ticks_ms()
        self.offset_z = 0.0  # Offset del giroscopio Z

    def LeerRegistro(self, Direccion):
        try:
            data = self.i2c.readfrom_mem(self.address, Direccion, 2)  # Leer 2 bytes
            Registro = (data[0] << 8) | data[1]  # Combinar bytes
            
            if Registro > 32767:
                Registro -= 65536  # Convertir a complemento a 2
            
            return Registro
        except OSError as e:
            print(f"Error al leer el registro {hex(Direccion)}: {e}")
            return 0

    def LeerGiroscopio(self):
        try:
            Gx = self.LeerRegistro(0x43) / 131.0
            Gy = self.LeerRegistro(0x45) / 131.0
            Gz = (self.LeerRegistro(0x47) / 131.0) - self.offset_z  # Aplicar offset
            return (Gx, Gy, Gz)
        except OSError as e:
            print(f"Error al leer el giroscopio: {e}")
            return (0, 0, 0)

    def calibrarGiroscopio(self, muestras=1000):
        print("Calibrando giroscopio... No muevas el robot.")
        offset_z = 0.0
        for _ in range(muestras):
            Gx, Gy, Gz = self.LeerGiroscopio()
            offset_z += Gz
            time.sleep_ms(10)  # Intervalo entre lecturas
        self.offset_z = offset_z / muestras  # Offset promedio
        print(f"Calibración completada. Offset Z: {self.offset_z:.2f}°/s")

    def actualizarYaw(self):
        try:
            current_time = time.ticks_ms()
            dt = (current_time - self.last_time) / 1000.0
            self.last_time = current_time
            
            Gz = self.LeerGiroscopio()[2]  # Solo eje Z
            self.yaw += Gz * dt  # Integración directa
            
            return self.yaw
        except OSError as e:
            print(f"Error al actualizar el yaw: {e}")
            return 0.0