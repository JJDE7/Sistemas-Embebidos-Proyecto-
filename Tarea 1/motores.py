from machine import Pin, PWM
import time

class Motores:
    def __init__(self):
        # Primero configurar los pines como salidas digitales en LOW
        self.Motor_A1 = Pin(18, Pin.OUT, value=0)
        self.Motor_A2 = Pin(19, Pin.OUT, value=0)
        self.Motor_B1 = Pin(2, Pin.OUT, value=0)
        self.Motor_B2 = Pin(4, Pin.OUT, value=0)
        
        # Pequeña pausa para estabilización
        time.sleep_ms(20)
        
        # Ahora convertirlos a PWM
        self.Motor_A1 = PWM(self.Motor_A1)
        self.Motor_A2 = PWM(self.Motor_A2)
        self.Motor_B1 = PWM(self.Motor_B1)
        self.Motor_B2 = PWM(self.Motor_B2)
        
        # Configurar frecuencia PWM
        self.Motor_A1.freq(1000)
        self.Motor_A2.freq(1000)
        self.Motor_B1.freq(1000)
        self.Motor_B2.freq(1000)
        
        # Asegurar que todos los PWM empiecen en 0
        self.detener_motores()
        time.sleep_ms(20)

        # Estado inicial
        self.estado_actual = "detenido"

    def mover_adelante(self, velocidad):
        """
        Mueve el robot hacia adelante a una velocidad uniforme.
        :param velocidad: Velocidad para ambas ruedas (0-1023).
        """
        velocidad = max(0, min(1023, int(velocidad)))
        self.Motor_A1.duty(0)
        self.Motor_A2.duty(velocidad)
        self.Motor_B1.duty(velocidad)
        self.Motor_B2.duty(0)
        self.estado_actual = "andando"

    def girar_derecha(self, velocidad):
        """
        Gira el robot hacia la izquierda.
        :param velocidad: Velocidad del giro.
        """
        velocidad = max(0, min(1023, int(velocidad)))
        self.Motor_A1.duty(velocidad)
        self.Motor_A2.duty(0)
        self.Motor_B1.duty(velocidad)
        self.Motor_B2.duty(0)
        self.estado_actual = "girando"

    def girar_izquierda(self, velocidad):
        """
        Gira el robot hacia la derecha.
        :param velocidad: Velocidad del giro.
        """
        velocidad = max(0, min(1023, int(velocidad)))
        self.Motor_A1.duty(0)
        self.Motor_A2.duty(velocidad)
        self.Motor_B1.duty(0)
        self.Motor_B2.duty(velocidad)
        self.estado_actual = "girando"

    def detener_motores(self):
        """
        Detiene ambos motores.
        """
        self.Motor_A1.duty(0)
        self.Motor_A2.duty(0)
        self.Motor_B1.duty(0)
        self.Motor_B2.duty(0)
        self.estado_actual = "detenido"

    def mover_ruedas(self, velocidad_izquierda, velocidad_derecha):
        """
        Ajusta las velocidades de las ruedas izquierda y derecha.
        :param velocidad_izquierda: Velocidad para la rueda izquierda.
        :param velocidad_derecha: Velocidad para la rueda derecha. 
        """
        velocidad_izquierda = max(0, min(1023, int(velocidad_izquierda)))
        velocidad_derecha = max(0, min(1023, int(velocidad_derecha)))

        self.Motor_A1.duty(0)
        self.Motor_A2.duty(velocidad_derecha)
        self.Motor_B1.duty(velocidad_izquierda)
        self.Motor_B2.duty(0)
        self.estado_actual = "andando"

    def en_movimiento(self):
        """
        Retorna True si el robot está en movimiento.
        """
        return self.estado_actual != "detenido"