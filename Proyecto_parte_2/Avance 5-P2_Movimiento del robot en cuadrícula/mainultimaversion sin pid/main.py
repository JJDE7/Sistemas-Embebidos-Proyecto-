from MPU6050 import MPU6050
from motores import Motores
from sensores import SensoresLaser
from pid_controller import PIDController
import time
import machine

# Configuración
velocidad_inicial, velocidad_final, velocidad_giro = 600, 400, 750
umbral_obstaculo = 130  # Umbral de distancia para detectar obstáculos (en mm)
angulo_giro = 90  # Ángulo absoluto a girar (grados)
kp = 2.0  # Ganancia proporcional para el PID

# Inicialización de componentes
def inicializar_componente(nombre, clase):
    try:
        print(f"Inicializando {nombre}...")
        componente = clase()
        print(f"{nombre} inicializado correctamente.")
        return componente
    except Exception as e:
        print(f"Error al inicializar {nombre}: {e}")
        machine.reset()

mpu = inicializar_componente("MPU6050", MPU6050)
motores = inicializar_componente("Motores", Motores)
sensores = inicializar_componente("SensoresLaser", SensoresLaser)
mpu.calibrarGiroscopio()

# Inicialización del controlador PID
pid = PIDController(kp)

# Bucle principal
try:
    print("Iniciando movimiento inicial...")
    yaw_inicial = mpu.actualizarYaw()  # Establecer yaw inicial al inicio
    print(f"Yaw inicial establecido en: {yaw_inicial:.2f}°")

    motores.mover_adelante(velocidad_inicial)
    time.sleep_ms(300)
    motores.mover_adelante(velocidad_final)

    while True:
        # Leer distancias solo al inicio de cada ciclo
        distancia_frontal, distancia_izquierda = sensores.leer_distancias()
        yaw_actual = mpu.actualizarYaw()

        print(f"Distancia frontal: {distancia_frontal} mm, Distancia izquierda: {distancia_izquierda} mm, Yaw actual: {yaw_actual:.2f}°")

        # Calcular desviación respecto al yaw inicial
        error = yaw_inicial - yaw_actual
        ajuste = pid.calcular_ajuste(error)

        # Monitorear los valores de error y ajuste
        print(f"[PID] Yaw inicial: {yaw_inicial:.2f}° | Yaw actual: {yaw_actual:.2f}° | Error: {error:.2f}° | Ajuste: {ajuste:.2f}")

        if distancia_frontal > umbral_obstaculo:
            # Ajustar velocidades de las ruedas para corregir el error
            if error > 0:
                # Desviación hacia la derecha: reducir velocidad de la rueda derecha
                velocidad_derecha = max(min(velocidad_final - ajuste, 1023), 0)
                velocidad_izquierda = max(min(velocidad_final, 1023), 0)
            else:
                # Desviación hacia la izquierda: reducir velocidad de la rueda izquierda
                velocidad_derecha = max(min(velocidad_final, 1023), 0)
                velocidad_izquierda = max(min(velocidad_final - ajuste, 1023), 0)

            # Monitorear las velocidades ajustadas
            print(f"[VELOCIDADES] Izquierda: {velocidad_izquierda:.2f} | Derecha: {velocidad_derecha:.2f}")

            motores.mover_ruedas(velocidad_izquierda, velocidad_derecha)

            time.sleep_ms(100)  # Pequeña pausa para movimiento continuo
        else:
            motores.detener_motores()
            time.sleep_ms(200)  # Espera para estabilización
            
            # Decidir dirección de giro
            if distancia_izquierda > umbral_obstaculo:
                print(f"Obstáculo frontal. Girando izquierda {angulo_giro}°")
                motores.mover_ruedas(velocidad_giro, -velocidad_giro)  # Giro sobre el eje
                target_yaw = yaw_inicial + angulo_giro
            else:
                print(f"Obstáculo frontal. Girando derecha {angulo_giro}°")
                motores.mover_ruedas(-velocidad_giro, velocidad_giro)  # Giro sobre el eje
                target_yaw = yaw_inicial - angulo_giro

            # Monitorear el giro hasta alcanzar el ángulo objetivo
            while True:
                yaw_actual = mpu.actualizarYaw()
                error = abs(target_yaw - yaw_actual)

                # Normalizar el error para cruce de 360°
                if error > 180:
                    error = 360 - error

                print(f"[GIRO] Yaw actual: {yaw_actual:.1f}° | Target: {target_yaw:.1f}° | Error: {error:.1f}°")
                
                if error < 5:  # Margen de 5°
                    break
                    
                time.sleep_ms(10)

            motores.detener_motores()

            # Corregir el desfase en el giro usando PID
            yaw_actual = mpu.actualizarYaw()
            error = target_yaw - yaw_actual

            # Normalizar el error para cruzar 360°
            if error > 180:
                error -= 360
            elif error < -180:
                error += 360

            print(f"[CORRECCIÓN] Yaw actual: {yaw_actual:.2f}° | Target: {target_yaw:.2f}° | Error inicial: {error:.2f}°")

            # Usar el PID para corregir el ángulo si el error es significativo
            while abs(error) > 5:  # Margen permitido de ±5°
                ajuste = pid.calcular_ajuste(error)
                
                # Corregir el ángulo girando sobre el eje
                if ajuste > 0:
                    motores.mover_ruedas(ajuste, -ajuste)  # Giro hacia la derecha
                else:
                    motores.mover_ruedas(-ajuste, ajuste)  # Giro hacia la izquierda

                time.sleep_ms(50)  # Pausa para aplicar el ajuste
                yaw_actual = mpu.actualizarYaw()
                error = target_yaw - yaw_actual

                # Normalizar el error para cruzar 360°
                if error > 180:
                    error -= 360
                elif error < -180:
                    error += 360

                print(f"[CORRECCIÓN] Yaw actual: {yaw_actual:.2f}° | Target: {target_yaw:.2f}° | Error: {error:.2f}°")

            # Una vez corregido, detener motores y reanudar movimiento en línea recta
            motores.detener_motores()
            yaw_inicial = target_yaw % 360  # Actualizar yaw inicial después de la corrección
            print(f"Corrección completada. Nuevo yaw inicial establecido en: {yaw_inicial:.2f}°")
            time.sleep_ms(300)  # Pausa post-corrección

except KeyboardInterrupt:
    print("\nDeteniendo motores...")
    motores.detener_motores()
except Exception as e:
    print(f"Error crítico: {e}")
    machine.reset()