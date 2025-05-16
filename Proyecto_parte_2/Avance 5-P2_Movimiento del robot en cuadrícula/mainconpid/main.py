from MPU6050 import MPU6050
from motores import Motores
from sensores import SensoresLaser
from pid_controller import PIDController
import time
import machine

# Configuración
velocidad_inicial, velocidad_final, velocidad_giro = 450, 380, 850
umbral_obstaculo = 100  # Umbral de distancia para detectar obstáculos (en mm)
angulo_giro = 90  # Ángulo absoluto a girar (grados)
kp = 100  # Aumenté la ganancia proporcional para mejor control

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
pid = PIDController(kp)  # Ahora con PID completo

def girar_90_grados(direccion):
    global yaw_inicial

    # Configuración de ángulo objetivo
    target_yaw = (yaw_inicial - angulo_giro) % 360 if direccion == "derecha" else (yaw_inicial + angulo_giro) % 360
    margen_error = 2  # Margen aceptable de error final (grados)
    pre_stop_margin = 1  # Grados antes del objetivo para giro inverso
    tiempo_giro_inverso = 50  # Duración del giro inverso en ms

    print(f"[GIRO] Inicio: {yaw_inicial:.1f}° | Target: {target_yaw:.1f}°")

    # 1. Giro principal
    if direccion == "derecha":
        motores.girar_derecha(velocidad_giro)
    else:
        motores.girar_izquierda(velocidad_giro)

    # Giro principal con detección de proximidad al objetivo
    while True:
        yaw_actual = mpu.actualizarYaw()
        error = target_yaw - yaw_actual

        # Normalización del error
        if error > 180:
            error -= 360
        elif error < -180:
            error += 360

        # Iniciar giro inverso justo antes de alcanzar el objetivo
        if abs(error) <= pre_stop_margin:
            print(f"[GIRO] Dentro del margen pre-stop ({pre_stop_margin}°). Iniciando giro inverso.")
            motores.detener_motores()
            if direccion == "derecha":
                motores.girar_izquierda(400)  # Giro inverso a velocidad mínima
            else:
                motores.girar_derecha(400)
            time.sleep_ms(tiempo_giro_inverso)
            motores.detener_motores()
            break

        time.sleep_ms(10)

    # Esperar estabilización tras detener motores
    time.sleep_ms(200)

    # Leer el ángulo final para actualizar la referencia
    yaw_inicial = mpu.actualizarYaw()
    print(f"[GIRO] Completado. Yaw final: {yaw_inicial:.1f}°")
    time.sleep_ms(300)


# Bucle principal (se mantiene igual)
try:
    print("Iniciando movimiento inicial...")
    yaw_inicial = mpu.actualizarYaw()
    print(f"Yaw inicial establecido en: {yaw_inicial:.2f}°")

    motores.mover_adelante(velocidad_inicial)
    time.sleep_ms(300)
    motores.mover_adelante(velocidad_final)

    while True:
        distancia_frontal, distancia_izquierda = sensores.leer_distancias()
        yaw_actual = mpu.actualizarYaw()

        print(f"Distancia frontal: {distancia_frontal} mm, Distancia izquierda: {distancia_izquierda} mm, Yaw actual: {yaw_actual:.2f}°")

        error = yaw_inicial - yaw_actual
        ajuste = pid.calcular_ajuste(error)

        print(f"[PID] Yaw inicial: {yaw_inicial:.2f}° | Yaw actual: {yaw_actual:.2f}° | Error: {error:.2f}° | Ajuste: {ajuste:.2f}")

        if distancia_frontal > umbral_obstaculo:
            if error > 0:
                velocidad_izquierda = max(min(velocidad_final - ajuste, 900), 300)
                velocidad_derecha = max(min(velocidad_final + 40 + ajuste, 900), 250)
            else:
                velocidad_derecha = max(min(velocidad_final + ajuste, 900), 250) 
                velocidad_izquierda = max(min(velocidad_final + 40 - ajuste, 900), 250)

            print(f"[VELOCIDADES] Izquierda: {velocidad_izquierda:.2f} | Derecha: {velocidad_derecha:.2f}")
            motores.mover_ruedas(velocidad_izquierda, velocidad_derecha)
            time.sleep_ms(20)
        else:
            motores.detener_motores()
            time.sleep_ms(200)
            
            # Decisión de giro corregida
            if distancia_izquierda > umbral_obstaculo:
                print(f"Espacio libre a la izquierda. Girando IZQUIERDA {angulo_giro}°")
                girar_90_grados("izquierda")
            else:
                print(f"Espacio libre a la derecha. Girando DERECHA {angulo_giro}°")
                girar_90_grados("derecha")
            
except KeyboardInterrupt:
    print("\nDeteniendo motores...")
    motores.detener_motores()
except Exception as e:
    print(f"Error crítico: {e}")
    machine.reset()


