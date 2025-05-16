from MPU6050 import MPU6050
from motores import Motores
from sensores import SensoresLaser
from pid_controller import PIDController
import time
import machine

# Configuración
velocidad_inicial, velocidad_final, velocidad_giro = 450, 380, 780
kp = 100  # Ganancia proporcional para el controlador PID
angulo_giro = 90
# Dimensiones del entorno
celda = 70  # Tamaño de cada celda en mm (7 cm)
offset_x = 80  # Desplazamiento en X desde el borde hasta el recuadro imaginario
offset_y = 70  # Desplazamiento en Y desde el borde hasta el recuadro imaginario

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

# Variables de posición
posicion_actual = [1, 1]  # El robot comienza en la celda (1, 1)
orientacion_actual = "NORTE"  # NORTE, SUR, ESTE, OESTE

def girar_90_grados(direccion):
    global yaw_inicial

    # Configuración de ángulo objetivo
    target_yaw = (yaw_inicial - angulo_giro) % 360 if direccion == "derecha" else (yaw_inicial + angulo_giro) % 360
    margen_error = 2  # Margen aceptable de error final (grados)
    pre_stop_margin = 9  # Grados antes del objetivo para giro inverso
    tiempo_giro_inverso = 100  # Duración del giro inverso en ms

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
    
def actualizar_orientacion(direccion_giro):
    global orientacion_actual
    orientaciones = ["NORTE", "ESTE", "SUR", "OESTE"]
    indice_actual = orientaciones.index(orientacion_actual)

    if direccion_giro == "derecha":
        orientacion_actual = orientaciones[(indice_actual + 1) % 4]
    elif direccion_giro == "izquierda":
        orientacion_actual = orientaciones[(indice_actual - 1) % 4]
    elif direccion_giro == "180":
        orientacion_actual = orientaciones[(indice_actual + 2) % 4]

    print(f"[ORIENTACIÓN] Actualizada a {orientacion_actual}")

def moverse_a_celda(destino):
    global posicion_actual, orientacion_actual

    destino_x, destino_y = destino
    actual_x, actual_y = posicion_actual

    print(f"[MOVIMIENTO] De celda {posicion_actual} a celda {destino}")

    # Movimiento en el eje Y (adelante o atrás)
    if destino_y != actual_y:
        distancia_requerida_y = abs(destino_y - actual_y) * celda
        direccion_y = "adelante" if destino_y > actual_y else "atras"

        print(f"[EJE Y] Movimiento {direccion_y} {distancia_requerida_y} mm")

        if direccion_y == "adelante" and orientacion_actual == "NORTE":
            # Leer la distancia inicial desde el sensor frontal
            distancia_frontal, _ = sensores.leer_distancias()
            print(f"distancia_frontal: {distancia_frontal :.3f}")
            distancia_inicial = distancia_frontal
            print(f"distancia_incial: {distancia_inicial :.3f}")
            distancia_objetivo = distancia_inicial - distancia_requerida_y
            print(f"distancia_objetivo: {distancia_objetivo :.3f}")

            motores.mover_adelante(velocidad_inicial)
            while True:
                distancia_frontal, _ = sensores.leer_distancias()  # Actualiza la lectura frontal
                yaw_actual = mpu.actualizarYaw()

                # Calcular error y ajuste proporcional
                error = yaw_inicial - yaw_actual
                ajuste = pid.calcular_ajuste(error)

                print(f"[PID] Yaw inicial: {yaw_inicial:.2f}° | Yaw actual: {yaw_actual:.2f}° | Error: {error:.2f}° | Ajuste: {ajuste:.2f}")

                # Ajustar velocidades de las ruedas
                if error > 0:
                    velocidad_izquierda = max(min(velocidad_final - ajuste, 900), 300)
                    velocidad_derecha = max(min(velocidad_final + 40 + ajuste, 900), 250)
                else:
                    velocidad_derecha = max(min(velocidad_final + ajuste, 900), 250)
                    velocidad_izquierda = max(min(velocidad_final + 40 - ajuste, 900), 300)

                print(f"[VELOCIDADES] Izquierda: {velocidad_izquierda:.2f} | Derecha: {velocidad_derecha:.2f}")
                motores.mover_ruedas(velocidad_izquierda, velocidad_derecha)

                # Verificar si se alcanzó el objetivo
                if distancia_frontal <= distancia_objetivo:
                    break

                time.sleep_ms(20)

        motores.detener_motores()

    # Movimiento en el eje X (derecha o izquierda)
    if destino_x != actual_x:
        distancia_requerida_x = abs(destino_x - actual_x) * celda
        direccion_x = "derecha" if destino_x > actual_x else "izquierda"

        print(f"[EJE X] Movimiento {direccion_x} {distancia_requerida_x} mm")

        # Girar en la dirección necesaria
        if direccion_x == "derecha" and orientacion_actual != "ESTE":
            print("[MOVIMIENTO] Girando a la derecha para alinearse con el eje X.")
            girar_90_grados("derecha")
            actualizar_orientacion("derecha")
        elif direccion_x == "izquierda" and orientacion_actual != "OESTE":
            print("[MOVIMIENTO] Girando a la izquierda para alinearse con el eje X.")
            girar_90_grados("izquierda")
            actualizar_orientacion("izquierda")
        
        # Leer la distancia inicial desde el sensor frontal
        distancia_frontal, _ = sensores.leer_distancias()
        print(f"distancia_frontal: {distancia_frontal :.3f}")
        distancia_inicial = distancia_frontal
        print(f"distancia_inicial: {distancia_inicial :.3f}")
        distancia_objetivo = distancia_inicial - distancia_requerida_x
        print(f"distancia_objetivo: {distancia_objetivo :.3f}")
        motores.mover_adelante(velocidad_inicial)
        
        while True:
            distancia_frontal, _ = sensores.leer_distancias()
            yaw_actual = mpu.actualizarYaw()

            # Calcular error y ajuste proporcional
            error = yaw_inicial - yaw_actual
            ajuste = pid.calcular_ajuste(error)

            print(f"[PID] Yaw inicial: {yaw_inicial:.2f}° | Yaw actual: {yaw_actual:.2f}° | Error: {error:.2f}° | Ajuste: {ajuste:.2f}")

            # Ajustar velocidades de las ruedas
            if error > 0:
                velocidad_izquierda = max(min(velocidad_final - ajuste, 900), 300)
                velocidad_derecha = max(min(velocidad_final + 40 + ajuste, 900), 250)
            else:
                velocidad_derecha = max(min(velocidad_final + ajuste, 900), 250)
                velocidad_izquierda = max(min(velocidad_final + 40 - ajuste, 900), 300)

            print(f"[VELOCIDADES] Izquierda: {velocidad_izquierda:.2f} | Derecha: {velocidad_derecha:.2f}")
            motores.mover_ruedas(velocidad_izquierda, velocidad_derecha)
            print(f"distancia_frontal: {distancia_frontal :.3f}") 
            # Verificar si se alcanzó el objetivo
            if distancia_frontal <= distancia_objetivo:
                break

            time.sleep_ms(20)

        motores.detener_motores()

    # Actualizar posición actual
    posicion_actual = [destino_x, destino_y]
    print(f"[MOVIMIENTO] Posición actualizada a {posicion_actual}")

# Bucle principal
try:
    print("Iniciando movimiento inicial...")
    yaw_inicial = mpu.actualizarYaw()
    print(f"Yaw inicial establecido en: {yaw_inicial:.2f}°")

    while True:
        # Aquí se define el destino deseado
        destino = [8, 6]  # Ejemplo: ir a la celda (x, y)
        moverse_a_celda(destino)

        # Pausa para detener el programa (puedes cambiar esta lógica para nuevas instrucciones)
        time.sleep(5)
        
except KeyboardInterrupt:
    print("\nDeteniendo motores...")
    motores.detener_motores()
except Exception as e:
    print(f"Error crítico: {e}")
    machine.reset()
 


