from picamera2 import Picamera2
import cv2
import numpy as np

# Inicializar camara
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format": 'RGB888', "size": (640, 480)})
picam2.configure(config)
picam2.start()

# Variables para calibracion
lower_color = np.array([0, 100, 100])
upper_color = np.array([10, 255, 255])
calibrating = False

# Funcion para calibracion con clic
def mouse_callback(event, x, y, flags, param):
    global lower_color, upper_color, calibrating
    
    if event == cv2.EVENT_LBUTTONDOWN and calibrating:
        pixel = hsv[y,x]
        print(f"HSV: {pixel}")
        lower_color = np.array([pixel[0]-10, pixel[1]-50, pixel[2]-50])
        upper_color = np.array([pixel[0]+10, pixel[1]+50, pixel[2]+50])
        print(f"Rango inferior: {lower_color}")
        print(f"Rango superior: {upper_color}")
        calibrating = False

cv2.namedWindow("Tracking")
cv2.setMouseCallback("Tracking", mouse_callback)

# Bucle principal
while True:
    frame = picam2.capture_array()
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    
    # Crear mascara y encontrar contornos
    mask = cv2.inRange(hsv, lower_color, upper_color)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Procesar contornos
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 500:  # Filtrar contornos pequenos
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            
            # Calcular centro
            center_x = x + w//2
            center_y = y + h//2
            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
            print(f"Posicion detectada: ({center_x}, {center_y})")
    
    # Mostrar imagenes
    cv2.imshow("Tracking", frame)
    cv2.imshow("Mascara", mask)
    
    # Controles de teclado
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('c'):
        calibrating = True
        print("Modo calibracion: Haz clic en el color objetivo")

# Limpieza
cv2.destroyAllWindows()
picam2.stop()