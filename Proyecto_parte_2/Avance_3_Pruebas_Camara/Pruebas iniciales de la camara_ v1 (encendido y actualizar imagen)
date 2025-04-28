import time
import random
import cv2
import numpy as np
import RPi.GPIO as GPIO
from picamera2 import Picamera2

BUTTON_PIN = 17

GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

picam2 = Picamera2()
resolutions = [(640, 480), (800, 600), (1024, 768), (1280, 720), (1920, 1080)]

picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480), "format": "RGB888"}))
picam2.start()

try:
    while True:
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)  # Convertir a HSV
        cv2.imshow("Camera Preview (HSV)", frame)

        if GPIO.input(BUTTON_PIN) == GPIO.LOW:
            new_res = random.choice(resolutions)
            print(f"Cambiando resolucion a: {new_res}")
            picam2.stop()
            picam2.configure(picam2.create_preview_configuration(main={"size": new_res, "format": "RGB888"}))
            picam2.start()
            time.sleep(0.5)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pass

GPIO.cleanup()
picam2.stop()
cv2.destroyAllWindows()