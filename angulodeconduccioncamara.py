import cv2
import numpy as np
import serial
import time

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
centro_imagen = 640 // 2
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2) # Esperar a que Arduino se reinicie al abrir el puerto

if not cap.isOpened():
    print("No se pudo abrir la cámara")
    exit()

while True:
    ret, img_bgr = cap.read()
    if not ret:
        print("No se pudo leer la cámara")
        break

    # Filtros de imagen
    img_grey = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    img_blur = cv2.GaussianBlur(img_grey, (3,3), 0, 0)
    img_canny = cv2.Canny(img_blur, 50, 150)

    # ROI (Región de Interés) Enfocada en la parte baja/media de la pantalla
    vertices = np.array([[(30,480),(30,280),(610,280),(610,480)]], dtype=np.int32)
    img_roi = np.zeros_like(img_grey)
    cv2.fillPoly(img_roi, vertices, 255)
    img_mask = cv2.bitwise_and(img_canny, img_roi)

    # Transformada de Hough
    lines = cv2.HoughLinesP(img_mask, 2, np.pi/180, 40, np.array([]), minLineLength=40, maxLineGap=15)
    img_lines = np.zeros_like(img_bgr)

    left_xs = []
    right_xs = []

    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                # Evitar líneas horizontales
                if abs(y2 - y1) < 5: 
                    continue
                
                x_medio = (x1 + x2) / 2
                
                # Clasificar la línea según la mitad de la pantalla en la que aparece
                if x_medio < 320:
                    left_xs.append(x_medio)
                    cv2.line(img_lines, (x1, y1), (x2, y2), [0, 0, 255], 3) # Rojo = Izquierda
                else:
                    right_xs.append(x_medio)
                    cv2.line(img_lines, (x1, y1), (x2, y2), [0, 255, 0], 3) # Verde = Derecha

    # LÓGICA DE MOVIMIENTO

    # CASO A: Se perdieron por completo las líneas
    if len(left_xs) == 0 and len(right_xs) == 0:
        
        vy = 0
        vw = 0
            
        # Enviar comando de búsqueda a Arduino
        ser.write(f"{vy},{vw}\n".encode())
        
    # CASO B: El robot ve las líneas (Seguimiento de carril)
    else:
        
        # Calcular el centro estimado del carril (El centro ideal es 320)
        if len(left_xs) > 0 and len(right_xs) > 0:
            centro_carril = (np.mean(left_xs) + np.mean(right_xs)) / 2
        elif len(left_xs) > 0:
            # Si solo ve la izquierda, estima dónde debería estar el centro sumando un offset
            centro_carril = np.mean(left_xs) + 140 
        else:
            # Si solo ve la derecha, estima el centro restando el offset
            centro_carril = np.mean(right_xs) - 140

        # Calcular el error respecto al centro de la pantalla (320)
        error = centro_carril - 320
        
        # Control Proporcional (Kp = 0.25).giro muy lento o muy brusco.
        Kp = 0.25
        vw = int(error * Kp)
        
        # Limitar la velocidad de giro máxima para evitar derrapes bruscos
        vw = max(min(vw, 60), -60)
        
        # Ajuste de velocidad lineal: si va derecho va a 50, si el error es grande gira 40 para girar mejor
        vy = 50 if abs(error) < 40 else 45 

        # Enviar comando de conducción en formato "vy,vw"
        ser.write(f"{vy},{vw}\n".encode())
        print(f"Enviado -> vy: {vy} | vw: {vw}")

    # Leer respuesta de confirmación del Arduino (opcional para depurar)
    if ser.in_waiting > 0:
        print("ARDUINO:", ser.readline().decode().strip())

    # Mostrar la cámara con las líneas pintadas
    #img_final = cv2.addWeighted(img_bgr, 0.7, img_lines, 1, 0)
    img_mask_bgr = cv2.cvtColor(img_mask, cv2.COLOR_GRAY2BGR)
    img_final = cv2.addWeighted(img_mask_bgr, 0.7, img_lines, 1, 0)
    cv2.imshow("Result", img_final)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
ser.close()