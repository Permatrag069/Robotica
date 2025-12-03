from ultralytics import YOLO
import cv2
import serial
import time
from config import (
    MODEL_NAME,
    SERIAL_PORT,
    FRAME_WIDTH,
    FRAME_HEIGHT,
    TARGET_AREA,
    AREA_DEADBAND,
    X_DEADBAND,
    Y_DEADBAND,
    STEP_DELAY,
    STATE_TRACKING,
    STATE_COLLECTED,
    MAX_STRAWBERRIES
)

# -----------------------------
# CARGAR MODELO YOLO
# -----------------------------
print(f"⏳ Cargando modelo {MODEL_NAME}...")
try:
    # Si la Raspberry Pi tiene un acelerador como Coral, el modo puede cambiar aquí.
    # Para la Pi estándar, se asume CPU.
    model = YOLO(MODEL_NAME)
    print("📋 Clases del modelo detectadas:", model.names)
except Exception as e:
    print(f"❌ Error cargando el modelo: {e}")
    exit()

# -----------------------------
# INICIAR CÁMARA Y ARDUINO
# -----------------------------
# *** CAMBIO CLAVE 1: Índice de la cámara para Raspberry Pi ***
# Si estás usando la cámara oficial de la Pi (Pi Camera Module), usa 0.
# Si estás usando una cámara USB, el índice 1 podría ser correcto, pero 0 es más común.
# Si el script falla, prueba 0, 1 o diferentes índices.
# También es común usar el backend V4L2 en Linux.
cap = cv2.VideoCapture(0) 

# *** MEJORA: Forzar la resolución definida en config.py ***
# Esto es crucial para que el TARGET_AREA y los deadbands funcionen como están calibrados.
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

window_name = "Detector de Fresa"
cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

arduino = None
try:
    # Verificación del Baud Rate: Es 115200, ¡perfecto! Coincide con el Arduino.
    # El SERIAL_PORT se toma del config.py, donde ya lo cambiaste a /dev/ttyACM0
    arduino = serial.Serial(SERIAL_PORT, 115200, timeout=0.01)
    time.sleep(2)
    print(f"🟢 Arduino conectado en {SERIAL_PORT}")

    print("⏳ Enviando H para llevar el brazo a HOME...")
    arduino.write(b'H')
    time.sleep(5)
    print("✅ Brazo en HOME. Iniciando seguimiento.")

except Exception as e:
    print(f"⚠️ AVISO: No se pudo conectar al Arduino en {SERIAL_PORT}. Corriendo en modo visualización. Error: {e}")

# =================================================================
# VARIABLES PARA MOVIMIENTO DE RUEDAS
# =================================================================
last_detection_time = time.time()    # última vez que vimos una fresa
moving = False                       # si las ruedas están moviéndose
move_start_time = 0.0                # cuándo empezaron a moverse

NO_DETECT_TIME = 3.0    # segundos sin fresa antes de moverse
MOVE_DURATION = 1.5     # AHORA: 1.5 segundos de movimiento cada vez

# =================================================================
# BUCLE PRINCIPAL DE COSECHA
# =================================================================
strawberry_count = 0
running = True

try:
    while strawberry_count < MAX_STRAWBERRIES and running:

        current_state = STATE_TRACKING
        print(f"\n--- 🍓 INICIANDO BUCLE #{strawberry_count + 1} de {MAX_STRAWBERRIES} ---")

        frame_count = 0
        results = None

        while current_state == STATE_TRACKING and running:

            ret, frame = cap.read()
            if not ret:
                print("❌ Error leyendo cámara. ¿Está bien el índice de la cámara (0, 1, etc.)?")
                running = False
                break

            frame_count += 1
            h, w, _ = frame.shape
            center_x = w // 2
            center_y = h // 2

            cv2.line(frame, (center_x, 0), (center_x, h), (100, 100, 100), 1)
            cv2.line(frame, (0, center_y), (w, center_y), (100, 100, 100), 1)

            # Detección de objetos (cada 3 frames)
            # *** CAMBIO CLAVE 2: Adaptar frecuencia de detección para Pi ***
            # Reducimos la frecuencia de detección para evitar sobrecargar la CPU de la Pi.
            if frame_count % 5 == 0: # Detectar cada 5 frames (antes era cada 3)
                results = model(frame, verbose=False, conf=0.25, imgsz=FRAME_WIDTH)

            best_det_coords = None
            best_area = 0

            if results:
                for r in results:
                    for box in r.boxes:
                        cls_id = int(box.cls[0])
                        class_name = model.names[cls_id].lower()
                        x1, y1, x2, y2 = map(int, box.xyxy[0])

                        if class_name == 'ripe':
                            area = (x2 - x1) * (y2 - y1)
                            if area > best_area:
                                best_area = area
                                best_det_coords = (x1, y1, x2, y2)
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        else:
                            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)

            # -----------------------------
            # CONTROL DEL BRAZO Y RUEDAS
            # -----------------------------
            now = time.time()

            if best_det_coords and arduino:
                # ✅ HAY UNA FRESA DETECTADA

                # Guardamos que vimos una fresa recientemente
                last_detection_time = now

                # Si las ruedas estaban avanzando, las detenemos
                if moving:
                    print("🛑 Fresa detectada: deteniendo ruedas (S).")
                    # Nota: Asumo que 'S' es el comando para 'Stop' en el Arduino.
                    arduino.write(b'S') 
                    moving = False

                # Control del brazo (igual que antes)
                x1, y1, x2, y2 = best_det_coords
                cx = (x1 + x2) // 2
                cy = (y1 + y2) // 2

                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

                error_x = cx - center_x
                error_y = cy - center_y
                error_area = best_area - TARGET_AREA

                print(f"Error X: {error_x} (Dead {X_DEADBAND}) | Error Y: {error_y} (Dead {Y_DEADBAND})")
                print(f"Error Area: {error_area} (Dead {AREA_DEADBAND}) | Area Actual: {best_area}")

                arduino_action = "N/A"
                action_sent = False

                # 1. TRIGGER DE RECOLECCIÓN
                if (abs(error_x) < X_DEADBAND and
                    abs(error_y) < Y_DEADBAND and
                    abs(error_area) < AREA_DEADBAND):

                    arduino_action = 'G'
                    print(f"✨ Fresa alineada. Enviando comando: {arduino_action}")
                    arduino.write(b'G')
                    current_state = STATE_COLLECTED
                    # El tiempo de espera es largo porque el Arduino realiza toda la secuencia de GRAB/DROP/REPOSO
                    time.sleep(15) 
                    break

                # 2. SEGUIMIENTO CON EL BRAZO (Ajustado para enviar un solo comando por ciclo)
                
                # Prioridad 1: Centrado Lateral (Base)
                if error_x > X_DEADBAND:
                    arduino_action = 'R'
                    arduino.write(b'R')
                    action_sent = True
                elif error_x < -X_DEADBAND:
                    arduino_action = 'L'
                    arduino.write(b'L')
                    action_sent = True
                
                # Prioridad 2: Altura (Codo)
                elif error_y > Y_DEADBAND:
                    arduino_action = 'D'
                    arduino.write(b'D')
                    action_sent = True
                elif error_y < -Y_DEADBAND:
                    arduino_action = 'U'
                    arduino.write(b'U')
                    action_sent = True
                    
                # Prioridad 3: Distancia (Hombro)
                elif error_area < -AREA_DEADBAND:
                    arduino_action = 'F'
                    arduino.write(b'F')
                    action_sent = True
                elif error_area > AREA_DEADBAND:
                    arduino_action = 'B'
                    arduino.write(b'B')
                    action_sent = True

                # Notificación
                if action_sent:
                    print(f"🚀 Enviando acción: {arduino_action}")
                
                # Pequeña pausa entre comandos
                time.sleep(STEP_DELAY)

            else:
                # 🚫 NO HAY FRESA DETECTADA (Lógica de avance de ruedas)
                if arduino:
                    # Si han pasado más de NO_DETECT_TIME sin detección y no estamos moviendo, avanzamos
                    if (now - last_detection_time > NO_DETECT_TIME) and not moving:
                        print(f"⏳ {NO_DETECT_TIME}s sin fresa → avanzando ruedas {MOVE_DURATION}s (W).")
                        # Nota: Asumo que 'W' es el comando para 'Forward'/'Walk' en el Arduino.
                        arduino.write(b'W')
                        moving = True
                        move_start_time = now

                    # Si ya estamos moviendo y pasaron MOVE_DURATION, detenemos
                    if moving and (now - move_start_time > MOVE_DURATION):
                        print(f"🛑 {MOVE_DURATION}s de avance cumplidos → deteniendo ruedas (S).")
                        arduino.write(b'S')
                        moving = False

            # Mostrar imagen y leer teclas
            cv2.imshow(window_name, frame)
            key = cv2.waitKey(20) & 0xFF

            if key == ord('q'):
                running = False
                break
            elif key == ord('p'):
                print("🆘 Tecla 'p' presionada: enviando comando de REPOSO (P) al brazo.")
                if arduino:
                    arduino.write(b'P')

        # Lógica después de cosechar una fresa
        if current_state == STATE_COLLECTED:
            strawberry_count += 1
            print(f"✅ Fresa #{strawberry_count} cosechada.")
            # Por seguridad, detenemos ruedas (si el Arduino tiene un mecanismo de ruedas)
            if arduino:
                arduino.write(b'S')
            time.sleep(2)

        if not running:
            break

except KeyboardInterrupt:
    print("\n\n🔴 Interrupción de teclado (Ctrl+C).")
    running = False

finally:
    # Detener ruedas por seguridad
    if arduino:
        arduino.write(b'S')

    cap.release()
    cv2.destroyAllWindows()
    if arduino:
        arduino.close()

    if strawberry_count == MAX_STRAWBERRIES:
        print(f"\n🎉 ¡Objetivo alcanzado! Se han cosechado {MAX_STRAWBERRIES} fresas.")
    elif not running:
        print("\n👋 Programa terminado por intervención del usuario.")
    else:
        print("\n👋 Programa terminado.")