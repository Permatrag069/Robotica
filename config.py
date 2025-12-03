# config.py

# -----------------------------
# PARÁMETROS GENERALES
# -----------------------------
MODEL_NAME      = "best.pt"      # Nombre del modelo YOLO
SERIAL_PORT     = "/dev/ttyACM0"
 # Puerto serial de comunicación con Arduino

# -----------------------------
# PARÁMETROS DE CÁMARA
# -----------------------------
FRAME_WIDTH     = 320            # Ancho del marco de la cámara
FRAME_HEIGHT    = 320            # Altura del marco de la cámara

# -----------------------------
# PARÁMETROS DE CONTROL DEL BRAZO (CALIBRADOS)
# -----------------------------
# 1. DISTANCIA (Eje Z - Hombro)
# TARGET_AREA: Área ideal para agarrar la fresa. (Calibrado a 89600)
# >>>>>>>>>> VALOR DE GRAB CORREGIDO <<<<<<<<<<
TARGET_AREA    = 65500     

# AREA_DEADBAND: Tolerancia de distancia. (1000 píxeles)
AREA_DEADBAND  = 700      

# 2. CENTRADO (Ejes X/Y - Base/Codo)
# X_DEADBAND / Y_DEADBAND: Tolerancia de centrado. (20 píxeles)
X_DEADBAND     = 40        
Y_DEADBAND     = 40        

# 3. VELOCIDAD
STEP_DELAY     = 0.05      # Delay entre comandos enviados

# -----------------------------
# ESTADOS DEL SISTEMA
# -----------------------------
STATE_TRACKING = 0          
STATE_COLLECTED = 1         

# -----------------------------
# PARÁMETROS DEL BUCLE
# -----------------------------
MAX_STRAWBERRIES = 1        # Número de fresas a cosechar antes de finalizar