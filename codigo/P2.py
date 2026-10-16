# Asignatura: Robotica Movil                            #
# Gestion de camara para un F1, el F1 debe de seguir    #
# la linea y completar el circuito.                     #
# Creador: Cristina Homobono Fernández                  #
# Fecha: 16/10/2026                                     #

# ---------------- Imports ----------------
import time
import cv2
import numpy as np
import HAL
import WebGUI
import Frequency

# ======================= PARÁMETROS =======================
# ----- Velocidades -----
V_MAX = 15.5
V_MIN = 5.5            # velocidad en curva cerrada
V_LOST = 2.5           # velocidad mientras busca la línea
W_MAX = 4.0            # saturación de la velocidad angular
W_RATE = 1.0           # cambio máximo de W por iteración
W_SEARCH = 2.0         # giro al buscar la línea perdida

# ----- PID de giro -----
KP_W, KI_W, KD_W = 1.5, 0.0, 0.12 #
NL_GAIN = 1.0          # ganancia no lineal

# ----- PID de velocidad -----
KP_V, KI_V, KD_V = 22.0, 0.0, 0.15 #La ganancia proporcinal, integral y derivativa

# ----- Ajustes generales para el PID -----
D_ALPHA = 0.5          # filtro exponencial de la derivada
I_LIMIT = 1.0          # Limite de la integral

# ----- Importancia de cada ROI -----
W_NEAR, W_FAR = 0.6, 0.4 #Error de guiro

# ----- Regiones de interés de la camara -----
FAR_TOP, FAR_BOTTOM = 250, 320
NEAR_TOP, NEAR_BOTTOM = 330, 400
STRIP_TOP, STRIP_BOTTOM = FAR_TOP, NEAR_BOTTOM

# ----- Rango HSV del rojo -----
RED_LOW_1 = np.array([0, 100, 70])
RED_HIGH_1 = np.array([10, 255, 255])
RED_LOW_2 = np.array([170, 100, 70])
RED_HIGH_2 = np.array([180, 255, 255])

MIN_AREA_FRAC = 0.003  # fracción mínima de píxeles rojos en la ROI para evitar errores con los pianos
LOST_HOLD_TIME = 0.10  # tiempo antes de pasar a la siguiente busqueda
KERNEL = np.ones((5, 5), np.uint8)


# ======================= CONTROLADOR PID =======================
class PID:
    def __init__(self, kp, ki, kd, i_limit=1.0, d_alpha=0.5):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i_limit, self.d_alpha = i_limit, d_alpha
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.prev_error = None
        self.d_filt = 0.0

    def update(self, error, dt):
        self.integral = float(np.clip(self.integral + error * dt,
                                      -self.i_limit, self.i_limit))
        if self.prev_error is None:
            raw_d = 0.0
        else:
            raw_d = (error - self.prev_error) / dt
        self.d_filt = self.d_alpha * raw_d + (1 - self.d_alpha) * self.d_filt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * self.d_filt


# ======================= PERCEPCIÓN (HSV) =======================
def red_mask_hsv(image_bgr):
    strip = image_bgr[STRIP_TOP:STRIP_BOTTOM, :]
    hsv = cv2.cvtColor(strip, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, RED_LOW_1, RED_HIGH_1) | cv2.inRange(hsv, RED_LOW_2, RED_HIGH_2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)
    return mask


def line_error(mask, top, bottom):
    sub = mask[top - STRIP_TOP: bottom - STRIP_TOP, :]
    m = cv2.moments(sub, binaryImage=True)
    if m["m00"] < MIN_AREA_FRAC * sub.size:
        return None, None
    cx = m["m10"] / m["m00"]
    cy = m["m01"] / m["m00"]
    half = sub.shape[1] / 2.0
    return (cx - half) / half, (int(cx), int(cy) + top)


# ======================= ESTADO =======================
pid_w = PID(KP_W, KI_W, KD_W, I_LIMIT, D_ALPHA)
pid_v = PID(KP_V, KI_V, KD_V, I_LIMIT, D_ALPHA)
last_sign = 1.0
last_V, last_W = V_MIN, 0.0
lost_since = None
prev_t = time.time()

# ======================= BUCLE PRINCIPAL =======================
while True:
    image = HAL.getImage()

    if image is not None:
        now = time.time()
        dt = float(np.clip(now - prev_t, 0.005, 0.1))
        prev_t = now

        h, w = image.shape[:2]
        mask = red_mask_hsv(image)
        err_near, pt_near = line_error(mask, NEAR_TOP, NEAR_BOTTOM)
        err_far, pt_far = line_error(mask, FAR_TOP, FAR_BOTTOM)

        debug = image.copy()
        cv2.rectangle(debug, (0, NEAR_TOP), (w, NEAR_BOTTOM), (0, 255, 0), 2)
        cv2.rectangle(debug, (0, FAR_TOP), (w, FAR_BOTTOM), (0, 255, 255), 2)

        # Error de giro: comparativa con la ROI cercana y lejana
        if err_near is not None and err_far is not None:
            err_w = W_NEAR * err_near + W_FAR * err_far
        else:
            err_w = err_near if err_near is not None else err_far

        if err_w is not None:
            if lost_since is not None:           # recuperamos la línea
                pid_w.reset()
                pid_v.reset()
                lost_since = None

            # W: PID con ganancia no lineal
            err_shaped = err_w * (1.0 + NL_GAIN * abs(err_w))
            W = -pid_w.update(err_shaped, dt)
            W = float(np.clip(W, -W_MAX, W_MAX))
            W = last_W + float(np.clip(W - last_W, -W_RATE, W_RATE))

            # V: PID sobre la curva
            e_n = abs(err_near) if err_near is not None else abs(err_w)
            e_f = abs(err_far) if err_far is not None else e_n
            curvature = min(1.0, 0.5 * e_f + 0.3 * e_n + 0.2 * abs(W) / W_MAX)
            braking = pid_v.update(curvature, dt)
            V = float(np.clip(V_MAX - braking, V_MIN, V_MAX))

            # Recordar hacia qué lado se perdió la línea 
            ref = err_far if err_far is not None else err_w
            if abs(ref) > 0.02:
                last_sign = float(np.sign(ref))
            last_V, last_W = V, W

            for pt, col in ((pt_near, (255, 0, 0)), (pt_far, (0, 0, 255))):
                if pt is not None:
                    cv2.circle(debug, pt, 6, col, -1)
        else:
            # Línea perdida: primero mantener rumbo, luego girar hacia la ultima localización
            if lost_since is None:
                lost_since = now
            if now - lost_since < LOST_HOLD_TIME:
                V, W = last_V * 0.6, last_W
            else:
                V, W = V_LOST, -W_SEARCH * last_sign
            last_W = W

        cv2.line(debug, (w // 2, FAR_TOP), (w // 2, NEAR_BOTTOM), (255, 255, 255), 1)
        HAL.setV(V)
        HAL.setW(W)
        WebGUI.showImage(debug)

    Frequency.tick(50)