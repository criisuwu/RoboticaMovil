# ---------------- Imports ----------------
import math
import HAL
import WebGUI
import Frequency

# ---------------- Parámetros ----------------
MAX_V = 20.0          # velocidad máxima
MIN_V = 2.5          # velocidad mínima
KW = 1.0             # ganancia del giro
MAX_W = 2.0          # velocidad angular máxima
K_TARGET = 1.0       # peso de la fuerza de atracción
K_OBS = 4.0          # peso de la fuerza de repulsión
OBS_RANGE = 5.0      # rango para detectar obstaculos
REACHED_DIST = 2.0   # distancia para dar un waypoint por alcanzado

FRONT_CONE = math.radians(25)   # semiapertura del cono frontal de seguridad
STOP_DIST = 1.5                 # Se activa la velocidad minima si el valor esta por debajo de esto
SLOW_DIST = 6.0                 # Si esta mas lejos de esta distancia no se frena

GAP_RANGE = 25.0                # rango para medir hueco libre
GAP_SECTOR = math.radians(70)   # sector frontal usado para medir el hueco
K_GAP = 5.6                     # peso de la fuerza hacia el lado más libre
GAP_MIN_URGENCY = 0.15          # Si esta mas bajo no se aplica la busqueda del hueco

SIDE_WEIGHT = 0.4    # peso de los rayos laterales en la repulsión
ALPHA_DIR = 0.35     # filtro del vector dirección
ALPHA_W = 0.4        # filtro de la velocidad angular
DEADBAND = math.radians(3)      # error angular ignorado para evitar oscilaciones

LASER_SIGN = 1

# ---------------- Funciones ----------------
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

# Devuelve una lista con la distancia y el angulo
def parse_laser_data(laser_data):
    polar = []
    for i in range(min(180, len(laser_data.values))):
        dist = laser_data.values[i]
        if math.isnan(dist) or dist <= 0.0:
            dist = float('inf')
        angle = LASER_SIGN * math.radians(i - 90)
        polar.append((dist, angle))
    return polar

# Transforma las coodernadas absolutas al sistema refencial del F1
def absolute2relative(x_abs, y_abs, robotx, roboty, robott):
    dx = x_abs - robotx
    dy = y_abs - roboty
    x_rel = dx * math.cos(-robott) - dy * math.sin(-robott)
    y_rel = dx * math.sin(-robott) + dy * math.cos(-robott)
    return x_rel, y_rel

# Crea el vector de atracción, lo normaliza y le suma la distancia
def target_vector(target, pose):
    tp = target.getPose()
    tx, ty = absolute2relative(tp.x, tp.y, pose.x, pose.y, pose.yaw)
    norm = math.hypot(tx, ty)
    if norm < 1e-6:
        return 0.0, 0.0, 0.0
    return tx / norm, ty / norm, norm

# Suma los vectores de repulsión y aqui damos mas importancia a los vectores centrales que a los laterales para evitar que el F1 vaya oscilando
def obstacle_vector(laser_polar):
    ox, oy = 0.0, 0.0
    for dist, angle in laser_polar:
        if dist >= OBS_RANGE:
            continue
        dist = max(dist, 0.3)
        mag = 1.0 / dist - 1.0 / OBS_RANGE
        weight = SIDE_WEIGHT + (1.0 - SIDE_WEIGHT) * math.cos(angle) ** 2
        mag *= weight
        ox += -mag * math.cos(angle)
        oy += -mag * math.sin(angle)
    n = max(len(laser_polar), 1)
    return ox / n, oy / n

# Mira la distancia minina dentro del cono frontal para evitar los choques
def front_min_distance(laser_polar):
    d = [dist for dist, ang in laser_polar if abs(ang) <= FRONT_CONE]
    return min(d) if d else OBS_RANGE * 2

# Con un rango valora en que lado hay mas hueco, su es + hay más hueco a la izquierda y si es - hay más hueco a la derecha
def gap_bias(laser_polar):
    left, right = 0.0, 0.0
    for d, a in laser_polar:
        if abs(a) > GAP_SECTOR:
            continue
        val = min(d, GAP_RANGE) * math.cos(a)
        if a > 0:
            left += val
        else:
            right += val
    total = left + right
    return 0.0 if total < 1e-6 else (left - right) / total


current_target = None
ax_f, ay_f = 1.0, 0.0     # vector dirección filtrado
w_prev = 0.0              # velocidad angular anterior
last_side = 1.0           # último lado elegido en indecisión

# ---------------- Bucle principal ----------------
while True:
    pose = HAL.getPose3d()
    laser_data = HAL.getLaserData()

    if len(laser_data.values) == 0:
        Frequency.tick()
        continue

    if current_target is None:
        current_target = WebGUI.getNextTarget()
        if current_target is None:
            HAL.setV(0)
            HAL.setW(0)
            Frequency.tick()
            continue

    # Fuerza de atracción
    tx, ty, dist_to_target = target_vector(current_target, pose)

    if dist_to_target < REACHED_DIST:
        current_target.setReached(True)
        nxt = WebGUI.getNextTarget()
        if nxt is not None:
            current_target = nxt
        tx, ty, dist_to_target = target_vector(current_target, pose)

    #Fuerza de repulsión
    laser_polar = parse_laser_data(laser_data)
    ox, oy = obstacle_vector(laser_polar)

    # Busca el lado con más hueco
    front = front_min_distance(laser_polar)
    urgency = clamp((SLOW_DIST - front) / SLOW_DIST, 0.0, 1.0)
    gy = 0.0
    if urgency > GAP_MIN_URGENCY:
        gy = K_GAP * urgency * gap_bias(laser_polar)

    # Vector dirección = atracción + repulsión + el hueco
    ax = K_TARGET * tx + K_OBS * ox
    ay = K_TARGET * ty + K_OBS * oy + gy

    # Decide el lado mas libre
    if math.hypot(ax, ay) < 0.1:
        gb = gap_bias(laser_polar)
        if abs(gb) > 0.05:
            last_side = 1.0 if gb > 0 else -1.0
        ax, ay = 0.5, 0.5 * last_side

    # Filtro exponencial del vector dirección
    ax_f = ALPHA_DIR * ax + (1 - ALPHA_DIR) * ax_f
    ay_f = ALPHA_DIR * ay + (1 - ALPHA_DIR) * ay_f

    # Control del F1
    angle_err = math.atan2(ay_f, ax_f)
    if abs(angle_err) < DEADBAND:
        angle_err = 0.0

    w_cmd = clamp(KW * angle_err, -MAX_W, MAX_W)
    w = ALPHA_W * w_cmd + (1 - ALPHA_W) * w_prev
    w_prev = w

    # Regula la velocidad al girar y al tener obstáculos delante
    turn_factor = min(abs(angle_err) / (math.pi / 2), 1.0)
    v = MAX_V - (MAX_V - MIN_V) * turn_factor

    front_factor = clamp((front - STOP_DIST) / (SLOW_DIST - STOP_DIST), 0.0, 1.0)
    v = max(MIN_V, v * front_factor)

    HAL.setV(v)
    HAL.setW(w)

    # Depuración
    WebGUI.showForces([tx * 2, ty * 2], [ox * 20, oy * 20], [ax_f * 2, ay_f * 2])
    WebGUI.showLocalTarget([tx * 2, ty * 2])

    Frequency.tick()
