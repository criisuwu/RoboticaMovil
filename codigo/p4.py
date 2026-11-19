# Asignatura: Robotica Movil                            #
# Navegación global de un taxi seleccionando un destino #
# Gradient Path Planning: campo = propagación + penalización #
# Creador: Cristina Homobono Fernández                  #
# Fecha: 19/11/2026                                     #

# ---------------- Imports ----------------
import math
import heapq
import traceback

import numpy as np
import HAL
import WebGUI
import Frequency

# ---------------- Parámetros ----------------
MAP_URL = '/resources/exercises/global_navigation/images/cityLargenBin.png'
SAFE_DIST = 8             # radio de influencia del obstáculo
PENALTY = 1.5             # peso del subcampo de penalización
PEN_WEIGHT = 0.3          # peso del subcampo de penalización en la suma
INFLATE = 1               # celdas prohibidas alrededor de los obstáculos
LOOKAHEAD = 3.5           # distancia del punto al que mira el coche
V_MAX, V_MIN = 8.0, 1.5
V_TURN = 0.8
W_MAX = 3.0 
K_W = 1.8                 # ganancia del giro
GOAL_RADIUS = 2.5         # distancia para dar el destino por alcanzado
STUCK_LIMIT = 20          # iteraciones sin descenso antes de replanificar

# Vecinos en 8 direcciones: (fila, columna, coste del paso)
# Es decir las celdas de movimiento
NEIGHBORS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]

# ---------------- Funciones ----------------
# Esta función devuelve la posición y la orientación del coche (x, y, yaw)
def get_pose():
    p = HAL.getPose3d()
    yaw = getattr(p, 'yaw', None)
    if yaw is None:
        yaw = getattr(p, 'theta', 0.0)
    return float(p.x), float(p.y), float(yaw)


# Esta función es la normalicación del ángulo
def normalize(a):
    return math.atan2(math.sin(a), math.cos(a))


# ---------------- Mapa ----------------
# Con esto realizamos la carga del mapa y lo pasa a escala de grises si viene en color
grid_map = np.array(WebGUI.getMap(MAP_URL))
if grid_map.ndim == 3:
    grid_map = grid_map.mean(axis=2)
H, W = grid_map.shape
gm = grid_map.astype(np.float64)


# Esta función pasa de coordenadas del mundo a celda (fila, columna). swap indica si worldToGrid devuelve el orden contrario
def raw_cell(wx, wy, swap):
    g = WebGUI.worldToGrid([wx, wy])
    gx, gy = int(round(g[0])), int(round(g[1]))
    return (gx, gy) if swap else (gy, gx)


# Esta función comprueba que la celda está dentro del mapa
def in_map(r, c):
    return 0 <= r < H and 0 <= c < W


# Esta función puntúa cada orden posible de worldToGrid: si el coche está sobre la carretera, delante y detrás de él también debería haber carretera
def score_orientation(swap, x, y, yaw):
    r, c = raw_cell(x, y, swap)
    if not in_map(r, c):
        return -1, None
    # rv es el valor de la carretera, calculada mediante la mediana de los píxeles alrededor del coche
    rv = float(np.median(gm[max(0, r - 2): r + 3, max(0, c - 2): c + 3]))
    s = 0
    for d in (-20, -12, -6, 6, 12, 20):
        rr, cc = raw_cell(x + d * math.cos(yaw), y + d * math.sin(yaw), swap)
        if in_map(rr, cc) and abs(gm[rr, cc] - rv) <= 60:
            s += 1
    return s, rv


# Con esto podemos probar los dos órdenes y elegir el que mejor encaja para la ruta
x0, y0, yaw0 = get_pose()
s_false, rv_false = score_orientation(False, x0, y0, yaw0)
s_true, rv_true = score_orientation(True, x0, y0, yaw0)

SWAP_XY = s_true > s_false
road_value = rv_true if SWAP_XY else rv_false
if road_value is None:
    road_value = 255.0

obstacle_np = np.abs(gm - road_value) > 60


# Esta función pasa de coordenadas del mundo a celda, sin salirse del mapa
def to_cell(world_xy):
    r, c = raw_cell(world_xy[0], world_xy[1], SWAP_XY)
    return min(max(r, 0), H - 1), min(max(c, 0), W - 1)


# Esta función pasa de celda a coordenadas del mundo
def to_world(cell):
    r, c = cell
    arg = [int(r), int(c)] if SWAP_XY else [int(c), int(r)]
    w = WebGUI.gridToWorld(arg)
    return float(w[0]), float(w[1])


# Esta función muestra el campo en la GUI, si falla se ignora
def safe_show_cost(cost):
    try:
        arr = np.array(cost, dtype=np.float64)
        finite = np.isfinite(arr)
        img = np.full(arr.shape, 255, dtype=np.uint8)
        if finite.any():
            img[finite] = (arr[finite] / arr[finite].max() * 254).astype(np.uint8)
        img[obstacle_np] = 255
        WebGUI.showNumpy(img)
    except Exception:
        pass


# Esta función calcula la distancia de cada celda al obstáculo más cercano
def obstacle_distance():
    big = 10 ** 6
    dist = [[big] * W for _ in range(H)]
    pq = []
    for r, c in np.argwhere(obstacle_np):
        dist[r][c] = 0.0
        pq.append((0.0, int(r), int(c)))
    heapq.heapify(pq)
    while pq:
        d0, r, c = heapq.heappop(pq)
        if d0 > dist[r][c] or d0 > SAFE_DIST:
            continue
        for dr, dc, step in NEIGHBORS:
            nr, nc = r + dr, c + dc
            if 0 <= nr < H and 0 <= nc < W and dist[nr][nc] > d0 + step:
                dist[nr][nc] = d0 + step
                heapq.heappush(pq, (d0 + step, nr, nc))
    return dist


dist_obs = obstacle_distance()
dist_obs_np = np.array(dist_obs, dtype=np.float64)
blocked_np = dist_obs_np <= INFLATE
blocked = blocked_np.tolist()

# ---------------- Penalización por cercanía a obstáculos ----------------
field_pen = np.maximum(0.0, SAFE_DIST - dist_obs_np) ** 2 * PENALTY * PEN_WEIGHT


# Esta función mira si la celda está bloqueada y si es el caso busca la libre más cercana, probando en anillos cada vez más grandes
def nearest_free(cell):
    r0, c0 = cell
    if not blocked[r0][c0]:
        return cell
    for rad in range(1, 80):
        for r in range(max(0, r0 - rad), min(H, r0 + rad + 1)):
            for c in range(max(0, c0 - rad), min(W, c0 + rad + 1)):
                if not blocked[r][c]:
                    return (r, c)
    return cell


# ---------------- Propagación a destino ----------------
# Esta funcion hace que se expanda desde el destino solo con el coste de los pasos sin valorar la penalización
def propagation(target):
    inf = float('inf')
    cost = [[inf] * W for _ in range(H)]
    cost[target[0]][target[1]] = 0.0
    pq = [(0.0, target)]
    while pq:
        cur, (r, c) = heapq.heappop(pq)
        if cur > cost[r][c]:
            continue
        for dr, dc, step in NEIGHBORS:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < H and 0 <= nc < W) or blocked[nr][nc]:
                continue
            if dr != 0 and dc != 0 and (blocked[r + dr][c] or blocked[r][c + dc]):
                continue
            new = cur + step
            if new < cost[nr][nc]:
                cost[nr][nc] = new
                heapq.heappush(pq, (new, (nr, nc)))
    return np.array(cost, dtype=np.float64)


field = None
field_prop = None


# ---------------- CAMPO TOTAL = propagación + penalización ----------------
# Esta función sirve para generar el campo
def build_field(goal_cell):
    global field_prop
    field_prop = propagation(goal_cell)
    return field_prop + field_pen


# Esta función mira si la celda no tiene valor finito en el campo (bloqueada), busca la celda finita más cercana al coche
def entry_cell(cell, f):
    r0, c0 = cell
    if np.isfinite(f[r0, c0]):
        return cell
    for rad in range(1, 30):
        ra, rb = max(0, r0 - rad), min(H, r0 + rad + 1)
        ca, cb = max(0, c0 - rad), min(W, c0 + rad + 1)
        sub = f[ra:rb, ca:cb]
        fin = np.argwhere(np.isfinite(sub))
        if len(fin):
            d = np.hypot(fin[:, 0] + ra - r0, fin[:, 1] + ca - c0)
            i, j = fin[int(np.argmin(d))]
            return (ra + int(i), ca + int(j))
    return None


# Esta función va valorando en el campo cual de las coordenadas tiene menor valor con el descenso del gradiente discreto
def next_cell(cell, f):
    r, c = cell
    best, best_v = cell, f[r, c]
    for dr, dc, _ in NEIGHBORS:
        nr, nc = r + dr, c + dc
        if not in_map(nr, nc):
            continue
        v = f[nr, nc]
        if not np.isfinite(v):
            continue
        if dr != 0 and dc != 0 and (blocked[r + dr][c] or blocked[r][c + dc]):
            continue
        if v < best_v:
            best, best_v = (nr, nc), v
    return best

# Esta función devuelve los puntos que va encontrando. Si no hay ruta guardada se consulta el campo cada vez.
def descent_points(x, y, f):
    cell = entry_cell(to_cell((x, y)), f)
    if cell is None:
        return []
    pts = []
    for _ in range(150):
        nxt = next_cell(cell, f)
        if nxt == cell:
            break
        cell = nxt
        wx, wy = to_world(cell)
        pts.append((wx, wy))
        if math.hypot(wx - x, wy - y) >= 2 * LOOKAHEAD:
            break
    return pts


goal_world = None


# Esta función calcula el campo para el destino indicado
def plan(target_world):
    global field, goal_world
    x, y, _ = get_pose()

    # Si ya estamos en el destino no planifica nada
    if math.hypot(target_world[0] - x, target_world[1] - y) < GOAL_RADIUS:
        field = None
        goal_world = None
        return

    goal = nearest_free(to_cell(target_world))
    new_field = build_field(goal)
    safe_show_cost(new_field)

    field = new_field
    # Si desde la posición del coche no se puede llega a ninguna celda con campo finito, no hay camino
    if entry_cell(to_cell((x, y)), field) is None:
        print("No hay camino hasta el destino")
        field = None
        goal_world = None
        return
    goal_world = target_world


last_target = None
stuck_count = 0

# ---------------- Bucle principal ----------------
while True:
    try:
        # Destino marcado con clic en el mapa
        target = WebGUI.getTargetPose()
        target = (float(target[0]), float(target[1])) if target is not None else None

        # Si el destino es nuevo se para el coche y se recalcula el campo
        if target is not None and target != last_target:
            last_target = target
            stuck_count = 0
            HAL.setV(0)
            HAL.setW(0)
            plan(target)

        if field is not None:
            x, y, yaw = get_pose()

            gx, gy = goal_world
            dist_goal = math.hypot(gx - x, gy - y)
            if dist_goal < GOAL_RADIUS:
                # Destino alcanzado: se para y se espera un nuevo clic
                HAL.setV(0)
                HAL.setW(0)
                field = None
                last_target = None
            else:
                # Se pregunta al campo hacia dónde bajar
                pts = descent_points(x, y, field)
                if not pts:
                    pts = descent_points(x, y, field_prop)
                if not pts:
                    # Sin dirección de descenso se para y si sigue sin haber se replanifica
                    HAL.setV(0)
                    HAL.setW(0)
                    stuck_count += 1
                    if stuck_count >= STUCK_LIMIT:
                        print("Sin descenso: replanificando")
                        stuck_count = 0
                        plan(goal_world)
                else:
                    stuck_count = 0
                    tx, ty = pts[-1]
                    for px, py in pts:
                        if math.hypot(px - x, py - y) >= LOOKAHEAD:
                            tx, ty = px, py
                            break

                    # Error entre la orientación del coche y el punto objetivo
                    err = normalize(math.atan2(ty - y, tx - x) - yaw)

                    if abs(err) > math.pi / 3:
                        tx, ty = pts[-1]
                        err = normalize(math.atan2(ty - y, tx - x) - yaw)

                    fx, fy = pts[-1]
                    far_err = abs(normalize(math.atan2(fy - y, fx - x) - yaw))

                    turn = max(abs(err), far_err)
                    v = max(V_MIN, V_MAX * max(0.0, 1.0 - turn / (math.pi / 4)))
                    if dist_goal < 15:
                        v = min(v, V_MIN + 1.0)

                    w = max(-W_MAX, min(W_MAX, K_W * err))

                    if abs(err) > math.pi / 2:
                        v = V_TURN

                    if abs(err) > math.pi / 3:
                        dist_t = math.hypot(tx - x, ty - y)
                        v = min(v, max(0.4, abs(w) * dist_t / 3.0))

                    HAL.setV(v)
                    HAL.setW(w)
    except Exception:   # Si falla algo se para el coche y se muestra el error
        traceback.print_exc()
        HAL.setV(0)
        HAL.setW(0)

    Frequency.tick(50)
