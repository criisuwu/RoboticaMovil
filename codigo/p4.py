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
SAFE_DIST = 8             # radio de influencia del obstáculo (celdas)
PENALTY = 1.5             # peso del subcampo de penalización (se aplica al cuadrado)
PEN_WEIGHT = 0.3          # peso del subcampo de penalización en la suma
INFLATE = 1               # celdas prohibidas alrededor de los obstáculos (radio del coche)
LOOKAHEAD = 3.5           # distancia del punto al que apunta el coche (metros)
V_MAX, V_MIN = 8.0, 1.5   # velocidad máxima y mínima (m/s)
V_TURN = 1.5              # velocidad al girar cuando el objetivo está muy desviado (m/s)
W_MAX = 2.0               # límite de velocidad angular (rad/s)
K_W = 1.8                 # ganancia del giro
GOAL_RADIUS = 2.5         # distancia para dar el destino por alcanzado (metros)
STUCK_LIMIT = 20          # iteraciones sin descenso antes de replanificar

# Vecinos en 8 direcciones: (fila, columna, coste del paso)
NEIGHBORS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]

# ---------------- Funciones ----------------
# Devuelve la posición y la orientación del coche (x, y, yaw)
def get_pose():
    p = HAL.getPose3d()
    # Según la versión el ángulo se llama yaw o theta
    yaw = getattr(p, 'yaw', None)
    if yaw is None:
        yaw = getattr(p, 'theta', 0.0)
    return float(p.x), float(p.y), float(yaw)


# Lleva un ángulo al rango (-pi, pi)
def normalize(a):
    return math.atan2(math.sin(a), math.cos(a))


# ---------------- Mapa ----------------
# Carga el mapa y lo pasa a escala de grises si viene en color
grid_map = np.array(WebGUI.getMap(MAP_URL))
if grid_map.ndim == 3:
    grid_map = grid_map.mean(axis=2)
H, W = grid_map.shape
gm = grid_map.astype(np.float64)


# Pasa de coordenadas del mundo a celda (fila, columna). swap indica si worldToGrid devuelve el orden contrario
def raw_cell(wx, wy, swap):
    g = WebGUI.worldToGrid([wx, wy])
    gx, gy = int(round(g[0])), int(round(g[1]))
    return (gx, gy) if swap else (gy, gx)


# Comprueba que la celda está dentro del mapa
def in_map(r, c):
    return 0 <= r < H and 0 <= c < W


# Puntúa cada orden posible de worldToGrid: si el coche está sobre la carretera, delante y detrás de él también debería haber carretera
def score_orientation(swap, x, y, yaw):
    r, c = raw_cell(x, y, swap)
    if not in_map(r, c):
        return -1, None
    # Valor de la carretera = mediana de los píxeles alrededor del coche
    rv = float(np.median(gm[max(0, r - 2): r + 3, max(0, c - 2): c + 3]))
    s = 0
    # Se miran varios puntos en la dirección del coche y se cuentan los que parecen carretera
    for d in (-20, -12, -6, 6, 12, 20):
        rr, cc = raw_cell(x + d * math.cos(yaw), y + d * math.sin(yaw), swap)
        if in_map(rr, cc) and abs(gm[rr, cc] - rv) <= 60:
            s += 1
    return s, rv


# Se prueban los dos órdenes y se elige el que mejor encaja
x0, y0, yaw0 = get_pose()
s_false, rv_false = score_orientation(False, x0, y0, yaw0)
s_true, rv_true = score_orientation(True, x0, y0, yaw0)

SWAP_XY = s_true > s_false
road_value = rv_true if SWAP_XY else rv_false
if road_value is None:
    # Si la pose inicial cae fuera del mapa se asume carretera blanca
    road_value = 255.0

# Es obstáculo cualquier celda cuyo valor se aleje del de la carretera
obstacle_np = np.abs(gm - road_value) > 60


# Pasa de coordenadas del mundo a celda, sin salirse del mapa
def to_cell(world_xy):
    r, c = raw_cell(world_xy[0], world_xy[1], SWAP_XY)
    return min(max(r, 0), H - 1), min(max(c, 0), W - 1)


# Pasa de celda a coordenadas del mundo
def to_world(cell):
    r, c = cell
    arg = [int(r), int(c)] if SWAP_XY else [int(c), int(r)]
    w = WebGUI.gridToWorld(arg)
    return float(w[0]), float(w[1])


# Muestra el campo en la GUI (si falla se ignora)
def safe_show_cost(cost):
    try:
        arr = np.array(cost, dtype=np.float64)
        finite = np.isfinite(arr)
        img = np.full(arr.shape, 255, dtype=np.uint8)
        if finite.any():
            # Se normaliza a 0-254 y los obstáculos se pintan en blanco
            img[finite] = (arr[finite] / arr[finite].max() * 254).astype(np.uint8)
        img[obstacle_np] = 255
        WebGUI.showNumpy(img)
    except Exception:
        pass


# Calcula la distancia de cada celda al obstáculo más cercano (hasta SAFE_DIST)
def obstacle_distance():
    big = 10 ** 6
    dist = [[big] * W for _ in range(H)]
    pq = []
    # Todas las celdas de obstáculo son el origen (distancia 0)
    for r, c in np.argwhere(obstacle_np):
        dist[r][c] = 0.0
        pq.append((0.0, int(r), int(c)))
    heapq.heapify(pq)
    # Dijkstra desde todos los obstáculos a la vez
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
# Celdas prohibidas: los obstáculos engordados con el radio del coche
blocked_np = dist_obs_np <= INFLATE
blocked = blocked_np.tolist()

# ---------------- Penalización por cercanía a obstáculos ----------------
# No depende del destino, así que se calcula una sola vez.
field_pen = np.maximum(0.0, SAFE_DIST - dist_obs_np) ** 2 * PENALTY * PEN_WEIGHT


# Si la celda está bloqueada busca la libre más cercana, probando en anillos cada vez más grandes
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


# ---------------- SUBCAMPO 1: propagación a destino ----------------
# Dijkstra desde el destino solo con el coste de los pasos (sin penalización)
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
            # Se descartan las celdas fuera del mapa o bloqueadas
            if not (0 <= nr < H and 0 <= nc < W) or blocked[nr][nc]:
                continue
            # En diagonal no se corta la esquina de un obstáculo
            if dr != 0 and dc != 0 and (blocked[r + dr][c] or blocked[r][c + dc]):
                continue
            new = cur + step
            if new < cost[nr][nc]:
                cost[nr][nc] = new
                heapq.heappush(pq, (new, (nr, nc)))
    return np.array(cost, dtype=np.float64)


field = None        # campo total actual (None = sin destino)
field_prop = None   # subcampo de propagación (de reserva si el campo total no tiene descenso)


# ---------------- CAMPO TOTAL = propagación + penalización ----------------
def build_field(goal_cell):
    global field_prop
    field_prop = propagation(goal_cell)
    return field_prop + field_pen   # inf + número = inf (celdas inalcanzables)


# Si la celda no tiene valor finito en el campo (bloqueada), busca la celda finita MÁS CERCANA al coche
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


# Consulta al campo: de las 8 vecinas, la de menor valor (descenso del gradiente discreto)
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
        # En diagonal no se corta la esquina de un obstáculo
        if dr != 0 and dc != 0 and (blocked[r + dr][c] or blocked[r][c + dc]):
            continue
        if v < best_v:
            best, best_v = (nr, nc), v
    return best


# Sigue el descenso del campo desde la posición del coche hasta 2*LOOKAHEAD metros.
# Devuelve los puntos (mundo) que va encontrando. No hay ruta guardada: se consulta el campo cada vez.
def descent_points(x, y, f):
    cell = entry_cell(to_cell((x, y)), f)
    if cell is None:
        return []
    pts = []
    for _ in range(150):
        nxt = next_cell(cell, f)
        if nxt == cell:        # mínimo del campo (destino)
            break
        cell = nxt
        wx, wy = to_world(cell)
        pts.append((wx, wy))
        if math.hypot(wx - x, wy - y) >= 2 * LOOKAHEAD:
            break
    return pts


goal_world = None    # destino actual en coordenadas del mundo


# Calcula el campo para el destino indicado
def plan(target_world):
    global field, goal_world
    x, y, _ = get_pose()

    # Si ya estamos en el destino no hay nada que planificar
    if math.hypot(target_world[0] - x, target_world[1] - y) < GOAL_RADIUS:
        field = None
        goal_world = None
        return

    goal = nearest_free(to_cell(target_world))
    new_field = build_field(goal)
    safe_show_cost(new_field)

    field = new_field
    # Si desde la posición del coche no se llega a ninguna celda con campo finito, no hay camino
    if entry_cell(to_cell((x, y)), field) is None:
        print("No hay camino hasta el destino")
        field = None
        goal_world = None
        return
    goal_world = target_world


last_target = None   # último destino recibido
stuck_count = 0      # iteraciones seguidas sin dirección de descenso

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
                last_target = None   # permite volver a elegir el mismo destino
            else:
                # Se pregunta al campo hacia dónde bajar
                pts = descent_points(x, y, field)
                # Mínimo local del campo total: se usa la propagación sola para salir de él
                if not pts:
                    pts = descent_points(x, y, field_prop)
                if not pts:
                    # Sin dirección de descenso: se para y, si persiste, se replanifica
                    HAL.setV(0)
                    HAL.setW(0)
                    stuck_count += 1
                    if stuck_count >= STUCK_LIMIT:
                        stuck_count = 0
                        plan(goal_world)
                else:
                    stuck_count = 0

                    # Punto objetivo a LOOKAHEAD metros siguiendo el descenso
                    tx, ty = pts[-1]
                    for px, py in pts:
                        if math.hypot(px - x, py - y) >= LOOKAHEAD:
                            tx, ty = px, py
                            break

                    # Error entre la orientación del coche y el punto objetivo
                    err = normalize(math.atan2(ty - y, tx - x) - yaw)

                    # Mira más adelante en el descenso para frenar antes de la curva
                    fx, fy = pts[-1]
                    far_err = abs(normalize(math.atan2(fy - y, fx - x) - yaw))

                    # La velocidad baja con el mayor de los dos errores (cercano y lejano)
                    turn = max(abs(err), far_err)
                    v = max(V_MIN, V_MAX * max(0.0, 1.0 - turn / (math.pi / 4)))
                    # Cerca del destino se limita la velocidad para llegar suave
                    if dist_goal < 15:
                        v = min(v, V_MIN + 1.0)

                    # Si el objetivo queda muy desviado se casi para y se gira
                    if abs(err) > math.pi / 2:
                        v = V_TURN

                    HAL.setV(v)
                    HAL.setW(max(-W_MAX, min(W_MAX, K_W * err)))
    except Exception:   # Si falla algo se para el coche y se muestra el error
        traceback.print_exc()
        HAL.setV(0)
        HAL.setW(0)

    Frequency.tick(50)
