# Asignatura: Robotica Movil                        #
# Navegación de un robot aspiradora de gama baja    #
# Creador: Cristina Homobono Fernández              #
# Fecha: 19/11/2026                                 #

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
DEBUG_MOVE_TEST = True    # avanza 1 s al arrancar para comprobar que HAL.setV funciona
SAFE_DIST = 8             # celdas: radio de influencia del obstáculo
PENALTY = 1.5             # peso extra (se aplica al cuadrado de la distancia que falta)
INFLATE = 1               # celdas "prohibidas" alrededor de obstáculos (radio del coche)
LOOKAHEAD = 4.5           # metros (antes 8: cortaba las esquinas)
V_MAX, V_MIN = 10.0, 1.5   # m/s
K_W = 1.8                 # ganancia angular
GOAL_RADIUS = 2.5         # m

NEIGHBORS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]


def log(*args):
    print("[GN]", *args)


def get_pose():
    p = HAL.getPose3d()
    yaw = getattr(p, 'yaw', None)
    if yaw is None:
        yaw = getattr(p, 'theta', 0.0)
    return float(p.x), float(p.y), float(yaw)


def normalize(a):
    return math.atan2(math.sin(a), math.cos(a))


# ---------------------------------------------- 0) test de actuación del coche
log("Script arrancado")
if DEBUG_MOVE_TEST:
    log("Test de movimiento: avanzando ~1 s. Si el coche NO se mueve, el problema "
        "no es el algoritmo (revisa que estés en modo HAL y que el script se ejecuta).")
    HAL.setV(2.0)
    HAL.setW(0.0)
    for _ in range(50):
        Frequency.tick(50)
    HAL.setV(0.0)
    HAL.setW(0.0)

# ------------------------------------------------------------------- mapa
grid_map = np.array(WebGUI.getMap(MAP_URL))
if grid_map.ndim == 3:
    grid_map = grid_map.mean(axis=2)
H, W = grid_map.shape
gm = grid_map.astype(np.float64)
log("Mapa:", grid_map.shape, "valores:", np.unique(grid_map)[:6])


def raw_cell(wx, wy, swap):
    g = WebGUI.worldToGrid([wx, wy])
    gx, gy = int(round(g[0])), int(round(g[1]))
    return (gx, gy) if swap else (gy, gx)


def in_map(r, c):
    return 0 <= r < H and 0 <= c < W


def score_orientation(swap, x, y, yaw):
    """
    Puntúa una interpretación de worldToGrid: el coche está sobre la carretera,
    así que delante y detrás de él debería seguir habiendo carretera.
    """
    r, c = raw_cell(x, y, swap)
    if not in_map(r, c):
        return -1, None
    rv = float(np.median(gm[max(0, r - 2): r + 3, max(0, c - 2): c + 3]))
    s = 0
    for d in (-20, -12, -6, 6, 12, 20):
        rr, cc = raw_cell(x + d * math.cos(yaw), y + d * math.sin(yaw), swap)
        if in_map(rr, cc) and abs(gm[rr, cc] - rv) <= 60:
            s += 1
    return s, rv


x0, y0, yaw0 = get_pose()
s_false, rv_false = score_orientation(False, x0, y0, yaw0)
s_true, rv_true = score_orientation(True, x0, y0, yaw0)
log("Pose inicial: x=%.1f y=%.1f yaw=%.2f" % (x0, y0, yaw0))
log("Puntuación orden (col,fila): %d | orden (fila,col): %d" % (s_false, s_true))

SWAP_XY = s_true > s_false
road_value = rv_true if SWAP_XY else rv_false
if road_value is None:
    log("ATENCIÓN: la pose inicial cae fuera del mapa; revisa worldToGrid")
    road_value = 255.0
log("Usando SWAP_XY =", SWAP_XY, "| valor de carretera =", road_value)

obstacle_np = np.abs(gm - road_value) > 60
obstacle = obstacle_np.tolist()
log("%.1f %% del mapa es carretera" % (100.0 * (1 - obstacle_np.mean())))


def to_cell(world_xy):
    r, c = raw_cell(world_xy[0], world_xy[1], SWAP_XY)
    return min(max(r, 0), H - 1), min(max(c, 0), W - 1)


def to_world(cell):
    r, c = cell
    arg = [r, c] if SWAP_XY else [c, r]
    w = WebGUI.gridToWorld(arg)
    return float(w[0]), float(w[1])


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
        log("showNumpy ha fallado (se ignora):")
        traceback.print_exc()


def safe_show_path(cells):
    try:
        pts = [[r, c] if SWAP_XY else [c, r] for r, c in cells]
        WebGUI.showPath(pts)
    except Exception:
        log("showPath ha fallado (se ignora):")
        traceback.print_exc()


def obstacle_distance():
    """Distancia aproximada (octil) a la celda de obstáculo más cercana."""
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
blocked = (np.array(dist_obs) <= INFLATE).tolist()   # obstáculo "engordado"


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


def wavefront(target):
    inf = float('inf')
    cost = [[inf] * W for _ in range(H)]
    parent = {}
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
            extra = max(0.0, SAFE_DIST - dist_obs[nr][nc]) ** 2 * PENALTY
            new = cur + step + extra
            if new < cost[nr][nc]:
                cost[nr][nc] = new
                parent[(nr, nc)] = (r, c)
                heapq.heappush(pq, (new, (nr, nc)))
    return cost, parent


world_path = []
idx = 0


def plan(target_world):
    global world_path, idx
    x, y, _ = get_pose()
    start = nearest_free(to_cell((x, y)))
    goal = nearest_free(to_cell(target_world))
    log("Planificando: start", start, "-> goal", goal)

    cost, parent = wavefront(goal)
    safe_show_cost(cost)

    if start not in parent and start != goal:
        log("NO hay camino entre start y goal (¿destino fuera de la carretera?)")
        world_path, idx = [], 0
        return

    cells = [start]
    while cells[-1] in parent:
        cells.append(parent[cells[-1]])

    safe_show_path(cells)
    world_path = [to_world(c) for c in cells]
    idx = 0
    log("Camino listo: %d celdas" % len(cells))


# ------------------------------------------------------------ bucle principal
log("Listo. Haz clic en el mapa para elegir destino.")
last_target = None
tick = 0

while True:
    tick += 1
    try:
        target = WebGUI.getTargetPose()
        target = (float(target[0]), float(target[1])) if target is not None else None

        if tick % 250 == 0:
            log("latido: target =", target, "| ruta activa =", bool(world_path))

        if target is not None and target != last_target:
            last_target = target
            log("Nuevo destino:", target)
            HAL.setV(0)
            HAL.setW(0)
            plan(target)

        if world_path:
            x, y, yaw = get_pose()

            window = world_path[idx: idx + 40]
            idx += int(np.argmin([math.hypot(px - x, py - y) for px, py in window]))

            gx, gy = world_path[-1]
            dist_goal = math.hypot(gx - x, gy - y)
            if dist_goal < GOAL_RADIUS:
                HAL.setV(0)
                HAL.setW(0)
                world_path = []
                log("Destino alcanzado")
            else:
                # punto objetivo a LOOKAHEAD metros
                tx, ty = gx, gy
                for px, py in world_path[idx:]:
                    if math.hypot(px - x, py - y) >= LOOKAHEAD:
                        tx, ty = px, py
                        break

                err = normalize(math.atan2(ty - y, tx - x) - yaw)

                # curvatura de la ruta más adelante: frena antes de la curva
                fx, fy = gx, gy
                for px, py in world_path[idx:]:
                    if math.hypot(px - x, py - y) >= 2 * LOOKAHEAD:
                        fx, fy = px, py
                        break
                far_err = abs(normalize(math.atan2(fy - y, fx - x) - yaw))

                turn = max(abs(err), far_err)
                v = max(V_MIN, V_MAX * max(0.0, 1.0 - turn / (math.pi / 3)))
                if dist_goal < 15:
                    v = min(v, V_MIN + 1.0)

                HAL.setV(v)
                HAL.setW(K_W * err)
    except Exception:
        log("Error en el bucle:")
        traceback.print_exc()
        HAL.setV(0)
        HAL.setW(0)

    Frequency.tick(50)
