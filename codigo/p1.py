import random
import time

import HAL
import WebGUI
import Frequency

# ---------------- Parámetros (ajústalos experimentalmente) ----------------
OBST_DIST = 0.35        # m: distancia frontal hasta considerrar el choque con el objeto
FRONT_RAYS = range(75, 106)  # rayos centrales del láser (90 = frente)

V0, V_MAX, V_GROWTH = 0.1, 1.0, 0.08   # espiral: v inicial, máxima, incremento (m/s por s)
W_SPIRAL = 1.5                          # rad/s constante en la espiral
SPIRAL_MAX_T = (V_MAX - V0) / V_GROWTH + 5.0  # cuando r deja de crecer, salimos

BACK_V, BACK_T = -0.2, 0.6
W_TURN = 3.0
TURN_MIN, TURN_MAX = 0.5, 2.5           # duración aleatoria del giro (s)
DASH_V, DASH_T = 0.6, 8.0

# ---------------- Estados ----------------
SPIRAL, BACK, TURN, DASH = "SPIRAL", "BACK", "TURN", "DASH"

state = SPIRAL
t_enter = time.time()
turn_time = 1.0
turn_dir = 1


def elapsed():
    return time.time() - t_enter


def obstacle():
    values = HAL.getLaserData().values
    if len(values) == 0:
        return False
    front = [values[i] for i in FRONT_RAYS if values[i] > 0]
    return len(front) > 0 and min(front) < OBST_DIST


# ---------------- Acciones por estado ----------------
def act_spiral():
    v = min(V0 + V_GROWTH * elapsed(), V_MAX)
    HAL.setV(v)
    HAL.setW(W_SPIRAL)


def act_back():
    HAL.setV(BACK_V)
    HAL.setW(0)


def act_turn():
    HAL.setV(0)
    HAL.setW(turn_dir * W_TURN)


def act_dash():
    HAL.setV(DASH_V)
    HAL.setW(0)


ACTIONS = {
    SPIRAL: act_spiral,
    BACK: act_back,
    TURN: act_turn,
    DASH: act_dash,
}

# ---------------- Tabla de transiciones: estado -> [(evento, siguiente)] ----------------
TRANSITIONS = {
    SPIRAL: [(obstacle, BACK),
             (lambda: elapsed() > SPIRAL_MAX_T, TURN)],
    BACK:   [(lambda: elapsed() > BACK_T, TURN)],
    TURN:   [(lambda: elapsed() > turn_time, DASH)],
    DASH:   [(obstacle, BACK),
             (lambda: elapsed() > DASH_T, SPIRAL)],
}


def on_enter(new_state):
    """Se ejecuta una sola vez al entrar en un estado."""
    global state, t_enter, turn_time, turn_dir
    state = new_state
    t_enter = time.time()
    if new_state == TURN:
        turn_time = random.uniform(TURN_MIN, TURN_MAX)
        turn_dir = random.choice([-1, 1])


# ---------------- Bucle principal ----------------
while True:
    ACTIONS[state]()

    for event, next_state in TRANSITIONS[state]:
        if event():
            on_enter(next_state)
            break

    Frequency.tick(20)