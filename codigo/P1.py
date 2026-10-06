# Asignatura: Robotica Movil                        #
# Navegación de un robot aspiradora de gama baja    #
# Creador: Cristina Homobono Fernández              #
# Fecha: 06/10/2026                                 #

# ---------------- Imports ----------------
import random
import time

import HAL
import WebGUI
import Frequency

# ---------------- Parámetros ----------------
OBST_DIST = 0.35        # distancia hasta considerar choque
FRONT_RAYS = range(75, 106)  # rayos laser (El modulo tiene rango de 180 pero yo los he definido en el rango de 75º y de 106º para acotar la vista)

V0, V_MAX, V_GROWTH = 0.1, 1.0, 0.08   # la creacion de la espiral donde definimos V0 como velocidad inicial, v_max como velocidad maxima, v_growth como la velocidad del crecimiento de la espiral
W_SPIRAL = 1.5                          # velocidad angular de la espiral
SPIRAL_MAX_T = (V_MAX - V0) / V_GROWTH + 5.0  # el radio maximo de crecimiento de la espiral

BACK_V, BACK_T = -0.2, 0.6  #velocidad para ir hacia atras y tiempo que esta yendo hacia atras
W_TURN = 2.5    #radio de giro sobre si mismo
TURN_MIN, TURN_MAX = 0.5, 2.5           # duración aleatoria del giro entre 0.5 segundos y 2.5 segundos
DASH_V, DASH_T = 0.6, 8.0   #Velocidad para ir hacia delante, y tiempo que se esta moviendo hacia delante

# ---------------- Estados ----------------
SPIRAL, BACK, TURN, DASH = "SPIRAL", "BACK", "TURN", "DASH"

state = SPIRAL
t_enter = time.time()
turn_time = 1.0
turn_dir = 1


# ---------------- Funciones auxiliares ----------------
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

# ---------------- Tabla de transiciones ----------------

def spiral_done():
    return elapsed() > SPIRAL_MAX_T

def back_done():
    return elapsed() > BACK_T

def turn_done():
    return elapsed() > turn_time

def dash_done():
    return elapsed() > DASH_T


TRANSITIONS = {
    SPIRAL: [(obstacle, BACK),
             (spiral_done, TURN)],
    BACK:   [(back_done, TURN)],
    TURN:   [(turn_done, DASH)],
    DASH:   [(obstacle, BACK),
             (dash_done, SPIRAL)],
}


def on_enter(new_state):
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