# cinematica.py - Persona 1
# Control cinematico diferencial, odometria y evasion reactiva del e-puck

import math

# ---------------------------------------------------------------------------
# Constantes fisicas del e-puck
# ---------------------------------------------------------------------------
WHEEL_RADIUS   = 0.0205   # metros (radio de cada rueda)
WHEEL_DISTANCE = 0.052    # metros (distancia entre ruedas, eje L)
MAX_SPEED      = 6.28     # rad/s (velocidad maxima de los motores)

# Umbral de proximidad para considerar obstaculo (ajustar segun escenario)
OBSTACLE_THRESHOLD = 80.0

# ---------------------------------------------------------------------------
# Variables internas de odometria (estado del robot)
# ---------------------------------------------------------------------------
_x     = 0.0   # posicion x en metros
_y     = 0.0   # posicion y en metros
_theta = 0.0   # orientacion en radianes

_last_enc_left  = 0.0
_last_enc_right = 0.0
_initialized    = False

# Referencias a dispositivos Webots (se asignan en init)
_robot       = None
_left_motor  = None
_right_motor = None
_enc_left    = None
_enc_right   = None
_sensors     = []
_timestep    = 32


# ---------------------------------------------------------------------------
# Inicializacion
# ---------------------------------------------------------------------------
def init(robot):
    """
    Llama esta funcion UNA vez al inicio del main.py, pasando el objeto Robot.
    Inicializa motores, encoders y sensores de proximidad.
    """
    global _robot, _left_motor, _right_motor
    global _enc_left, _enc_right, _sensors, _timestep
    global _last_enc_left, _last_enc_right, _initialized

    _robot    = robot
    _timestep = int(robot.getBasicTimeStep())

    # Motores
    _left_motor  = robot.getDevice('left wheel motor')
    _right_motor = robot.getDevice('right wheel motor')
    _left_motor.setPosition(float('inf'))   # modo velocidad
    _right_motor.setPosition(float('inf'))
    _left_motor.setVelocity(0.0)
    _right_motor.setVelocity(0.0)

    # Encoders (position sensors)
    _enc_left  = robot.getDevice('left wheel sensor')
    _enc_right = robot.getDevice('right wheel sensor')
    _enc_left.enable(_timestep)
    _enc_right.enable(_timestep)

    # Sensores de proximidad ps0..ps7
    _sensors = []
    for i in range(8):
        ps = robot.getDevice(f'ps{i}')
        ps.enable(_timestep)
        _sensors.append(ps)

    # Dar un paso para que los encoders tengan valor inicial
    robot.step(_timestep)
    _last_enc_left  = _enc_left.getValue()
    _last_enc_right = _enc_right.getValue()
    _initialized    = True

    print(f"[cinematica] Inicializado. timestep={_timestep} ms")


# ---------------------------------------------------------------------------
# Control de movimiento
# ---------------------------------------------------------------------------
def set_velocity(v, omega):
    """
    Envia velocidad lineal v (m/s) y angular omega (rad/s) al robot.
    Internamente convierte a velocidades de rueda usando cinematica diferencial.

    Ecuaciones (del PDF del proyecto):
        vr = v + (omega * L / 2)
        vl = v - (omega * L / 2)
    """
    vr = v + (omega * WHEEL_DISTANCE / 2.0)
    vl = v - (omega * WHEEL_DISTANCE / 2.0)

    # Limitar a velocidad maxima
    vr = max(-MAX_SPEED, min(MAX_SPEED, vr / WHEEL_RADIUS))
    vl = max(-MAX_SPEED, min(MAX_SPEED, vl / WHEEL_RADIUS))

    _left_motor.setVelocity(vl)
    _right_motor.setVelocity(vr)


def stop():
    """Detiene el robot completamente."""
    _left_motor.setVelocity(0.0)
    _right_motor.setVelocity(0.0)


# ---------------------------------------------------------------------------
# Odometria
# ---------------------------------------------------------------------------
def update_odometry():
    """
    Debe llamarse en CADA paso del loop principal (despues de robot.step).
    Actualiza la estimacion de posicion (x, y, theta) con los encoders.

    Ecuaciones del modelo de movimiento (del PDF):
        delta_sr = r * delta_theta_r
        delta_sl = r * delta_theta_l
        delta_s   = (delta_sr + delta_sl) / 2
        delta_phi = (delta_sr - delta_sl) / L
        x_k  = x_{k-1} + delta_s * cos(theta_{k-1} + delta_phi/2)
        y_k  = y_{k-1} + delta_s * sin(theta_{k-1} + delta_phi/2)
        phi_k = phi_{k-1} + delta_phi
    """
    global _x, _y, _theta, _last_enc_left, _last_enc_right

    enc_l = _enc_left.getValue()
    enc_r = _enc_right.getValue()

    delta_enc_l = enc_l - _last_enc_left
    delta_enc_r = enc_r - _last_enc_right

    _last_enc_left  = enc_l
    _last_enc_right = enc_r

    # Distancia recorrida por cada rueda
    delta_sl = WHEEL_RADIUS * delta_enc_l
    delta_sr = WHEEL_RADIUS * delta_enc_r

    # Desplazamiento y cambio de orientacion
    delta_s   = (delta_sr + delta_sl) / 2.0
    delta_phi = (delta_sr - delta_sl) / WHEEL_DISTANCE

    # Actualizar pose
    _x     += delta_s * math.cos(_theta + delta_phi / 2.0)
    _y     += delta_s * math.sin(_theta + delta_phi / 2.0)
    _theta += delta_phi

    # Normalizar theta entre -pi y pi
    _theta = math.atan2(math.sin(_theta), math.cos(_theta))


def get_pose():
    """
    Retorna la pose estimada actual del robot.
    Retorno: (x, y, theta)  en metros y radianes.
    """
    return (_x, _y, _theta)


def reset_pose(x=0.0, y=0.0, theta=0.0):
    """Resetea la pose a un valor conocido (util al inicio de cada prueba)."""
    global _x, _y, _theta
    _x, _y, _theta = x, y, theta


# ---------------------------------------------------------------------------
# Sensores de proximidad
# ---------------------------------------------------------------------------
def get_sensor_values():
    """
    Retorna lista con los 8 valores crudos de proximidad [ps0..ps7].
    Valores altos = obstaculo cerca.
    """
    return [ps.getValue() for ps in _sensors]


def is_obstacle_ahead():
    """
    Retorna True si hay un obstaculo peligroso al frente del robot.
    Usa ps0, ps1 (derecha-frente) y ps6, ps7 (izquierda-frente).
    """
    vals = get_sensor_values()
    front = [vals[0], vals[1], vals[6], vals[7]]
    return any(v > OBSTACLE_THRESHOLD for v in front)


def is_obstacle_left():
    """Retorna True si hay obstaculo a la izquierda (ps5, ps6)."""
    vals = get_sensor_values()
    return any(vals[i] > OBSTACLE_THRESHOLD for i in [5, 6])


def is_obstacle_right():
    """Retorna True si hay obstaculo a la derecha (ps1, ps2)."""
    vals = get_sensor_values()
    return any(vals[i] > OBSTACLE_THRESHOLD for i in [1, 2])


# ---------------------------------------------------------------------------
# Navegacion reactiva local
# ---------------------------------------------------------------------------
def reactive_avoid():
    """
    Ejecuta una maniobra de evasion simple basada en sensores.
    Llamar solo cuando is_obstacle_ahead() sea True.

    Logica:
      - Si hay mas obstaculo a la derecha -> gira izquierda
      - Si hay mas obstaculo a la izquierda -> gira derecha
      - Si es simetrico -> gira izquierda por defecto
    """
    vals = get_sensor_values()

    right_val = vals[0] + vals[1] + vals[2]
    left_val  = vals[5] + vals[6] + vals[7]

    if right_val >= left_val:
        # Obstaculo mas a la derecha: gira izquierda
        set_velocity(0.0, 2.0)
    else:
        # Obstaculo mas a la izquierda: gira derecha
        set_velocity(0.0, -2.0)