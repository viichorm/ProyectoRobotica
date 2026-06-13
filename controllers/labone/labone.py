from controller import Robot
import math
import csv
import heapq

# ════════════════════════════════════════════════════════════════
#  FUNCIONES UTILITARIAS
# ════════════════════════════════════════════════════════════════

def filtro_exponencial(valor_nuevo, valor_anterior, alpha=0.3):
    return alpha * valor_nuevo + (1 - alpha) * valor_anterior

def sensor_a_distancia(valor_sensor, max_sensor=4095.0, max_dist=0.10):
    valor_clamp = max(0.0, min(float(valor_sensor), max_sensor))
    return max_dist * (1.0 - valor_clamp / max_sensor)

def normalizar_angulo(angulo):
    """Normaliza ángulo al rango [-π, π]."""
    while angulo >  math.pi: angulo -= 2 * math.pi
    while angulo < -math.pi: angulo += 2 * math.pi
    return angulo

# ════════════════════════════════════════════════════════════════
#  CONFIGURACIÓN DE ESCENARIOS
# ════════════════════════════════════════════════════════════════
#
#  Cambia ESCENARIO = 'FACIL' o 'COMPLEJO' para seleccionar arena.
#  Ajusta START, GOAL y la grilla según tu mundo de Webots.
#
ESCENARIO = 'FACIL'   # <-- cambiar aquí para el segundo escenario

if ESCENARIO == 'FACIL':
    ARENA_W   = 1.0
    ARENA_H   = 1.0
    CELL_SIZE = 0.05         # 20×20 celdas
    ROWS      = int(ARENA_H / CELL_SIZE)   # 20
    COLS      = int(ARENA_W / CELL_SIZE)   # 20

    START_X, START_Y, START_PHI = -0.45, -0.45, 0.785398
    GOAL_X, GOAL_Y = 0.45, 0.45

    # Conversión: col = int((x + 0.5) / 0.05),  row = int((y + 0.5) / 0.05)
    #
    # NARANJA  centro(-0.15,  0.20)  size(0.30×0.20)
    #   X: -0.30→0.00  cols  4→10   Y: 0.10→0.30  rows 12→16
    # AZUL     centro( 0.15, -0.10)  size(0.30×0.20)
    #   X:  0.00→0.30  cols 10→16   Y:-0.20→0.00  rows  6→10
    # MORADO   centro(-0.15, -0.40)  size(0.30×0.20)
    #   X: -0.30→0.00  cols  4→10   Y:-0.50→-0.30 rows  0→ 4
    # META VERDE (sólido en 0.45,0.45) — marcada para que A* rodee
    #   X:  0.40→0.50  cols 18→20   Y: 0.40→0.50  rows 18→20

    GRID = [[0]*COLS for _ in range(ROWS)]

    for r in range(12, 16):   # Naranja
        for c in range(4, 10): GRID[r][c] = 1
    for r in range(6, 10):    # Azul
        for c in range(10, 16): GRID[r][c] = 1
    for r in range(0, 4):     # Morado
        for c in range(4, 10): GRID[r][c] = 1
    for r in range(18, 20):   # Meta verde (sólido)
        for c in range(18, 20): GRID[r][c] = 1

else:  # COMPLEJO
    ARENA_W   = 1.2
    ARENA_H   = 1.2
    CELL_SIZE = 0.05
    ROWS      = int(ARENA_H / CELL_SIZE)   # 24
    COLS      = int(ARENA_W / CELL_SIZE)   # 24
    START_X, START_Y, START_PHI = -0.50, -0.50, 0.0
    GOAL_X, GOAL_Y = 0.50, 0.50
    GRID = [[0]*COLS for _ in range(ROWS)]
    # --- Ejemplo: obstáculos del escenario complejo ---
    for c in range(4, 10):  GRID[8][c]  = 1
    for c in range(12, 20): GRID[8][c]  = 1
    for r in range(10, 18): GRID[r][6]  = 1
    for r in range(4, 12):  GRID[r][18] = 1
    for c in range(8, 16):  GRID[16][c] = 1

# Margen de seguridad: infla obstáculos 1 celda (radio del e-puck ≈ 3.7 cm)
def inflar_obstaculos(grid, margen=1):
    filas = len(grid)
    cols  = len(grid[0])
    nueva = [fila[:] for fila in grid]
    for r in range(filas):
        for c in range(cols):
            if grid[r][c] == 1:
                for dr in range(-margen, margen+1):
                    for dc in range(-margen, margen+1):
                        nr, nc = r+dr, c+dc
                        if 0 <= nr < filas and 0 <= nc < cols:
                            nueva[nr][nc] = 1
    return nueva

MARGEN_INFLADO = 1 if ESCENARIO == 'FACIL' else 1
GRID = inflar_obstaculos(GRID, margen=MARGEN_INFLADO)

# ════════════════════════════════════════════════════════════════
#  CONVERSIÓN MUNDO ↔ GRILLA
# ════════════════════════════════════════════════════════════════

ORIGEN_X = -ARENA_W / 2.0
ORIGEN_Y = -ARENA_H / 2.0

def world_to_grid(x, y):
    col = int((x - ORIGEN_X) / CELL_SIZE)
    row = int((y - ORIGEN_Y) / CELL_SIZE)
    col = max(0, min(col, COLS - 1))
    row = max(0, min(row, ROWS - 1))
    return row, col

def grid_to_world(row, col):
    x = ORIGEN_X + col * CELL_SIZE + CELL_SIZE / 2.0
    y = ORIGEN_Y + row * CELL_SIZE + CELL_SIZE / 2.0
    return x, y

def celda_libre_mas_cercana(grid, inicio):
    filas = len(grid)
    cols = len(grid[0])
    if grid[inicio[0]][inicio[1]] == 0:
        return inicio

    from collections import deque
    cola = deque([inicio])
    visitadas = {inicio}
    direcciones = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]

    while cola:
        r, c = cola.popleft()
        for dr, dc in direcciones:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < filas and 0 <= nc < cols):
                continue
            vecino = (nr, nc)
            if vecino in visitadas:
                continue
            if grid[nr][nc] == 0:
                return vecino
            visitadas.add(vecino)
            cola.append(vecino)

    return inicio

# ════════════════════════════════════════════════════════════════
#  ALGORITMO A* (8 DIRECCIONES)
# ════════════════════════════════════════════════════════════════

def heuristica(a, b):
    """Distancia euclidiana como heurística admisible."""
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)

def astar(grid, inicio, meta):
    """
    Devuelve lista de celdas (row, col) desde inicio hasta meta,
    o lista vacía si no hay camino.
    """
    filas = len(grid)
    cols  = len(grid[0])
    DIRS  = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
    COSTS = [1.0,   1.0,  1.0,   1.0,  1.414,   1.414,  1.414,  1.414]

    open_set = []
    heapq.heappush(open_set, (0.0, inicio))
    came_from = {}
    g_score   = {inicio: 0.0}

    while open_set:
        _, actual = heapq.heappop(open_set)
        if actual == meta:
            # Reconstruir ruta
            ruta = []
            while actual in came_from:
                ruta.append(actual)
                actual = came_from[actual]
            ruta.append(inicio)
            ruta.reverse()
            return ruta

        for (dr, dc), costo in zip(DIRS, COSTS):
            vecino = (actual[0]+dr, actual[1]+dc)
            if not (0 <= vecino[0] < filas and 0 <= vecino[1] < cols):
                continue
            if grid[vecino[0]][vecino[1]] == 1:
                continue
            if abs(dr) == 1 and abs(dc) == 1:
                if grid[actual[0] + dr][actual[1]] == 1 or grid[actual[0]][actual[1] + dc] == 1:
                    continue
            nuevo_g = g_score[actual] + costo
            if nuevo_g < g_score.get(vecino, float('inf')):
                came_from[vecino] = actual
                g_score[vecino]   = nuevo_g
                f = nuevo_g + heuristica(vecino, meta)
                heapq.heappush(open_set, (f, vecino))
    return []  # sin camino

# ════════════════════════════════════════════════════════════════
#  SUAVIZADO DE RUTA – BRESENHAM (LÍNEA DE VISIÓN LIBRE)
# ════════════════════════════════════════════════════════════════

def bresenham_libre(grid, a, b):
    """True si la línea recta entre a y b no pasa por ningún obstáculo."""
    r0, c0 = a
    r1, c1 = b
    dr = abs(r1-r0); dc = abs(c1-c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    err = dr - dc
    while True:
        if grid[r0][c0] == 1:
            return False
        if r0 == r1 and c0 == c1:
            break
        e2 = 2 * err
        if e2 > -dc: err -= dc; r0 += sr
        if e2 <  dr: err += dr; c0 += sc
    return True

def suavizar_ruta(grid, ruta):
    """Elimina nodos intermedios redundantes usando línea de visión libre."""
    if len(ruta) <= 2:
        return ruta
    suavizada = [ruta[0]]
    i = 0
    while i < len(ruta) - 1:
        j = len(ruta) - 1
        while j > i + 1:
            if bresenham_libre(grid, ruta[i], ruta[j]):
                break
            j -= 1
        suavizada.append(ruta[j])
        i = j
    return suavizada

# ════════════════════════════════════════════════════════════════
#  INICIALIZACIÓN DEL ROBOT
# ════════════════════════════════════════════════════════════════

RobotEpuck       = Robot()
TimestepMuestreo = int(RobotEpuck.getBasicTimeStep())
VelocidadMaxima  = 6.28

MotorIzquierdo = RobotEpuck.getDevice('left wheel motor')
MotorDerecho   = RobotEpuck.getDevice('right wheel motor')
MotorIzquierdo.setPosition(float('inf'))
MotorDerecho.setPosition(float('inf'))
MotorIzquierdo.setVelocity(0.0)
MotorDerecho.setVelocity(0.0)

EncoderIzquierdo = RobotEpuck.getDevice('left wheel sensor')
EncoderIzquierdo.enable(TimestepMuestreo)
EncoderDerecho = RobotEpuck.getDevice('right wheel sensor')
EncoderDerecho.enable(TimestepMuestreo)

SensorFrontalDerecho    = RobotEpuck.getDevice('ps0')
SensorDiagonalDerecho   = RobotEpuck.getDevice('ps1')
SensorLateralDerecho    = RobotEpuck.getDevice('ps2')
SensorTraseroDerecho    = RobotEpuck.getDevice('ps3')
SensorTraseroIzquierdo  = RobotEpuck.getDevice('ps4')
SensorLateralIzquierdo  = RobotEpuck.getDevice('ps5')
SensorDiagonalIzquierdo = RobotEpuck.getDevice('ps6')
SensorFrontalIzquierdo  = RobotEpuck.getDevice('ps7')

for sensor in [
    SensorFrontalDerecho, SensorDiagonalDerecho,
    SensorLateralDerecho, SensorTraseroDerecho,
    SensorTraseroIzquierdo, SensorLateralIzquierdo,
    SensorDiagonalIzquierdo, SensorFrontalIzquierdo
]:
    sensor.enable(TimestepMuestreo)

# ════════════════════════════════════════════════════════════════
#  PARÁMETROS FÍSICOS DEL E-PUCK
# ════════════════════════════════════════════════════════════════

RadioRueda      = 0.0205
DistanciaRuedas = 0.052
Ts              = TimestepMuestreo / 1000.0

# ════════════════════════════════════════════════════════════════
#  PARÁMETROS DE CONTROL
# ════════════════════════════════════════════════════════════════

AlphaFiltro = 0.3
UmbralDiagonal = 600
Histeresis     = 100

UmbralDistDeteccion  = 0.08
UmbralDistPeligro    = 0.05
UmbralDistHisteresis = 0.012

# Seguimiento de waypoints
UMBRAL_WAYPOINT   = 0.04    # metros – distancia para considerar waypoint alcanzado
UMBRAL_META       = 0.05    # metros – distancia para declarar meta alcanzada
KP_ANGULAR        = 2.5     # ganancia proporcional angular
VELOCIDAD_BASE    = VelocidadMaxima * 0.65

# ════════════════════════════════════════════════════════════════
#  PARÁMETROS DEL FILTRO DE KALMAN (Lab 2)
# ════════════════════════════════════════════════════════════════

Q_kalman = 0.0001
R_kalman = 0.0015
d_kalman = 0.10
P_kalman = 1.0

# ════════════════════════════════════════════════════════════════
#  PLANIFICACIÓN DE RUTA A*
# ════════════════════════════════════════════════════════════════

nodo_inicio_bruto = world_to_grid(START_X, START_Y)
nodo_inicio = celda_libre_mas_cercana(GRID, nodo_inicio_bruto)
# La meta real (0.45, 0.45) cae en celda (19,19) que está fuera del área libre.
# Usamos (17,17) — la celda libre más cercana antes del sólido verde — como nodo meta A*.
# El robot recorrerá el último tramo físicamente guiado por UMBRAL_META.
nodo_meta_astar = (17, 17)
nodo_meta       = world_to_grid(GOAL_X, GOAL_Y)   # solo para referencia/CSV

print(f"[A*] Calculando ruta: {nodo_inicio} → {nodo_meta_astar}")
ruta_celdas = astar(GRID, nodo_inicio, nodo_meta_astar)

if not ruta_celdas:
    print("[A*] ERROR: No se encontró ruta. Verifica la grilla y los puntos de inicio/meta.")
    ruta_celdas = [nodo_inicio]
else:
    ruta_celdas = suavizar_ruta(GRID, ruta_celdas)
    print(f"[A*] Ruta suavizada: {len(ruta_celdas)} waypoints")

# ── DEBUG: imprimir grilla con ruta ─────────────────────────────
ruta_set = set(ruta_celdas)
print("=== GRILLA (S=inicio G=meta .=ruta █=obstáculo) ===")
for r in range(ROWS-1, -1, -1):
    fila = ""
    for c in range(COLS):
        if (r, c) == nodo_inicio:       fila += "S"
        elif (r, c) == nodo_meta_astar: fila += "G"
        elif (r, c) in ruta_set:        fila += "."
        elif GRID[r][c] == 1:           fila += "█"
        else:                           fila += " "
    print(f"{r:2d}|{fila}|")
print("    " + "".join(str(c % 10) for c in range(COLS)))

# Convertir celdas a coordenadas del mundo
waypoints = [grid_to_world(r, c) for r, c in ruta_celdas]
waypoints.append((GOAL_X, GOAL_Y))   # asegurar meta exacta al final
IndiceWaypoint = 0

# ════════════════════════════════════════════════════════════════
#  CONFIGURACIÓN CSV
# ════════════════════════════════════════════════════════════════

IntervaloCSV         = 0.1
UltimoTiempoGuardado = -1.0

# CSV 1: Datos crudos de sensores y encoders (Lab 2)
arch_crudos = open('../../data/datos_crudos.csv', 'w', newline='', encoding='utf-8-sig')
esc_crudos  = csv.writer(arch_crudos, delimiter=';')
esc_crudos.writerow([
    'ps0_FD','ps1_DD','ps2_LD','ps3_TD',
    'ps4_TI','ps5_LI','ps6_DI','ps7_FI',
    'Theta_Izq_rad','Theta_Der_rad','DeltaS_Izq_m','DeltaS_Der_m',
    'AvanceLineal_m','GiroAngular_rad','VelLineal_ms','VelAngular_rads','timestamp_s'
])

# CSV 2: Filtros EMA y Kalman (Lab 2)
arch_kalman = open('../../data/datos_kalman.csv', 'w', newline='', encoding='utf-8-sig')
esc_kalman  = csv.writer(arch_kalman, delimiter=';')
esc_kalman.writerow([
    'distFD_cruda_m','distFI_cruda_m','distMin_cruda_m',
    'distFD_EMA_m','distFI_EMA_m','distMin_EMA_m',
    'pred_kalman_m','dist_kalman_m','K_kalman','P_kalman',
    'AvanceLineal_m','timestamp_s'
])

# CSV 3: Pose global + estado del árbitro (Proyecto Final)
arch_pose = open('../../data/datos_pose.csv', 'w', newline='', encoding='utf-8-sig')
esc_pose  = csv.writer(arch_pose, delimiter=';')
esc_pose.writerow([
    'PoseX_m','PoseY_m','PosePhi_rad',
    'IndiceWaypoint','WP_X_m','WP_Y_m',
    'DistWaypoint_m','ErrorAngular_rad',
    'EstadoArbitro','timestamp_s'
])

# CSV 4: Ruta planificada por A* (una sola vez)
arch_ruta = open('../../data/ruta_planificada.csv', 'w', newline='', encoding='utf-8-sig')
esc_ruta  = csv.writer(arch_ruta, delimiter=';')
esc_ruta.writerow(['IndiceWP','Row','Col','X_m','Y_m'])
for i, ((r, c), (wx, wy)) in enumerate(zip(ruta_celdas, waypoints)):
    esc_ruta.writerow([i, r, c, round(wx, 4), round(wy, 4)])
arch_ruta.close()

# ════════════════════════════════════════════════════════════════
#  ESTADO INICIAL
# ════════════════════════════════════════════════════════════════

ThetaIzqAnterior = 0.0
ThetaDerAnterior = 0.0
PrimeraLecturaEncoders = True

# Odometría global (Lab 1 + Lab 2 integrados)
PoseX   = START_X
PoseY   = START_Y
PosePhi = START_PHI

# Filtros EMA
FD_filt = None
FI_filt = None

# Árbitro de decisiones
# Estados: 'waypoint' | 'bypass' | 'meta'
EstadoArbitro          = 'waypoint'
DireccionGiroBloqueada = None
SesgoDerechaActivo     = False
PasosBypass            = 0
MinPasosBypass         = 6
PasosLibreParaSalir    = 4

VelocidadIzquierda = 0.0
VelocidadDerecha   = 0.0

print(f"[INIT] Pose inicial: ({PoseX:.2f}, {PoseY:.2f}, {math.degrees(PosePhi):.1f}°)")
print(f"[INIT] Meta: ({GOAL_X}, {GOAL_Y})")
print(f"[INIT] Waypoints totales: {len(waypoints)}")

# ════════════════════════════════════════════════════════════════
#  BUCLE PRINCIPAL
# ════════════════════════════════════════════════════════════════

while RobotEpuck.step(TimestepMuestreo) != -1:

    TiempoActual = RobotEpuck.getTime()

    # ── 1. ENCODERS – ODOMETRÍA (Lab 1 cinemática + Lab 2 encoders) ──
    ThetaIzqActual = EncoderIzquierdo.getValue()
    ThetaDerActual = EncoderDerecho.getValue()

    if PrimeraLecturaEncoders:
        ThetaIzqAnterior = ThetaIzqActual
        ThetaDerAnterior = ThetaDerActual
        DeltaThetaIzq = 0.0
        DeltaThetaDer = 0.0
        PrimeraLecturaEncoders = False
    else:
        DeltaThetaIzq = ThetaIzqActual - ThetaIzqAnterior
        DeltaThetaDer = ThetaDerActual - ThetaDerAnterior

    DeltaSIzq = RadioRueda * DeltaThetaIzq
    DeltaSDer = RadioRueda * DeltaThetaDer

    AvanceLineal = (DeltaSIzq + DeltaSDer) / 2.0
    GiroAngular  = (DeltaSDer - DeltaSIzq) / DistanciaRuedas

    VelocidadLineal  = AvanceLineal / Ts
    VelocidadAngular = GiroAngular  / Ts

    ThetaIzqAnterior = ThetaIzqActual
    ThetaDerAnterior = ThetaDerActual

    # ── 2. POSE GLOBAL (Ecuaciones pág. 4 de la pauta) ───────────────
    #   xk = x_{k-1} + Δs · cos(φ_{k-1} + Δφ/2)
    #   yk = y_{k-1} + Δs · sin(φ_{k-1} + Δφ/2)
    #   φk = φ_{k-1} + Δφ   → normalizado en [-π, π]
    PoseX   += AvanceLineal * math.cos(PosePhi + GiroAngular / 2.0)
    PoseY   += AvanceLineal * math.sin(PosePhi + GiroAngular / 2.0)
    PosePhi  = normalizar_angulo(PosePhi + GiroAngular)

    # ── 3. SENSORES CRUDOS ───────────────────────────────────────────
    ValFD = SensorFrontalDerecho.getValue()
    ValDD = SensorDiagonalDerecho.getValue()
    ValLD = SensorLateralDerecho.getValue()
    ValTD = SensorTraseroDerecho.getValue()
    ValTI = SensorTraseroIzquierdo.getValue()
    ValLI = SensorLateralIzquierdo.getValue()
    ValDI = SensorDiagonalIzquierdo.getValue()
    ValFI = SensorFrontalIzquierdo.getValue()

    # ── 4. FILTRO EMA (Lab 2) ────────────────────────────────────────
    if FD_filt is None:
        FD_filt = ValFD
        FI_filt = ValFI
    FD_filt = filtro_exponencial(ValFD, FD_filt, AlphaFiltro)
    FI_filt = filtro_exponencial(ValFI, FI_filt, AlphaFiltro)

    # ── 5. CONVERSIÓN A METROS ───────────────────────────────────────
    DistFD_cruda  = sensor_a_distancia(ValFD)
    DistFI_cruda  = sensor_a_distancia(ValFI)
    DistMin_cruda = min(DistFD_cruda, DistFI_cruda)
    DistFD_ema    = sensor_a_distancia(FD_filt)
    DistFI_ema    = sensor_a_distancia(FI_filt)
    DistMin_ema   = min(DistFD_ema, DistFI_ema)
    z_k = DistMin_ema

    # ── 6. FILTRO DE KALMAN 1D (Lab 2) ──────────────────────────────
    DeltaD_k = -AvanceLineal
    d_pred   = max(0.0, min(d_kalman + DeltaD_k, 0.10))
    P_pred   = P_kalman + Q_kalman
    K_k      = P_pred / (P_pred + R_kalman)
    d_kalman = max(0.0, min(d_pred + K_k * (z_k - d_pred), 0.10))
    P_kalman = (1.0 - K_k) * P_pred

    # ── 7. ÁRBITRO DE DECISIONES ─────────────────────────────────────
    #
    #  Prioridad (mayor a menor):
    #    1. Meta alcanzada         → detener
    #    2. Obstáculo detectado    → rodeo local con sesgo
    #    3. Default                → seguir waypoint A*

    AmenazaLateralDerecha   = max(ValDD, ValLD)
    AmenazaLateralIzquierda = max(ValDI, ValLI)

    UmbralSesgoActivacion = UmbralDiagonal
    UmbralSesgoLiberacion  = UmbralDiagonal - 120

    if SesgoDerechaActivo:
        if AmenazaLateralIzquierda < UmbralSesgoLiberacion and DistFI_ema > UmbralDistDeteccion:
            SesgoDerechaActivo = False
    else:
        if AmenazaLateralIzquierda > UmbralSesgoActivacion or DistFI_ema < UmbralDistDeteccion:
            SesgoDerechaActivo = True

    UmbralDetActual  = (UmbralDistDeteccion + UmbralDistHisteresis) \
                        if DireccionGiroBloqueada else UmbralDistDeteccion
    UmbralDiagActual = (UmbralDiagonal - Histeresis) \
                        if DireccionGiroBloqueada else UmbralDiagonal

    FrenteBloqueado = (d_kalman < UmbralDetActual
                       or DistFD_ema < UmbralDetActual
                       or DistFI_ema < UmbralDetActual
                       or ValDD > UmbralDiagActual
                       or ValDI > UmbralDiagActual)
    PeligroCritico  = d_kalman < UmbralDistPeligro

    # — Verificar si se alcanzó la meta —
    DistMeta = math.sqrt((PoseX - GOAL_X)**2 + (PoseY - GOAL_Y)**2)
    if EstadoArbitro != 'meta' and IndiceWaypoint >= len(waypoints) - 1 and DistMeta < UMBRAL_META:
        EstadoArbitro = 'meta'
        print(f"[META] ¡Meta alcanzada en t={TiempoActual:.1f}s! Pos=({PoseX:.3f},{PoseY:.3f})")

    if EstadoArbitro == 'meta':
        VelocidadIzquierda = 0.0
        VelocidadDerecha   = 0.0

    elif EstadoArbitro == 'bypass' or FrenteBloqueado or PeligroCritico:
        # — Rodeo local: avanza sesgado hasta recuperar espacio libre —
        EstadoArbitro = 'bypass'
        if DireccionGiroBloqueada is None:
            if AmenazaLateralIzquierda >= AmenazaLateralDerecha or DistFI_ema < UmbralDistDeteccion:
                DireccionGiroBloqueada = 'DERECHA'
            else:
                DireccionGiroBloqueada = 'IZQUIERDA'
            PasosBypass = 0

        PasosBypass += 1
        MultAvance = 0.42 if PeligroCritico else 0.34
        MultGiro   = 0.58 if PeligroCritico else 0.42
        if DireccionGiroBloqueada == 'DERECHA':
            VelocidadIzquierda = VelocidadMaxima * MultAvance
            VelocidadDerecha   = VelocidadMaxima * MultGiro
        else:
            VelocidadIzquierda = VelocidadMaxima * MultGiro
            VelocidadDerecha   = VelocidadMaxima * MultAvance

        FrontLibre = (d_kalman > UmbralDetActual and DistFD_ema > UmbralDetActual and DistFI_ema > UmbralDetActual)
        LadoLibre   = (DistFI_ema > UmbralDistDeteccion + 0.015 and DistFD_ema > UmbralDistDeteccion + 0.015)
        if PasosBypass >= MinPasosBypass and FrontLibre and LadoLibre:
            PasosSalidaObstaculo = 0
            DireccionGiroBloqueada = None
            EstadoArbitro = 'waypoint'

    else:
        # — Seguimiento de waypoints A* (Proyecto Final) —
        EstadoArbitro          = 'waypoint'
        DireccionGiroBloqueada = None

        # Avanzar índice si waypoint actual fue alcanzado
        while IndiceWaypoint < len(waypoints) - 1:
            wx, wy = waypoints[IndiceWaypoint]
            dist   = math.sqrt((PoseX - wx)**2 + (PoseY - wy)**2)
            if dist < UMBRAL_WAYPOINT:
                IndiceWaypoint += 1
            else:
                break

        wx, wy = waypoints[min(IndiceWaypoint, len(waypoints)-1)]

        # Error angular hacia el waypoint
        dx = wx - PoseX
        dy = wy - PoseY
        DistWP = math.hypot(dx, dy)
        # CONDICIONAL DE PROTECCIÓN ANTI-NaN:
        if DistWP < 0.001:  # Si la distancia es menor a 1 milímetro
            ErrorAngular = 0.0
            AnguloObjetivo = PosePhi
        else:
            AnguloObjetivo = math.atan2(dy, dx)
            ErrorAngular   = normalizar_angulo(AnguloObjetivo - PosePhi)

        if SesgoDerechaActivo:
            ErrorAngular = normalizar_angulo(ErrorAngular - 0.25)

        # Velocidad traslacional atenuada por coseno del error angular
        # (si el waypoint está de costado, el robot pivota sin avanzar)
        cos_err = math.cos(ErrorAngular)
        VelTraslacional = VELOCIDAD_BASE * max(0.0, cos_err)
        if SesgoDerechaActivo:
            VelTraslacional *= 0.9

        # Velocidad rotacional proporcional
        VelRotacional = KP_ANGULAR * ErrorAngular

        # Cinemática diferencial inversa
        VelocidadIzquierda = VelTraslacional - VelRotacional
        VelocidadDerecha   = VelTraslacional + VelRotacional

        # Saturar
        VelocidadIzquierda = max(min(VelocidadIzquierda, VelocidadMaxima), -VelocidadMaxima)
        VelocidadDerecha   = max(min(VelocidadDerecha,   VelocidadMaxima), -VelocidadMaxima)

    if not (math.isfinite(VelocidadIzquierda) and math.isfinite(VelocidadDerecha)):
        print(
            f"[WARN] Velocidad no finita detectada: izq={VelocidadIzquierda}, der={VelocidadDerecha}. "
            "Forzando parada segura."
        )
        VelocidadIzquierda = 0.0
        VelocidadDerecha = 0.0

    MotorIzquierdo.setVelocity(VelocidadIzquierda)
    MotorDerecho.setVelocity(VelocidadDerecha)

    # ── 8. REGISTRO CSV ──────────────────────────────────────────────
    if TiempoActual - UltimoTiempoGuardado >= IntervaloCSV:
        UltimoTiempoGuardado = TiempoActual

        def r6(v): return round(v, 6) if isinstance(v, float) else v

        esc_crudos.writerow([r6(v) for v in [
            ValFD, ValDD, ValLD, ValTD, ValTI, ValLI, ValDI, ValFI,
            ThetaIzqActual, ThetaDerActual, DeltaSIzq, DeltaSDer,
            AvanceLineal, GiroAngular, VelocidadLineal, VelocidadAngular, TiempoActual
        ]])

        esc_kalman.writerow([r6(v) for v in [
            DistFD_cruda, DistFI_cruda, DistMin_cruda,
            DistFD_ema, DistFI_ema, DistMin_ema,
            d_pred, d_kalman, K_k, P_kalman, AvanceLineal, TiempoActual
        ]])

        wp_x, wp_y = waypoints[min(IndiceWaypoint, len(waypoints)-1)]
        dist_wp    = math.sqrt((PoseX-wp_x)**2 + (PoseY-wp_y)**2)
        dx_wp = wp_x - PoseX; dy_wp = wp_y - PoseY
        err_ang = normalizar_angulo(math.atan2(dy_wp, dx_wp) - PosePhi)

        esc_pose.writerow([r6(v) for v in [
            PoseX, PoseY, PosePhi,
            IndiceWaypoint, wp_x, wp_y,
            dist_wp, err_ang,
            EstadoArbitro, TiempoActual
        ]])

# ── 9. CIERRE DE ARCHIVOS ────────────────────────────────────────
arch_crudos.close()
arch_kalman.close()
arch_pose.close()
print("[FIN] Simulación terminada. Archivos CSV guardados en /data/")