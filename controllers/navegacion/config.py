"""
Configuracion global para el controlador 'navegacion' del mundo facil.

Arena: RectangleArena 2.8 x 2.8 m, centrada en (0,0).
Grilla: 14 x 14 celdas de 0.2 m/celda.
Robot:  e-puck diferencial.
"""

import math

# ---------------------------------------------------------------------------
# Modo de decision de la senal frontal
#   "crudo"    -> sensor frontal sin filtrar
#   "filtrado" -> media movil
#   "kalman"   -> Kalman 1D (recomendado)
# ---------------------------------------------------------------------------
MODO_DECISION = "kalman"

# ---------------------------------------------------------------------------
# Parametros fisicos del e-puck
# ---------------------------------------------------------------------------
RADIO_RUEDA      = 0.0205    # m
DISTANCIA_RUEDAS = 0.052     # m  (distancia entre centros de ruedas)
VELOCIDAD_MAX    = 6.28      # rad/s (velocidad maxima del motor)

# ---------------------------------------------------------------------------
# Tiempo de muestreo
# ---------------------------------------------------------------------------
TIME_STEP = 64               # ms  (paso de simulacion Webots)
TS        = TIME_STEP / 1000.0   # s
FS        = 1.0 / TS             # Hz

# ---------------------------------------------------------------------------
# Umbrales de navegacion reactiva
# Los sensores PS del e-puck devuelven valores MAS ALTOS cuando el
# obstaculo esta MAS CERCA.
# ---------------------------------------------------------------------------
UMBRAL_FRONTAL        = 300.0   # Incrementado para retrasar el giro reactivo
UMBRAL_LATERAL        = 600.0   # Elevado para ignorar las paredes del pasillo estándar
UMBRAL_LATERAL_EXTREMO= 1200.0  # Escape solo en colisión inminente
ZONA_MUERTA_LATERAL   =  45.0   # banda muerta para decision de giro

PASOS_ESCAPE  = 15              # Reducido para acortar el secuestro de motores
PASOS_SALIDA  = 5               # Reducido

# ---------------------------------------------------------------------------
# Velocidades de navegacion reactiva (rad/s sobre los motores)
# ---------------------------------------------------------------------------
VEL_AVANCE        = 3.0
VEL_GIRO          = 2.5
GANANCIA_CENTRADO = 0.004
CORRECCION_MAX    = 0.45

# ---------------------------------------------------------------------------
# Filtrado y fusion sensorial
# ---------------------------------------------------------------------------
VENTANA_MEDIA_MOVIL = 5     # muestras para la media movil
Q_KALMAN            = 0.8   # ruido de proceso Kalman
R_KALMAN            = 22.0  # ruido de medicion Kalman
ESCALA_SENSOR       = 120.0 # factor de escala encoder -> unidades sensor

# ---------------------------------------------------------------------------
# Navegacion global: A* sobre grilla de ocupacion
# ---------------------------------------------------------------------------
TAMANO_CELDA   = 0.2        # m/celda
MOVIMIENTO_ASTAR = "4"      # "4" (Manhattan) o "8" (Euclidiana)

# Origen Webots del centro de la celda (0, 0) de la grilla.
# La arena va de -1.4 a 1.4 en X e Y.
# Centro de la celda (fila=0, col=0):
#   x = -1.4 + 0.5*0.2 = -1.3
#   y =  1.4 - 0.5*0.2 =  1.3
ORIGEN_WEBOTS = (-1.3, 1.3)

# Nombre del archivo CSV de la grilla (debe estar junto a este .py o
# indicar la ruta relativa desde la raiz del proyecto)
CSV_GRILLA = "facil_grid.csv"

# Orientacion inicial del robot en el mundo (aprox. 100 grados ~ 1.748 rad)
THETA_INICIAL = 1.74776

# ---------------------------------------------------------------------------
# Seguimiento de waypoints
# ---------------------------------------------------------------------------
WAYPOINT_TOLERANCIA       = 0.08    # m  distancia para dar wp por alcanzado
WAYPOINT_VEL_LINEAL       = 0.045   # m/s velocidad nominal de avance
WAYPOINT_GANANCIA_ANGULAR = 3.2     # ganancia del controlador angular
WAYPOINT_GIRO_EN_SITIO    = 0.35    # rad umbral para girar sin avanzar

# ---------------------------------------------------------------------------
# Deteccion y recuperacion ante bloqueo
# ---------------------------------------------------------------------------
BLOQUEO_AVANCE_MIN         = 0.0005   # m  avance minimo por ciclo
BLOQUEO_CICLOS             = 35       # ciclos sin avance para activar recuperacion
RECUPERACION_RETROCESO_PASOS = 18
RECUPERACION_GIRO_PASOS      = 28
RECUPERACION_VEL_RETROCESO   = -2.0  # rad/s
RECUPERACION_VEL_GIRO        =  2.2  # rad/s

# ---------------------------------------------------------------------------
# Etiquetas de acciones (para consola y CSV)
# ---------------------------------------------------------------------------
ACCION_SEGUIR_RUTA          = "SEGUIR_RUTA"
ACCION_GIRAR_A_WAYPOINT     = "GIRAR_A_WAYPOINT"
ACCION_META_ALCANZADA       = "META_ALCANZADA"
ACCION_RECUPERAR_RETROCEDER = "RECUPERAR_RETROCEDER"
ACCION_RECUPERAR_GIRAR      = "RECUPERAR_GIRAR"
ACCION_AVANZAR              = "AVANZAR"
ACCION_GIRAR_IZQUIERDA      = "GIRAR_IZQUIERDA"
ACCION_GIRAR_DERECHA        = "GIRAR_DERECHA"
ACCION_ESCAPE_IZQUIERDA     = "ESCAPE_IZQUIERDA"
ACCION_ESCAPE_DERECHA       = "ESCAPE_DERECHA"
ACCION_SALIDA_ESCAPE        = "SALIDA_ESCAPE"
ACCION_CENTRAR_IZQUIERDA    = "CENTRAR_IZQUIERDA"
ACCION_CENTRAR_DERECHA      = "CENTRAR_DERECHA"

ACCIONES = [
    ACCION_AVANZAR,
    ACCION_GIRAR_IZQUIERDA,
    ACCION_GIRAR_DERECHA,
    ACCION_ESCAPE_IZQUIERDA,
    ACCION_ESCAPE_DERECHA,
    ACCION_SALIDA_ESCAPE,
    ACCION_CENTRAR_IZQUIERDA,
    ACCION_CENTRAR_DERECHA,
    ACCION_SEGUIR_RUTA,
    ACCION_GIRAR_A_WAYPOINT,
    ACCION_RECUPERAR_RETROCEDER,
    ACCION_RECUPERAR_GIRAR,
    ACCION_META_ALCANZADA,
]

# ---------------------------------------------------------------------------
# Utilidades de formato
# ---------------------------------------------------------------------------
BOLD  = "\033[1m"
RESET = "\033[0m"


def negrita(texto: str) -> str:
    """Devuelve el texto con codigo ANSI de negrita."""
    return f"{BOLD}{texto}{RESET}"


def separador() -> None:
    print("=" * 60)


def limitar(valor: float, minimo: float, maximo: float) -> float:
    """Satura un valor al intervalo [minimo, maximo]."""
    return max(minimo, min(maximo, valor))