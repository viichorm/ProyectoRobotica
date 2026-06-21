"""
Indices y utilidades de lectura para los 8 sensores de proximidad del e-puck.

Distribucion de sensores (vista superior):
        ps7  ps0
      ps6      ps1
    ps5          ps2
      ps4      ps3

Los sensores devuelven valores mas altos cuando el obstaculo esta mas cerca.
"""

from config import TIME_STEP

# Indices de los sensores relevantes
IDX_FRONTAL_DER = 0   # ps0 - frontal derecho
IDX_FRONTAL_IZQ = 7   # ps7 - frontal izquierdo
IDX_LATERAL_DER = 2   # ps2 - lateral derecho
IDX_LATERAL_IZQ = 5   # ps5 - lateral izquierdo


def inicializar_sensores_proximidad(robot):
    """Habilita los 8 sensores PS y devuelve la lista."""
    sensores = []
    for i in range(8):
        sensor = robot.getDevice(f"ps{i}")
        sensor.enable(TIME_STEP)
        sensores.append(sensor)
    return sensores


def leer_sensores_proximidad(sensores):
    """Devuelve una lista con el valor actual de cada sensor."""
    return [s.getValue() for s in sensores]


def obtener_senal_frontal(valores_ps):
    """Promedio de los dos sensores frontales (ps0 y ps7)."""
    return (valores_ps[IDX_FRONTAL_DER] + valores_ps[IDX_FRONTAL_IZQ]) / 2.0