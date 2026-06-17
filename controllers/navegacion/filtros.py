"""
filtros.py
Persona 2 — Filtrado de sensores de proximidad

Aplica un filtro de media movil simple (Moving Average) sobre los 8 valores
de proximidad del e-puck para estabilizar las lecturas antes de usarlas
en evasion reactiva y registro de metricas.

Uso tipico en el main loop:
    import filtros
    ...
    raw  = cin.get_sensor_values()        # lista de 8 floats
    suav = filtros.filter_sensors(raw)    # lista de 8 floats filtrados
"""

from collections import deque

# ---------------------------------------------------------------------------
# Configuracion del filtro
# ---------------------------------------------------------------------------

# Numero de muestras que promedia la ventana.
# - Ventana chica (3-5): respuesta rapida, poco suavizado.
# - Ventana grande (8-10): mas suavizado, pero introduce mas retardo.
WINDOW_SIZE = 5

# ---------------------------------------------------------------------------
# Estado interno: una ventana deslizante por cada sensor (ps0..ps7)
# ---------------------------------------------------------------------------
_NUM_SENSORS = 8
_windows = [deque(maxlen=WINDOW_SIZE) for _ in range(_NUM_SENSORS)]


# ---------------------------------------------------------------------------
# API publica
# ---------------------------------------------------------------------------

def filter_sensors(raw):
    """
    Recibe los 8 valores crudos de proximidad y devuelve la lista suavizada.

    Parametros
    ----------
    raw : list[float]  — salida directa de cin.get_sensor_values() (8 valores)

    Retorna
    -------
    list[float]  — 8 valores filtrados por media movil.
                   Mientras la ventana no este llena, promedia las muestras
                   disponibles (comportamiento correcto desde el primer paso).
    """
    smoothed = []
    for i, value in enumerate(raw):
        _windows[i].append(value)
        avg = sum(_windows[i]) / len(_windows[i])
        smoothed.append(avg)
    return smoothed


def reset():
    """
    Limpia todas las ventanas. Util al iniciar una nueva prueba o escenario
    para que el filtro no arrastre valores del run anterior.
    """
    for w in _windows:
        w.clear()


def get_window_size():
    """Retorna el tamano de ventana configurado (util para logs/README)."""
    return WINDOW_SIZE


# ---------------------------------------------------------------------------
# Prueba autonoma  —  python filtros.py
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import random

    print(f"=== Prueba de media movil (ventana={WINDOW_SIZE}) ===\n")

    # Simular 12 pasos con sensores ruidosos
    random.seed(42)
    for step in range(12):
        raw = [0.0] * 8
        raw[0] = 100.0 + random.uniform(-20, 20)   # ps0: valor ruidoso
        raw[7] = 50.0  + random.uniform(-10, 10)   # ps7: valor ruidoso

        suav = filter_sensors(raw)

        print(f"Paso {step+1:2d} | "
              f"ps0 crudo={raw[0]:6.1f}  suav={suav[0]:6.2f} | "
              f"ps7 crudo={raw[7]:6.1f}  suav={suav[7]:6.2f}")

    print("\n--- Reset y nuevo inicio ---")
    reset()
    raw_test = [200.0] * 8
    suav_test = filter_sensors(raw_test)
    print(f"Primer paso post-reset: suav[0]={suav_test[0]:.2f}  (debe ser 200.00)")