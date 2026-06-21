"""
Grilla de ocupacion 2D y conversion entre celdas y coordenadas Webots.

Convenio de coordenadas para el mundo 'facil':
    - Arena:  2.8 x 2.8 m, centrada en (0, 0).
    - Grilla: 14 x 14 celdas de 0.2 m/celda.
    - Celda (fila=0, col=0): esquina superior izquierda.
    - Centro de la celda (r, c):
          x_webots =  ORIGEN_X + c * TAMANO
          y_webots =  ORIGEN_Y - r * TAMANO
      donde ORIGEN_X = -1.3, ORIGEN_Y = 1.3 (centro de la celda [0,0]).

Valores del CSV:
    0 = libre
    1 = obstaculo
    2 = posicion de inicio
    3 = meta
"""

import csv
import math

from config import ORIGEN_WEBOTS, TAMANO_CELDA


# ---------------------------------------------------------------------------
# Funciones de conversion desacopladas (utiles fuera de la clase)
# ---------------------------------------------------------------------------

def celda_a_webots(fila: int, col: int,
                   tamano: float = TAMANO_CELDA,
                   origen: tuple = ORIGEN_WEBOTS) -> tuple[float, float]:
    """
    Centro de la celda (fila, col) en coordenadas Webots (x, y).

    La fila crece hacia abajo en la matriz; el eje Y de Webots crece
    hacia adelante (arriba en la vista cenital).
    """
    ox, oy = origen
    x = ox + col * tamano
    y = oy - fila * tamano
    return x, y


def webots_a_celda(x: float, y: float,
                   tamano: float = TAMANO_CELDA,
                   origen: tuple = ORIGEN_WEBOTS,
                   limites: tuple | None = None,
                   limitar: bool = True) -> tuple[int, int] | None:
    """
    Coordenadas Webots (x, y) -> indices (fila, columna) de la grilla.

    Si 'limites=(filas, cols)' y 'limitar=True', satura el resultado al
    borde valido en lugar de devolver indices fuera de rango.
    Si 'limitar=False' y la celda esta fuera, devuelve None.
    """
    ox, oy = origen
    col = round((x - ox) / tamano)
    fila = round((oy - y) / tamano)

    if limites is None:
        return fila, col

    filas, cols = limites
    dentro = 0 <= fila < filas and 0 <= col < cols

    if dentro:
        return fila, col

    if not limitar:
        return None

    fila = max(0, min(filas - 1, fila))
    col  = max(0, min(cols  - 1, col))
    return fila, col


# ---------------------------------------------------------------------------
# Clase principal
# ---------------------------------------------------------------------------

class OccupancyGrid:
    LIBRE     = 0
    OBSTACULO = 1
    INICIO    = 2
    META      = 3

    def __init__(self,
                 ruta_csv: str,
                 tamano_celda: float = TAMANO_CELDA,
                 origen_webots: tuple = ORIGEN_WEBOTS):
        self.ruta_csv      = ruta_csv
        self.tamano_celda  = tamano_celda
        self.origen_webots = origen_webots

        self.grid    = self._cargar_csv(ruta_csv)
        self.filas   = len(self.grid)
        self.columnas= len(self.grid[0]) if self.filas else 0

        self.inicio = self._buscar(self.INICIO)
        self.meta   = self._buscar(self.META)

        if self.inicio is None:
            raise ValueError("La grilla no tiene celda de inicio (valor 2).")
        if self.meta is None:
            raise ValueError("La grilla no tiene celda de meta (valor 3).")

    # ------------------------------------------------------------------
    # Carga del CSV
    # ------------------------------------------------------------------

    @staticmethod
    def _cargar_csv(ruta: str) -> list[list[int]]:
        matriz = []
        with open(ruta, newline="") as f:
            for fila in csv.reader(f):
                if not fila:
                    continue
                matriz.append([int(v.strip()) for v in fila])
        if not matriz:
            raise ValueError(f"CSV vacio: {ruta}")
        n_cols = len(matriz[0])
        if any(len(f) != n_cols for f in matriz):
            raise ValueError("La grilla CSV no es rectangular.")
        return matriz

    def _buscar(self, valor: int) -> tuple[int, int] | None:
        for r in range(self.filas):
            for c in range(self.columnas):
                if self.grid[r][c] == valor:
                    return r, c
        return None

    # ------------------------------------------------------------------
    # Consultas sobre la grilla
    # ------------------------------------------------------------------

    def dentro_de_limites(self, celda: tuple) -> bool:
        r, c = celda
        return 0 <= r < self.filas and 0 <= c < self.columnas

    def es_libre(self, celda: tuple) -> bool:
        """Una celda es transitable si esta dentro de limites y NO es obstaculo."""
        if not self.dentro_de_limites(celda):
            return False
        r, c = celda
        return self.grid[r][c] != self.OBSTACULO

    # ------------------------------------------------------------------
    # Conversion de coordenadas
    # ------------------------------------------------------------------

    def celda_a_mundo(self, celda: tuple) -> tuple[float, float]:
        """Devuelve el centro de la celda en coordenadas Webots."""
        return celda_a_webots(celda[0], celda[1],
                               self.tamano_celda, self.origen_webots)

    def mundo_a_celda(self, x: float, y: float,
                      limitar_a_grilla: bool = True) -> tuple[int, int] | None:
        """Convierte coordenadas Webots a (fila, columna)."""
        return webots_a_celda(x, y,
                               self.tamano_celda,
                               self.origen_webots,
                               limites=(self.filas, self.columnas),
                               limitar=limitar_a_grilla)

    # ------------------------------------------------------------------
    # Simplificacion y generacion de waypoints
    # ------------------------------------------------------------------

    def simplificar_ruta(self, ruta: list) -> list:
        """
        Elimina celdas intermedias en tramos rectos.
        Solo conserva los puntos de inflexion y los extremos.
        """
        if len(ruta) <= 2:
            return list(ruta)

        simplificada = [ruta[0]]
        dir_ant = None

        for i in range(1, len(ruta)):
            dr = ruta[i][0] - ruta[i - 1][0]
            dc = ruta[i][1] - ruta[i - 1][1]
            dir_act = (dr, dc)
            if dir_ant is not None and dir_act != dir_ant:
                simplificada.append(ruta[i - 1])
            dir_ant = dir_act

        simplificada.append(ruta[-1])
        return simplificada

    def path_to_waypoints(self, ruta: list,
                          simplificar: bool = True) -> list[tuple[float, float]]:
        """Convierte una ruta de celdas en coordenadas Webots."""
        ruta_uso = self.simplificar_ruta(ruta) if simplificar else ruta
        return [self.celda_a_mundo(c) for c in ruta_uso]

    # ------------------------------------------------------------------
    # Metricas
    # ------------------------------------------------------------------

    def calcular_longitud_ruta(self, ruta: list) -> float:
        """Longitud total de la ruta en metros."""
        if len(ruta) < 2:
            return 0.0
        total = 0.0
        for i in range(1, len(ruta)):
            x1, y1 = self.celda_a_mundo(ruta[i - 1])
            x2, y2 = self.celda_a_mundo(ruta[i])
            total += math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        return total