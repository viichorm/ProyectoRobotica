"""
Algoritmo A* sobre grilla de ocupacion 2D.

Soporta movimiento en 4 u 8 direcciones.
Heuristica Manhattan para 4-direcciones, Euclidiana para 8-direcciones.
"""

import heapq
import math


# Movimientos cardinales (coste 1) y diagonales (coste sqrt(2))
_MOVIMIENTOS_4 = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0)]
_MOVIMIENTOS_8 = _MOVIMIENTOS_4 + [
    (-1, -1, math.sqrt(2.0)),
    (-1,  1, math.sqrt(2.0)),
    ( 1, -1, math.sqrt(2.0)),
    ( 1,  1, math.sqrt(2.0)),
]


# ---------------------------------------------------------------------------
# Funciones auxiliares
# ---------------------------------------------------------------------------

def _heuristica(celda, meta, tipo: str) -> float:
    dr = abs(celda[0] - meta[0])
    dc = abs(celda[1] - meta[1])
    if tipo == "8":
        return math.sqrt(dr * dr + dc * dc)
    return dr + dc   # Manhattan


def _movimientos(tipo: str):
    return _MOVIMIENTOS_8 if tipo == "8" else _MOVIMIENTOS_4


def _dentro_de_limites(grilla, celda) -> bool:
    if hasattr(grilla, "dentro_de_limites"):
        return grilla.dentro_de_limites(celda)
    r, c = celda
    return 0 <= r < len(grilla) and 0 <= c < len(grilla[0])


def _es_libre(grilla, celda) -> bool:
    if hasattr(grilla, "es_libre"):
        return grilla.es_libre(celda)
    if not _dentro_de_limites(grilla, celda):
        return False
    r, c = celda
    return grilla[r][c] != 1


def _reconstruir(came_from: dict, inicio, meta) -> list:
    if meta not in came_from and meta != inicio:
        return []
    actual = meta
    ruta = [actual]
    while actual != inicio:
        actual = came_from[actual]
        ruta.append(actual)
    ruta.reverse()
    return ruta


# ---------------------------------------------------------------------------
# Funcion principal
# ---------------------------------------------------------------------------

def astar(grilla, inicio, meta, tipo_movimiento: str = "4") -> list:
    """
    Ejecuta A* y devuelve la lista de celdas desde 'inicio' hasta 'meta'.

    Args:
        grilla:           OccupancyGrid o lista 2D (0=libre, 1=obstaculo).
        inicio:           tupla (fila, columna).
        meta:             tupla (fila, columna).
        tipo_movimiento:  "4" (cardinal) o "8" (con diagonales).

    Returns:
        Lista de celdas [inicio, ..., meta] o [] si no hay ruta.
    """
    if not _dentro_de_limites(grilla, inicio) or not _dentro_de_limites(grilla, meta):
        return []
    if not _es_libre(grilla, inicio) or not _es_libre(grilla, meta):
        return []

    frontera: list = []
    contador = 0
    heapq.heappush(frontera, (0.0, contador, inicio))

    came_from: dict = {}
    costo_g: dict   = {inicio: 0.0}
    visitados: set  = set()

    movs = _movimientos(tipo_movimiento)

    while frontera:
        _, _, actual = heapq.heappop(frontera)

        if actual in visitados:
            continue

        if actual == meta:
            ruta = _reconstruir(came_from, inicio, meta)
            # Validar que ninguna celda de la ruta sea obstaculo
            if all(_es_libre(grilla, c) for c in ruta):
                return ruta
            return []

        visitados.add(actual)

        for dr, dc, coste in movs:
            vecino = (actual[0] + dr, actual[1] + dc)
            if not _es_libre(grilla, vecino):
                continue
            nuevo_g = costo_g[actual] + coste
            if nuevo_g < costo_g.get(vecino, float("inf")):
                costo_g[vecino] = nuevo_g
                f = nuevo_g + _heuristica(vecino, meta, tipo_movimiento)
                contador += 1
                heapq.heappush(frontera, (f, contador, vecino))
                came_from[vecino] = actual

    return []