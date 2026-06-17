"""
planner.py
Persona 2 — Planificacion de rutas con A*

Funciones publicas:
    plan_path(grid, start, goal)          -> [(fila, col), ...]
    cells_to_world(path, origin, cell_size) -> [(x, y), ...]

Para probar sin Webots:
    python planner.py
"""

import heapq
import math
import numpy as np


# ---------------------------------------------------------------------------
# A* — implementacion compacta con heapq
# ---------------------------------------------------------------------------

def _heuristic(a, b):
    """Distancia euclidea como heuristica admisible."""
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def plan_path(grid, start, goal):
    """
    Calcula la ruta optima en la grilla usando A*.

    Parametros
    ----------
    grid  : numpy.ndarray 2D (filas x cols)
            0 = libre, 1 = obstaculo
    start : (fila, col)  celda de inicio
    goal  : (fila, col)  celda destino

    Retorna
    -------
    list[(fila, col)]  camino desde start hasta goal (inclusive),
                       o lista vacia si no existe ruta.
    """
    rows, cols = grid.shape

    # Movimientos: 4 cardinales + 4 diagonales
    neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1),
                 (-1, -1), (-1, 1), (1, -1), (1, 1)]

    # g_cost[nodo] = costo acumulado desde start
    g_cost = {start: 0.0}
    came_from = {}

    # heap: (f, g, nodo)
    heap = [(0.0 + _heuristic(start, goal), 0.0, start)]

    while heap:
        f, g, current = heapq.heappop(heap)

        if current == goal:
            # Reconstruir camino
            path = []
            node = goal
            while node in came_from:
                path.append(node)
                node = came_from[node]
            path.append(start)
            path.reverse()
            return path

        # Si ya encontramos un camino mejor, ignorar
        if g > g_cost.get(current, float('inf')):
            continue

        r, c = current
        for dr, dc in neighbors:
            nr, nc = r + dr, c + dc

            # Verificar limites
            if not (0 <= nr < rows and 0 <= nc < cols):
                continue
            # Verificar obstaculo
            if grid[nr, nc] != 0:
                continue

            # Costo del paso (diagonal cuesta mas)
            step = math.sqrt(2) if (dr != 0 and dc != 0) else 1.0
            new_g = g + step

            neighbor = (nr, nc)
            if new_g < g_cost.get(neighbor, float('inf')):
                g_cost[neighbor] = new_g
                came_from[neighbor] = current
                f_new = new_g + _heuristic(neighbor, goal)
                heapq.heappush(heap, (f_new, new_g, neighbor))

    return []   # Sin ruta


# ---------------------------------------------------------------------------
# Conversion celda -> coordenadas mundo
# ---------------------------------------------------------------------------

def cells_to_world(path, origin, cell_size):
    """
    Convierte una lista de celdas (fila, col) a coordenadas reales (x, y).

    Parametros
    ----------
    path      : [(fila, col), ...]  salida de plan_path
    origin    : (x0, y0)  esquina inferior-izquierda del mapa en metros
    cell_size : float     metros por celda

    Retorna
    -------
    [(x, y), ...]  centros de celda en metros
                   Nota: asume la misma convencion que grid_simple/complejo,
                   donde fila 0 = Y maximo.
    """
    if not path:
        return []

    # Necesitamos conocer el numero total de filas para invertir la fila.
    # Lo inferimos del maximo indice de fila en el path.
    max_row = max(r for r, c in path)
    # Esto subestima si goal no es la fila maxima; mejor pasar rows aparte,
    # pero para compatibilidad con la interfaz del README lo deducimos del grid
    # si esta disponible, o usamos el maximo del path como aproximacion.
    # -> La funcion acepta opcionalmente n_rows como cuarto argumento.
    return _cells_to_world_impl(path, origin, cell_size, None)


def _cells_to_world_impl(path, origin, cell_size, n_rows):
    """Implementacion interna que acepta n_rows opcional."""
    result = []
    if n_rows is None:
        n_rows = max(r for r, c in path) + 1  # estimacion minima

    for (row, col) in path:
        x = origin[0] + (col + 0.5) * cell_size
        y = origin[1] + ((n_rows - 1 - row) + 0.5) * cell_size
        result.append((x, y))
    return result


# ---------------------------------------------------------------------------
# Visualizacion con matplotlib (solo para pruebas sin Webots)
# ---------------------------------------------------------------------------

def visualize(grid, path, origin, cell_size, title="Ruta A*", filename=None):
    """
    Dibuja la grilla y la ruta usando matplotlib.

    Parametros
    ----------
    grid      : numpy array 2D
    path      : [(fila, col), ...]
    origin    : (x0, y0)
    cell_size : float
    title     : str
    filename  : str o None  (si se da, guarda PNG)
    """
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches

    rows, cols = grid.shape
    extent = [origin[0], origin[0] + cols * cell_size,
              origin[1], origin[1] + rows * cell_size]

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.imshow(grid, cmap="gray_r", origin="upper", extent=extent,
              vmin=0, vmax=1, alpha=0.6)

    if path:
        waypoints = _cells_to_world_impl(path, origin, cell_size, rows)
        xs = [p[0] for p in waypoints]
        ys = [p[1] for p in waypoints]
        ax.plot(xs, ys, "b-", linewidth=2, label="Ruta A*")
        ax.plot(xs[0],  ys[0],  "go", markersize=10, label="Inicio")
        ax.plot(xs[-1], ys[-1], "r*", markersize=14, label="Meta")

    ax.set_title(title)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.legend()
    ax.grid(True, linewidth=0.3, alpha=0.4)
    plt.tight_layout()

    if filename:
        plt.savefig(filename, dpi=130)
        print(f"[planner] Guardado: {filename}")
    plt.show()


# ---------------------------------------------------------------------------
# Prueba autonoma
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    print("=== Prueba escenario SIMPLE ===")
    from grid_simple import GRID as G_S, ORIGIN as O_S, CELL_SIZE as CS_S, world_to_cell as w2c_s

    start_s = w2c_s(-0.35, -0.33)   # posicion inicial del e-puck en facil.wbt
    goal_s  = w2c_s( 0.276,  -0.190)   # meta arbitraria (esquina inferior derecha)

    print(f"  Inicio celda: {start_s}  ->  mundo {(-0.35, -0.33)}")
    print(f"  Meta  celda: {goal_s}   ->  mundo {(0.276, -0.190)}")

    path_s = plan_path(G_S, start_s, goal_s)
    print(f"  Ruta: {len(path_s)} celdas")

    if path_s:
        wps_s = cells_to_world(path_s, O_S, CS_S)
        print(f"  Primeros waypoints (m): {wps_s[:4]}")
        visualize(G_S, path_s, O_S, CS_S,
                  title="Escenario simple — A*",
                  filename="ruta_simple.png")
    else:
        print("  [!] No se encontro ruta")

    print()
    print("=== Prueba escenario COMPLEJO ===")
    from grid_complejo import GRID as G_C, ORIGIN as O_C, CELL_SIZE as CS_C, world_to_cell as w2c_c

    start_c = w2c_c(-0.505, -0.505)  # posicion inicial del e-puck en dificil.wbt
    goal_c  = w2c_c( -0.178,   0.597)   # meta arbitraria zona superior derecha

    print(f"  Inicio celda: {start_c}")
    print(f"  Meta  celda: {goal_c}")

    path_c = plan_path(G_C, start_c, goal_c)
    print(f"  Ruta: {len(path_c)} celdas")

    if path_c:
        wps_c = cells_to_world(path_c, O_C, CS_C)
        print(f"  Primeros waypoints (m): {wps_c[:4]}")
        visualize(G_C, path_c, O_C, CS_C,
                  title="Escenario complejo — A*",
                  filename="ruta_complejo.png")
    else:
        print("  [!] No se encontro ruta")