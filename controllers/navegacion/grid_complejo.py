"""
grid_complejo.py
Grilla de ocupacion 2D exacta para el escenario dificil.wbt
Arena: 1.2 x 1.2 m  ->  grilla 24 x 24 celdas de 0.05 m
ORIGIN: (-0.6, -0.6)  esquina inferior-izquierda

Fila 0 = Y maximo (norte), col 0 = X minimo (oeste).
"""

import numpy as np
import math

CELL_SIZE = 0.05
ORIGIN    = (-0.6, -0.6)
_ROWS     = 24
_COLS     = 24

# Inflación moderada (2 cm) para el laberinto. 
# Evita que roce esquinas pero no asfixia los pasillos estrechos.
INFLATION = 0.02
# ---------------------------------------------------------------------------
# Conversiones
# ---------------------------------------------------------------------------

def world_to_cell(x, y):
    col = int((x - ORIGIN[0]) / CELL_SIZE)
    row = int((y - ORIGIN[1]) / CELL_SIZE)
    row = (_ROWS - 1) - row
    return (max(0, min(_ROWS-1, row)), max(0, min(_COLS-1, col)))

def cell_to_world(row, col):
    x = ORIGIN[0] + (col + 0.5) * CELL_SIZE
    y = ORIGIN[1] + ((_ROWS - 1 - row) + 0.5) * CELL_SIZE
    return (x, y)

# ---------------------------------------------------------------------------
# Herramientas de marcado exacto
# ---------------------------------------------------------------------------

def _mark_exact_box(grid, cx, cy, size_x, size_y, angle_rad=0.0):
    """Marca celdas verificando si su centro cae dentro del rectángulo rotado e inflado."""
    sx = (size_x / 2.0) + INFLATION
    sy = (size_y / 2.0) + INFLATION
    cos_a, sin_a = math.cos(-angle_rad), math.sin(-angle_rad)

    for r in range(_ROWS):
        for c in range(_COLS):
            wx = ORIGIN[0] + (c + 0.5) * CELL_SIZE
            wy = ORIGIN[1] + ((_ROWS - 1 - r) + 0.5) * CELL_SIZE
            dx, dy = wx - cx, wy - cy
            
            # Rotar punto a las coordenadas locales del rectángulo
            lx = dx * cos_a - dy * sin_a
            ly = dx * sin_a + dy * cos_a
            
            # Chequeo de colisión
            if abs(lx) <= sx and abs(ly) <= sy:
                grid[r, c] = 1

# ---------------------------------------------------------------------------
# Construccion de la grilla
# ---------------------------------------------------------------------------

def _build_grid():
    grid = np.zeros((_ROWS, _COLS), dtype=np.int8)

    # Formato de los obstaculos: (x, y, ancho_x, alto_y, angulo_rad)
    obstacles = [
        ( 0.02,    -0.09,    0.12, 0.12,  0.0),      # bloque_central
        ( 0.04,     0.12,    0.3,  0.07, -1.5708),   # pared_sup_izq
        (-0.02,     0.29,    0.4,  0.03,  0.0),      # pared_sup_izq(2)
        (-0.26,     0.43,    0.3,  0.07,  1.5708),   # pared_sup_izq(4)
        ( 0.01,     0.47,    0.3,  0.07,  0.0),      # pared_sup_izq(3)
        ( 0.32,     0.12,    0.3,  0.07, -1.5708),   # pared_sup_izq(1)
        ( 0.32,    -0.21,    0.25, 0.07,  1.5708),   # pared_sup_der
        (-0.4131,  -0.2521,  0.07, 0.35, -0.7854),   # pared_vert_izq
        (-0.2646,  -0.4148,  0.07, 0.35, -0.7854),   # pared_vert_izq(1)
        ( 0.13,    -0.28,    0.1,  0.1,   0.0),      # caja_esq_sup_der
        ( 0.23,    -0.28,    0.1,  0.1,   0.0),      # caja_inf_der
        (-0.1501,  -0.1201,  0.22, 0.06,  0.0),      # pared_diagonal
        (-0.0300,  -0.2701,  0.22, 0.06,  0.0),      # pared_diagonal(1)
        ( 0.3,     -0.06,    0.1,  0.05,  0.0),      # caja_esq_sup_izq
        # Pilares (0.06x0.06)
        ( 0.33,     0.31,    0.06, 0.06,  0.0), 
        ( 0.33,     0.38,    0.06, 0.06,  0.0), 
        ( 0.33,     0.45,    0.06, 0.06,  0.0), 
        ( 0.26,     0.47,    0.06, 0.06,  0.0), 
        ( 0.19,     0.47,    0.06, 0.06,  0.0), 
        (-0.1,      0.54,    0.06, 0.06,  0.0),
    ]

    for (tx, ty, sx, sy, ang) in obstacles:
        _mark_exact_box(grid, tx, ty, sx, sy, ang)

    return grid

GRID = _build_grid()

# ---------------------------------------------------------------------------
# Visualizacion
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from collections import deque

    rows, cols = GRID.shape
    libres = int((GRID == 0).sum())
    print(f"Celdas libres: {libres}/{rows*cols} ({100*libres/(rows*cols):.1f}%)")

    # BFS para verificar conectividad desde posicion inicial del robot
    start = world_to_cell(-0.505, -0.505)
    # Cambié la meta para el chequeo a una zona más segura (arriba a la izquierda)
    goal  = world_to_cell(-0.35, 0.40) 
    
    visited = np.zeros_like(GRID, dtype=bool)
    q = deque([start])
    visited[start] = True
    dirs = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
    
    while q:
        r, c = q.popleft()
        for dr, dc in dirs:
            nr, nc = r+dr, c+dc
            if 0<=nr<rows and 0<=nc<cols and not visited[nr,nc] and GRID[nr,nc]==0:
                visited[nr,nc] = True
                q.append((nr,nc))

    alcanzables = int(visited.sum())
    print(f"Celdas alcanzables desde inicio {start}: {alcanzables}")
    print(f"Meta {goal} alcanzable: {visited[goal]}")

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.imshow(GRID, cmap="gray_r", origin="upper",
              extent=[ORIGIN[0], ORIGIN[0]+cols*CELL_SIZE,
                      ORIGIN[1], ORIGIN[1]+rows*CELL_SIZE])
    ax.set_title("Grilla COMPLEJA (Exacta con Inflación)")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    plt.tight_layout()
    plt.savefig("grid_complejo_preview.png", dpi=120)
    plt.show()