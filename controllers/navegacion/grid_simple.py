"""
grid_simple.py
Grilla de ocupacion 2D exacta para facil.wbt.
Arena: 1.0 x 1.0 m  (RectangleArena sin floorSize explicito = 1x1)
CELL_SIZE: 0.05 m  -> grilla de 20 x 20 celdas
ORIGIN: (-0.5, -0.5)  esquina inferior-izquierda
"""
import numpy as np
import math

CELL_SIZE = 0.05
ORIGIN    = (-0.5, -0.5)
_ROWS     = 20
_COLS     = 20

# Margen de seguridad en metros (1.5 cm) para evitar que A* roce las esquinas
INFLATION = 0.01

def world_to_cell(x, y):
    col = int((x - ORIGIN[0]) / CELL_SIZE)
    row = int((y - ORIGIN[1]) / CELL_SIZE)
    row = (_ROWS - 1) - row
    return (max(0, min(_ROWS-1, row)), max(0, min(_COLS-1, col)))

def cell_to_world(row, col):
    x = ORIGIN[0] + (col + 0.5) * CELL_SIZE
    y = ORIGIN[1] + ((_ROWS - 1 - row) + 0.5) * CELL_SIZE
    return (x, y)

def _mark_exact_box(grid, cx, cy, size_x, size_y, angle_rad):
    """Marca celdas verificando si su centro cae dentro del rectangulo rotado."""
    sx = (size_x / 2.0) + INFLATION
    sy = (size_y / 2.0) + INFLATION
    cos_a, sin_a = math.cos(-angle_rad), math.sin(-angle_rad)

    for r in range(_ROWS):
        for c in range(_COLS):
            wx = ORIGIN[0] + (c + 0.5) * CELL_SIZE
            wy = ORIGIN[1] + ((_ROWS - 1 - r) + 0.5) * CELL_SIZE
            
            dx, dy = wx - cx, wy - cy
            # Rotar punto a las coordenadas locales del rectangulo
            lx = dx * cos_a - dy * sin_a
            ly = dx * sin_a + dy * cos_a
            
            if abs(lx) <= sx and abs(ly) <= sy:
                grid[r, c] = 1

def _build_grid():
    grid = np.zeros((_ROWS, _COLS), dtype=np.int8)

    # Obstaculos leidos directamente de facil.wbt con su tamano REAL
    # Formato: (x, y, size_x, size_y, angulo_rad)
    obstacles = [
        (-0.268659, -0.43088,  0.1, 0.1,  2.0944),
        (-0.408659, -0.1884,   0.1, 0.1,  2.0944),
        (-0.192159, -0.0634,   0.4, 0.1, -2.6179),
        ( 0.110946,  0.1116,   0.3, 0.1, -2.6179),
        ( 0.360946, -0.3214,   0.3, 0.1, -2.6179),
        ( 0.297546, -0.0116,   0.3, 0.1,  1.0472),
        ( 0.397546, -0.1848,   0.1, 0.1,  1.0472),
        ( 0.127746, -0.3175,   0.1, 0.1,  1.0472),
        ( 0.177746, -0.4041,   0.1, 0.1,  1.0472),
        (-0.052158, -0.3059,   0.4, 0.1, -2.6179)
    ]

    for (tx, ty, sx, sy, ang) in obstacles:
        _mark_exact_box(grid, tx, ty, sx, sy, ang)

    return grid

GRID = _build_grid()

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    rows, cols = GRID.shape
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.imshow(GRID, cmap="gray_r", origin="upper",
              extent=[ORIGIN[0], ORIGIN[0]+cols*CELL_SIZE,
                      ORIGIN[1], ORIGIN[1]+rows*CELL_SIZE])
    ax.set_title("Grilla SIMPLE (Corregida con rotacion)")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    plt.tight_layout()
    plt.show()