# Proyecto Final — Navegación Autónoma en Webots
**ICI 4150 · Robótica y Sistemas Autónomos 2026-01**  
Línea A: Planificación de rutas con A*  
Lenguaje: Python · Simulador: Webots · Robot: e-puck

---

## Estado del proyecto

### Persona 1 — Infraestructura y control cinemático
- [x] Escenario simple diseñado en Webots
- [x] Escenario complejo diseñado en Webots
- [x] Configuración del e-puck (motores, encoders, sensores ps0–ps7)
- [x] Cinemática diferencial (`set_velocity`)
- [x] Odometría con encoders (`get_pose`, `update_odometry`)
- [x] Detección de obstáculos (`is_obstacle_ahead`)
- [x] Evasión reactiva local (`reactive_avoid`)
- [x] Calibración de umbral de sensores (threshold = 80)

### Persona 2 — Algoritmia y navegación global
- [ ] Grilla de ocupación 2D — escenario simple (`grid_simple.py`)
- [ ] Grilla de ocupación 2D — escenario complejo (`grid_complejo.py`)
- [ ] Algoritmo A* (`plan_path`)
- [ ] Conversión ruta → waypoints reales (`cells_to_world`)
- [ ] Filtrado de sensores (`filtros.py`)

### Persona 3 — Integración, análisis y documentación
- [ ] Loop principal con navegación hacia waypoints (`navegacion.py`)
- [ ] Registro de métricas en CSV
- [ ] Gráficos ruta planificada vs trayectoria real
- [ ] README completo
- [ ] Video demostrativo (escenario simple y complejo)

---

## Archivos del proyecto

| Archivo | Responsable | Estado |
|---|---|---|
| `navegacion.py` | P1 / P3 | En progreso |
| `cinematica.py` | P1 | Listo |
| `planner.py` | P2 | Pendiente |
| `grid_simple.py` | P2 | Pendiente |
| `grid_complejo.py` | P2 | Pendiente |
| `filtros.py` | P2 | Pendiente |
| `worlds/escenario_simple.wbt` | P1 | Listo |
| `worlds/escenario_complejo.wbt` | P1 | Listo |

---

## Interfaz entre módulos

```python
# Lo que P1 expone a P2 y P3
cin.get_pose()            # -> (x, y, theta) en metros y radianes
cin.set_velocity(v, omega) # -> None
cin.is_obstacle_ahead()   # -> bool
cin.get_sensor_values()   # -> [ps0..ps7]

# Lo que P2 expone a P3
plan_path(grid, start, goal)          # -> [(fila, col), ...]
cells_to_world(path, origin, cell_size) # -> [(x, y), ...]
filter_sensors(raw_values)            # -> [float, ...]
```
