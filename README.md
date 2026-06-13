# ProyectoRobotica
Proyecto Robotica


"""
labone.py
─────────────────────────────────────────────────────────────────────────────
Proyecto Final – Navegación Autónoma con Planificación de Rutas (A*)
Curso  : ICI 4150 – Robótica y Sistemas Autónomos 2026-01
PUCV   – Línea A: Planificación de rutas sobre grilla de ocupación

Extiende el Laboratorio 2 (evasión reactiva + Kalman + odometría) con:
  · Grilla de ocupación 2D del entorno (definida manualmente)
  · Algoritmo A* con 8 direcciones para planificación inicio → meta
  · Suavizado de ruta por línea de visión (Bresenham)
  · Seguimiento de waypoints con control proporcional
  · Árbitro de prioridad: evasión > seguimiento
  · Registro en CSV: pose, sensores, ruta planificada
─────────────────────────────────────────────────────────────────────────────
"""