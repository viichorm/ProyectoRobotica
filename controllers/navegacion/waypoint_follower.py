"""
Seguimiento de waypoints mediante control proporcional diferencial.

El seguidor recibe la pose actual del robot (x, y, theta) en cada ciclo
y genera las velocidades angulares de cada rueda para avanzar hacia el
waypoint activo. Cuando la distancia al waypoint es menor que la tolerancia
configurada, avanza al siguiente.
"""

import math

from config import (
    ACCION_GIRAR_A_WAYPOINT,
    ACCION_META_ALCANZADA,
    ACCION_SEGUIR_RUTA,
    DISTANCIA_RUEDAS,
    RADIO_RUEDA,
    VELOCIDAD_MAX,
    WAYPOINT_GANANCIA_ANGULAR,
    WAYPOINT_GIRO_EN_SITIO,
    WAYPOINT_TOLERANCIA,
    WAYPOINT_VEL_LINEAL,
    limitar,
)


def _normalizar_angulo(a: float) -> float:
    """Lleva el angulo al intervalo (-pi, pi]."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class WaypointFollower:
    """
    Controlador proporcional para seguir una secuencia de waypoints.

    Comportamiento:
        - Si el error angular supera WAYPOINT_GIRO_EN_SITIO, el robot gira
          en el lugar (velocidad lineal = 0).
        - En caso contrario avanza a WAYPOINT_VEL_LINEAL con correccion
          angular proporcional.
        - Al llegar al ultimo waypoint emite ACCION_META_ALCANZADA y se
          detiene.
    """

    def __init__(self,
                 waypoints: list[tuple[float, float]],
                 tolerancia: float = WAYPOINT_TOLERANCIA,
                 vel_lineal: float = WAYPOINT_VEL_LINEAL,
                 ganancia_angular: float = WAYPOINT_GANANCIA_ANGULAR):
        self.waypoints       = waypoints
        self.tolerancia      = tolerancia
        self.vel_lineal      = vel_lineal
        self.ganancia_angular= ganancia_angular
        self.indice          = 0

    # ------------------------------------------------------------------

    def terminado(self) -> bool:
        return self.indice >= len(self.waypoints)

    def waypoint_actual(self) -> tuple[float, float] | None:
        if self.terminado():
            return None
        return self.waypoints[self.indice]

    def actualizar(self, x: float, y: float,
                   theta: float) -> tuple[float, float, str]:
        """
        Calcula las velocidades de rueda para el ciclo actual.

        Returns:
            (vel_izq, vel_der, accion)
        """
        if self.terminado():
            return 0.0, 0.0, ACCION_META_ALCANZADA

        # --- avanzar al waypoint siguiente si estamos cerca --------------
        while not self.terminado():
            obj_x, obj_y = self.waypoint_actual()
            dx = obj_x - x
            dy = obj_y - y
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > self.tolerancia:
                break
            self.indice += 1

        if self.terminado():
            return 0.0, 0.0, ACCION_META_ALCANZADA

        # --- error angular hacia el waypoint activo ----------------------
        obj_x, obj_y = self.waypoint_actual()
        dx = obj_x - x
        dy = obj_y - y

        angulo_objetivo  = math.atan2(dy, dx)
        error_angular    = _normalizar_angulo(angulo_objetivo - theta)

        # --- seleccion de accion -----------------------------------------
        if abs(error_angular) > WAYPOINT_GIRO_EN_SITIO:
            # Giro en el sitio: solo velocidad angular
            vel_lineal = 0.0
            accion     = ACCION_GIRAR_A_WAYPOINT
        else:
            vel_lineal = self.vel_lineal
            accion     = ACCION_SEGUIR_RUTA

        vel_angular = self.ganancia_angular * error_angular

        # --- cinematica inversa diferencial ------------------------------
        vel_izq = (vel_lineal - vel_angular * DISTANCIA_RUEDAS / 2.0) / RADIO_RUEDA
        vel_der = (vel_lineal + vel_angular * DISTANCIA_RUEDAS / 2.0) / RADIO_RUEDA

        vel_izq = limitar(vel_izq, -VELOCIDAD_MAX, VELOCIDAD_MAX)
        vel_der = limitar(vel_der, -VELOCIDAD_MAX, VELOCIDAD_MAX)

        return vel_izq, vel_der, accion