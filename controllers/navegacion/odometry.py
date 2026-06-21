"""
Odometria diferencial calculada a partir de los encoders del e-puck.

Modelo cinematico:
    ds    = (d_der + d_izq) / 2
    dtheta = (d_der - d_izq) / L
    x     += ds * cos(theta + dtheta/2)
    y     += ds * sin(theta + dtheta/2)
    theta += dtheta

donde d_izq, d_der son los arcos recorridos por cada rueda en el ciclo actual.
"""

import math

from config import DISTANCIA_RUEDAS, RADIO_RUEDA, TS


class OdometriaDiferencial:
    """Mantiene la pose odometrica (x, y, theta) y la actualiza cada ciclo."""

    def __init__(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        self.x = x
        self.y = y
        self.theta = self._normalizar(theta)

        self._enc_izq_ant = None
        self._enc_der_ant = None

    # ------------------------------------------------------------------
    # API publica
    # ------------------------------------------------------------------

    def establecer_pose(self, x: float, y: float, theta: float) -> None:
        """Sobreescribe la pose actual (usada para correccion por Supervisor)."""
        self.x = x
        self.y = y
        self.theta = self._normalizar(theta)

    def actualizar(self, enc_izq: float, enc_der: float):
        """
        Actualiza la pose a partir de las lecturas de encoder.

        Returns:
            ds (float): desplazamiento lineal del ciclo [m].
            v  (float): velocidad lineal estimada [m/s].
            w  (float): velocidad angular estimada [rad/s].
        """
        if self._enc_izq_ant is None:
            self._enc_izq_ant = enc_izq
            self._enc_der_ant = enc_der
            return 0.0, 0.0, 0.0

        d_izq = RADIO_RUEDA * (enc_izq - self._enc_izq_ant)
        d_der = RADIO_RUEDA * (enc_der - self._enc_der_ant)

        self._enc_izq_ant = enc_izq
        self._enc_der_ant = enc_der

        ds     = (d_izq + d_der) / 2.0
        dtheta = (d_der - d_izq) / DISTANCIA_RUEDAS

        # Integracion con angulo promedio del ciclo (mas preciso)
        theta_mid = self.theta + dtheta / 2.0
        self.x    += ds * math.cos(theta_mid)
        self.y    += ds * math.sin(theta_mid)
        self.theta = self._normalizar(self.theta + dtheta)

        return ds, ds / TS, dtheta / TS

    # ------------------------------------------------------------------
    # Utilidades internas
    # ------------------------------------------------------------------

    @staticmethod
    def _normalizar(angulo: float) -> float:
        """Lleva el angulo al intervalo (-pi, pi]."""
        return (angulo + math.pi) % (2.0 * math.pi) - math.pi