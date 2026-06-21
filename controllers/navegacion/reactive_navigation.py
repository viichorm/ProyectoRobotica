"""
Capa de navegacion reactiva basada en sensores de proximidad.

Tiene PRIORIDAD sobre el seguidor de waypoints: si detecta un obstaculo
cercano, toma control de los motores hasta que la via quede despejada.

Jerarquia de comportamientos (de mayor a menor prioridad):
    1. Escape activo  : giro en arco durante PASOS_ESCAPE ciclos.
    2. Salida de escape: avance recto durante PASOS_SALIDA ciclos.
    3. Centrado en pasillo: correccion lateral proporcional.
    4. Avance libre   : delega al WaypointFollower.
"""

from config import (
    ACCION_AVANZAR,
    ACCION_CENTRAR_DERECHA,
    ACCION_CENTRAR_IZQUIERDA,
    ACCION_ESCAPE_DERECHA,
    ACCION_ESCAPE_IZQUIERDA,
    ACCION_GIRAR_DERECHA,
    ACCION_GIRAR_IZQUIERDA,
    ACCION_SALIDA_ESCAPE,
    CORRECCION_MAX,
    GANANCIA_CENTRADO,
    PASOS_ESCAPE,
    PASOS_SALIDA,
    UMBRAL_FRONTAL,
    UMBRAL_LATERAL,
    UMBRAL_LATERAL_EXTREMO,
    VEL_AVANCE,
    VEL_GIRO,
    VELOCIDAD_MAX,
    ZONA_MUERTA_LATERAL,
    limitar,
)
from sensors import IDX_FRONTAL_DER, IDX_FRONTAL_IZQ, IDX_LATERAL_DER, IDX_LATERAL_IZQ


class NavegacionReactiva:
    """Decide velocidades de rueda usando reglas reactivas basadas en sensores."""

    def __init__(self):
        self._escape_pasos: int = 0
        self._escape_dir:   str | None = None
        self._salida_pasos: int = 0

    # ------------------------------------------------------------------
    # API publica
    # ------------------------------------------------------------------

    def requiere_intervencion(self, valores_ps: list, valor_decision: float) -> bool:
        """
        Devuelve True si la capa reactiva debe tomar el control.

        Se activa si:
            - Hay una maniobra de escape o salida en curso.
            - La senal frontal (filtrada/Kalman) supera el umbral.
            - Algun sensor frontal individual supera el umbral.
            - Un sensor lateral supera el umbral extremo.
        """
        if self._escape_pasos > 0 or self._salida_pasos > 0:
            return True

        f_izq = valores_ps[IDX_FRONTAL_IZQ]
        f_der = valores_ps[IDX_FRONTAL_DER]
        l_izq = valores_ps[IDX_LATERAL_IZQ]
        l_der = valores_ps[IDX_LATERAL_DER]

        return (
            valor_decision > UMBRAL_FRONTAL
            or f_izq > UMBRAL_FRONTAL
            or f_der > UMBRAL_FRONTAL
            or l_izq > UMBRAL_LATERAL_EXTREMO
            or l_der > UMBRAL_LATERAL_EXTREMO
        )

    def decidir_movimiento(self, valores_ps: list,
                           valor_decision: float) -> tuple[float, float, str]:
        """
        Calcula (vel_izq, vel_der, accion) usando solo los sensores.

        Debe llamarse unicamente cuando requiere_intervencion() == True.
        """
        f_izq = valores_ps[IDX_FRONTAL_IZQ]
        f_der = valores_ps[IDX_FRONTAL_DER]
        l_izq = valores_ps[IDX_LATERAL_IZQ]
        l_der = valores_ps[IDX_LATERAL_DER]

        # --- 1. Escape activo -------------------------------------------
        if self._escape_pasos > 0:
            self._escape_pasos -= 1
            if self._escape_pasos == 0:
                self._salida_pasos = PASOS_SALIDA
            if self._escape_dir == "derecha":
                return -0.5, -VEL_GIRO, ACCION_ESCAPE_DERECHA
            return -VEL_GIRO, -0.5, ACCION_ESCAPE_IZQUIERDA

        # --- 2. Salida de escape ----------------------------------------
        if self._salida_pasos > 0:
            self._salida_pasos -= 1
            return VEL_AVANCE, VEL_AVANCE, ACCION_SALIDA_ESCAPE

        # --- 3. Iniciar escape (obstaculo detectado) --------------------
        lateral_extremo  = l_izq > UMBRAL_LATERAL_EXTREMO or l_der > UMBRAL_LATERAL_EXTREMO
        frontal_peligroso= (valor_decision > UMBRAL_FRONTAL
                            or f_izq > UMBRAL_FRONTAL
                            or f_der > UMBRAL_FRONTAL)

        if lateral_extremo or frontal_peligroso:
            self._escape_pasos = PASOS_ESCAPE
            if l_izq > l_der:
                self._escape_dir = "derecha"
                return -0.5, -VEL_GIRO, ACCION_ESCAPE_DERECHA
            self._escape_dir = "izquierda"
            return -VEL_GIRO, -0.5, ACCION_ESCAPE_IZQUIERDA

        # --- 4. Obstaculo frontal moderado: girar ----------------------
        if valor_decision > UMBRAL_FRONTAL or f_izq > UMBRAL_FRONTAL or f_der > UMBRAL_FRONTAL:
            diferencia = l_izq - l_der
            if abs(diferencia) > ZONA_MUERTA_LATERAL:
                girar_derecha = l_izq > l_der
            else:
                girar_derecha = f_izq > f_der

            if girar_derecha:
                return VEL_GIRO, -VEL_GIRO, ACCION_GIRAR_DERECHA
            return -VEL_GIRO, VEL_GIRO, ACCION_GIRAR_IZQUIERDA

        # --- 5. Obstaculos laterales moderados: centrar en pasillo -----
        if l_izq > UMBRAL_LATERAL or l_der > UMBRAL_LATERAL:
            error      = l_izq - l_der
            correccion = limitar(GANANCIA_CENTRADO * error, -CORRECCION_MAX, CORRECCION_MAX)
            v_izq      = limitar(VEL_AVANCE + correccion, -VELOCIDAD_MAX, VELOCIDAD_MAX)
            v_der      = limitar(VEL_AVANCE - correccion, -VELOCIDAD_MAX, VELOCIDAD_MAX)
            accion     = ACCION_CENTRAR_DERECHA if error > 0 else ACCION_CENTRAR_IZQUIERDA
            return v_izq, v_der, accion

        # --- 6. Via despejada ------------------------------------------
        return VEL_AVANCE, VEL_AVANCE, ACCION_AVANZAR