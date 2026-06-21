"""
Filtros de senal para suavizar y fusionar la lectura frontal del e-puck.

FiltroMediaMovil  : promedio de las ultimas N muestras.
FiltroKalman1D    : Kalman escalar que fusiona prediccion por encoders
                    con medicion del sensor frontal.
"""


class FiltroMediaMovil:
    """Promedio deslizante sobre una ventana de N muestras."""

    def __init__(self, ventana: int):
        self.ventana = ventana
        self._buffer: list[float] = []

    def actualizar(self, valor: float) -> float:
        self._buffer.append(valor)
        if len(self._buffer) > self.ventana:
            self._buffer.pop(0)
        return sum(self._buffer) / len(self._buffer)


class FiltroKalman1D:
    """
    Filtro de Kalman escalar para estimar la proximidad frontal.

    Estado: distancia estimada al obstaculo frontal (en unidades del sensor).

    Prediccion:
        d_pred = d_hat + delta_encoder * escala
        P_pred = P + Q

    Correccion:
        K      = P_pred / (P_pred + R)
        d_hat  = d_pred + K * (medicion - d_pred)
        P      = (1 - K) * P_pred
    """

    def __init__(self, q: float, r: float):
        self.q = q          # covarianza del ruido de proceso
        self.r = r          # covarianza del ruido de medicion

        self.d_hat:  float | None = None   # estimacion actual
        self.p:      float = 1.0           # covarianza del error
        self.k:      float = 0.0           # ganancia de Kalman (ultimo ciclo)
        self.d_pred: float = 0.0
        self.p_pred: float = 0.0

    # ------------------------------------------------------------------
    def inicializado(self) -> bool:
        return self.d_hat is not None

    def _inicializar(self, primera_medicion: float) -> None:
        self.d_hat  = primera_medicion
        self.p      = 1.0
        self.k      = 0.0
        self.d_pred = primera_medicion
        self.p_pred = self.p

    def _predecir(self, delta_encoder: float) -> None:
        self.d_pred = self.d_hat + delta_encoder
        self.p_pred = self.p + self.q

    def _corregir(self, medicion: float) -> float:
        self.k      = self.p_pred / (self.p_pred + self.r)
        self.d_hat  = self.d_pred + self.k * (medicion - self.d_pred)
        self.p      = (1.0 - self.k) * self.p_pred
        return self.d_hat

    def actualizar(self, delta_encoder: float, medicion: float) -> float:
        """
        Realiza un ciclo prediccion-correccion.

        Args:
            delta_encoder: avance lineal del ciclo [m] multiplicado por la
                           escala del sensor (config.ESCALA_SENSOR).
            medicion:      valor crudo del sensor frontal.

        Returns:
            Estimacion filtrada de la proximidad frontal.
        """
        if not self.inicializado():
            self._inicializar(medicion)
            return self.d_hat

        self._predecir(delta_encoder)
        return self._corregir(medicion)