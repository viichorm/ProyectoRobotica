"""
Registro de metricas por ciclo y exportacion a CSV al finalizar la simulacion.

El archivo se guarda en la carpeta 'results/' en la raiz del proyecto,
con el nombre:
    log_<DDMMAAAA-HHMMSS>_Mundo=<escenario>_Modo=<modo>.csv
"""

import csv
import os
from datetime import datetime

from config import negrita, separador


class MetricsLogger:
    """Acumula muestras en memoria y las vuelca a CSV al terminar."""

    def __init__(self):
        self._registros: list[dict] = []

    def agregar(self, registro: dict) -> None:
        """Agrega un registro (diccionario de una fila) al buffer."""
        self._registros.append(registro)

    def guardar_csv(self, robot, modo: str) -> str | None:
        """
        Guarda todos los registros en un archivo CSV.

        Args:
            robot: instancia Supervisor de Webots (para obtener el nombre
                   del mundo y construir la ruta de salida).
            modo:  etiqueta del modo de decision utilizado.

        Returns:
            Ruta absoluta del archivo generado, o None si no hay datos.
        """
        if not self._registros:
            print("[MetricsLogger] Sin registros que guardar.")
            return None

        # --- nombre del archivo ------------------------------------------
        ruta_mundo = robot.getWorldPath() or ""
        escenario  = os.path.splitext(os.path.basename(ruta_mundo))[0] or "desconocido"
        fecha_hora = datetime.now().strftime("%d%m%Y-%H%M%S")
        nombre     = f"log_{fecha_hora}_Mundo={escenario}_Modo={modo}.csv"

        # --- carpeta de resultados al nivel del repositorio --------------
        raiz = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
        carpeta = os.path.join(raiz, "results")
        os.makedirs(carpeta, exist_ok=True)
        ruta = os.path.join(carpeta, nombre)

        # --- escritura ---------------------------------------------------
        campos = list(self._registros[0].keys())
        with open(ruta, "w", newline="") as f:
            escritor = csv.DictWriter(f, fieldnames=campos)
            escritor.writeheader()
            escritor.writerows(self._registros)

        # --- resumen en consola ------------------------------------------
        separador()
        print(negrita(" SIMULACION FINALIZADA"))
        print(f" CSV guardado          : {ruta}")
        print(f" Escenario             : {escenario}")
        print(f" Muestras registradas  : {len(self._registros)}")
        print(f" Modo utilizado        : {modo}")
        separador()

        return ruta