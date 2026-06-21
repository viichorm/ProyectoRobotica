"""
Controlador principal para navegacion en Webots.
Nombre del controlador: navegacion
"""

import math
import os
import sys

from controller import Supervisor

from astar import astar
from config import (
    ACCION_GIRAR_A_WAYPOINT,
    ACCION_META_ALCANZADA,
    ACCION_RECUPERAR_GIRAR,
    ACCION_RECUPERAR_RETROCEDER,
    ACCIONES,
    BLOQUEO_AVANCE_MIN,
    BLOQUEO_CICLOS,
    ESCALA_SENSOR,
    FS,
    MODO_DECISION,
    ORIGEN_WEBOTS,
    Q_KALMAN,
    R_KALMAN,
    RECUPERACION_GIRO_PASOS,
    RECUPERACION_RETROCESO_PASOS,
    RECUPERACION_VEL_GIRO,
    RECUPERACION_VEL_RETROCESO,
    TAMANO_CELDA,
    THETA_INICIAL,
    TIME_STEP,
    TS,
    UMBRAL_FRONTAL,
    UMBRAL_LATERAL,
    VENTANA_MEDIA_MOVIL,
    negrita,
    separador,
)
from differential_drive import aplicar_velocidades, inicializar_encoders, inicializar_motores
from filters import FiltroKalman1D, FiltroMediaMovil
from metrics_logger import MetricsLogger
from occupancy_grid import OccupancyGrid
from odometry import OdometriaDiferencial
from reactive_navigation import NavegacionReactiva
from sensors import (
    IDX_LATERAL_DER,
    IDX_LATERAL_IZQ,
    inicializar_sensores_proximidad,
    leer_sensores_proximidad,
    obtener_senal_frontal,
)
from waypoint_follower import WaypointFollower


class ControladorNavegacion:
    _MODOS_VALIDOS = {
        "raw":      "crudo",
        "crudo":    "crudo",
        "filtered": "filtrado",
        "filtrado": "filtrado",
        "kalman":   "kalman",
    }

    def __init__(self, archivo_grilla="facil_grid.csv", tipo_movimiento="4", 
                 tamano_celda=TAMANO_CELDA, origen_webots=ORIGEN_WEBOTS):
        self.robot = Supervisor()
        
        self.archivo_grilla = archivo_grilla
        self.tipo_movimiento = tipo_movimiento
        self.tamano_celda = tamano_celda
        self.origen_webots = origen_webots

        self.motor_izq, self.motor_der = inicializar_motores(self.robot)
        self.enc_izq,   self.enc_der   = inicializar_encoders(self.robot)
        self.sensores_ps               = inicializar_sensores_proximidad(self.robot)

        self.filtro_media  = FiltroMediaMovil(VENTANA_MEDIA_MOVIL)
        self.filtro_kalman = FiltroKalman1D(Q_KALMAN, R_KALMAN)
        self.odometria     = OdometriaDiferencial()

        self.navegacion = NavegacionReactiva()
        self.logger     = MetricsLogger()

        self.muestra    = 0
        self.modo       = self._leer_modo()

        self.grid      = None
        self.ruta      = []
        self.waypoints = []
        self.seguidor  = None

        self.bloqueo_contador      = 0
        self.recuperacion_retroceso= 0
        self.recuperacion_giro     = 0

        self.usando_supervisor     = False
        self.ultima_pose_supervisor= None

        self._inicializar_navegacion()
        self._imprimir_encabezado()

    def _leer_modo(self) -> str:
        clave = MODO_DECISION.strip().lower()
        if clave not in self._MODOS_VALIDOS:
            print(f"[ADVERTENCIA] Modo invalido '{MODO_DECISION}'. Se usara 'kalman'.")
            return "kalman"
        return self._MODOS_VALIDOS[clave]

    def _yaw_desde_orientacion(self, ori) -> float:
        return math.atan2(ori[3], ori[0])

    def _leer_pose_supervisor(self):
        try:
            nodo     = self.robot.getSelf()
            pos      = nodo.getPosition()
            ori      = nodo.getOrientation()
            theta    = self._yaw_desde_orientacion(ori)
            return pos[0], pos[1], theta
        except Exception:
            return None

    def _sincronizar_supervisor(self):
        pose = self._leer_pose_supervisor()
        if pose is None:
            return None

        avance = None
        if self.ultima_pose_supervisor is not None:
            dx = pose[0] - self.ultima_pose_supervisor[0]
            dy = pose[1] - self.ultima_pose_supervisor[1]
            avance = math.sqrt(dx * dx + dy * dy)

        self.ultima_pose_supervisor = pose
        self.usando_supervisor      = True
        self.odometria.establecer_pose(*pose)
        return avance

    def _celda_libre_cercana(self, x: float, y: float):
        celda_ini = self.grid.mundo_a_celda(x, y)

        if self.grid.es_libre(celda_ini):
            return celda_ini

        visitados = {celda_ini}
        cola      = [celda_ini]
        cabeza    = 0

        while cabeza < len(cola) and len(cola) < 50:
            fr, fc = cola[cabeza]
            cabeza += 1
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                vecino = (fr + dr, fc + dc)
                if vecino in visitados:
                    continue
                if not self.grid.dentro_de_limites(vecino):
                    continue
                visitados.add(vecino)
                if self.grid.es_libre(vecino):
                    return vecino
                cola.append(vecino)

        print("[ADVERTENCIA] No se encontro celda libre cercana. Usando inicio del CSV.")
        return self.grid.inicio

    def _inicializar_navegacion(self) -> None:
        dir_controlador = os.path.dirname(os.path.abspath(__file__))
        ruta_csv = os.path.join(dir_controlador, self.archivo_grilla)

        try:
            self.grid = OccupancyGrid(
                ruta_csv,
                tamano_celda=self.tamano_celda,
                origen_webots=self.origen_webots,
            )

            pose = self._leer_pose_supervisor()
            if pose is None:
                x_ini, y_ini = self.grid.celda_a_mundo(self.grid.inicio)
                pose = (x_ini, y_ini, THETA_INICIAL)
                celda_inicio = self.grid.inicio
            else:
                self.usando_supervisor      = True
                self.ultima_pose_supervisor = pose
                celda_inicio = self._celda_libre_cercana(pose[0], pose[1])

            self.odometria.establecer_pose(*pose)

            self.ruta = astar(
                self.grid,
                celda_inicio,
                self.grid.meta,
                tipo_movimiento=self.tipo_movimiento,
            )

            if not self.ruta:
                print("[ADVERTENCIA] A* no encontro ruta. Solo navegacion reactiva.")
                return

            self.waypoints = self.grid.path_to_waypoints(self.ruta)
            self.seguidor  = WaypointFollower(self.waypoints)

        except Exception as exc:
            print(f"[ADVERTENCIA] Error al inicializar navegacion global: {exc}")
            print("[ADVERTENCIA] Se usara solo navegacion reactiva.")
            self.grid = self.ruta = self.waypoints = None
            self.seguidor = None

    def _imprimir_encabezado(self) -> None:
        separador()
        print(negrita(f" CONTROLADOR NAVEGACION - Archivo: {self.archivo_grilla}"))
        separador()
        print(f" Modo de decision      : {self.modo}")
        print(f" Tiempo de muestreo Ts : {TS:.3f} s  ({FS:.2f} Hz)")
        print(f" Umbral frontal        : {UMBRAL_FRONTAL}")
        print(f" Umbral lateral        : {UMBRAL_LATERAL}")
        print(f" Media movil           : ventana = {VENTANA_MEDIA_MOVIL}")
        print(f" Kalman                : Q={Q_KALMAN}, R={R_KALMAN}")
        print(f" Movimiento A* : {self.tipo_movimiento} direcciones")

        if self.grid is not None and self.ruta:
            print(f" Grilla                : {self.grid.filas}x{self.grid.columnas} "
                  f"({self.tamano_celda} m/celda)")
            print(f" Pose fuente           : "
                  f"{'Supervisor Webots' if self.usando_supervisor else 'odometria encoders'}")
            print(f" Inicio grilla         : {self.grid.inicio}")
            print(f" Meta grilla           : {self.grid.meta}")
            print(f" Celdas ruta A* : {len(self.ruta)}")
            print(f" Waypoints             : {len(self.waypoints)}")
            longitud = self.grid.calcular_longitud_ruta(self.ruta)
            print(f" Longitud estimada     : {longitud:.2f} m")
        else:
            print(" Planificador A* : desactivado (solo reactivo)")

        separador()
        print(negrita(" INICIANDO EJECUCION AUTONOMA"))
        separador()
        print()

    def _valor_decision(self, crudo: float, filtrado: float, kalman: float) -> float:
        if self.modo == "crudo":
            return crudo
        if self.modo == "filtrado":
            return filtrado
        return kalman

    def _dist_waypoint_actual(self) -> float | None:
        if self.seguidor is None or self.seguidor.terminado():
            return None
        ox, oy = self.seguidor.waypoint_actual()
        dx = ox - self.odometria.x
        dy = oy - self.odometria.y
        return math.sqrt(dx * dx + dy * dy)

    def _recuperacion_activa(self) -> bool:
        return self.recuperacion_retroceso > 0 or self.recuperacion_giro > 0

    def _iniciar_recuperacion(self) -> None:
        self.recuperacion_retroceso = RECUPERACION_RETROCESO_PASOS
        self.recuperacion_giro      = RECUPERACION_GIRO_PASOS
        self.bloqueo_contador       = 0
        print("[RECUPERACION] Robot bloqueado. Iniciando maniobra de recuperacion.")

    def _ejecutar_recuperacion(self):
        if self.recuperacion_retroceso > 0:
            self.recuperacion_retroceso -= 1
            return (RECUPERACION_VEL_RETROCESO,
                    RECUPERACION_VEL_RETROCESO,
                    ACCION_RECUPERAR_RETROCEDER)

        if self.recuperacion_giro > 0:
            self.recuperacion_giro -= 1
            return (-RECUPERACION_VEL_GIRO,
                     RECUPERACION_VEL_GIRO,
                     ACCION_RECUPERAR_GIRAR)

        return None

    def _actualizar_bloqueo(self, avance: float, accion: str,
                             vel_izq: float, vel_der: float) -> None:
        if accion in (ACCION_GIRAR_A_WAYPOINT, ACCION_META_ALCANZADA,
                      ACCION_RECUPERAR_RETROCEDER, ACCION_RECUPERAR_GIRAR):
            self.bloqueo_contador = 0
            return

        intentando_avanzar = vel_izq > 0.5 and vel_der > 0.5

        if intentando_avanzar and avance < BLOQUEO_AVANCE_MIN:
            self.bloqueo_contador += 1
        else:
            self.bloqueo_contador = 0

        if self.bloqueo_contador >= BLOQUEO_CICLOS and not self._recuperacion_activa():
            self._iniciar_recuperacion()

    def _seleccionar_movimiento(self, valores_ps: list, valor_decision: float) -> tuple[float, float, str]:
        if self.navegacion.requiere_intervencion(valores_ps, valor_decision):
            return self.navegacion.decidir_movimiento(valores_ps, valor_decision)

        recuperacion = self._ejecutar_recuperacion()
        if recuperacion is not None:
            return recuperacion

        if self.seguidor is not None:
            return self.seguidor.actualizar(
                self.odometria.x,
                self.odometria.y,
                self.odometria.theta,
            )

        return self.navegacion.decidir_movimiento(valores_ps, valor_decision)

    def _imprimir_estado(self, tiempo: float, f_crudo: float, f_filt: float, f_kal: float,
                         valores_ps: list, v_lin: float, v_ang: float, dist_wp, accion: str) -> None:
        print(f"* [{negrita('t')}: {tiempo:.2f}s]  [{negrita('muestra')}: {self.muestra}]")
        print(f"  {negrita('Frontal')} : raw={f_crudo:.1f}  filt={f_filt:.1f}  kal={f_kal:.1f}  K={self.filtro_kalman.k:.3f}")
        print(f"  {negrita('Lateral')} : izq={valores_ps[IDX_LATERAL_IZQ]:.1f}  der={valores_ps[IDX_LATERAL_DER]:.1f}")
        print(f"  {negrita('Odometria')}: x={self.odometria.x:.3f}  y={self.odometria.y:.3f}  th={self.odometria.theta:.3f} rad")
        print(f"  {negrita('Velocidad')}: v={v_lin:.3f} m/s  w={v_ang:.3f} rad/s")

        if self.seguidor is not None:
            wp_total = len(self.waypoints)
            wp_idx   = min(self.seguidor.indice, wp_total)
            print(f"  {negrita('Waypoint')} : {wp_idx}/{wp_total}  dist={dist_wp:.3f} m" if dist_wp is not None else f"  {negrita('Waypoint')} : {wp_idx}/{wp_total}")

        print(f"  {negrita('Accion')}   : {negrita(accion)}\n")

    def ejecutar(self) -> None:
        while self.robot.step(TIME_STEP) != -1:
            self.muestra += 1
            tiempo = self.muestra * TS

            valores_ps = leer_sensores_proximidad(self.sensores_ps)
            enc_izq_v  = self.enc_izq.getValue()
            enc_der_v  = self.enc_der.getValue()

            ds, v_lin, v_ang = self.odometria.actualizar(enc_izq_v, enc_der_v)
            avance_sup = self._sincronizar_supervisor()
            avance_bloqueo = avance_sup if avance_sup is not None else abs(ds)

            f_crudo   = obtener_senal_frontal(valores_ps)
            f_filtrado = self.filtro_media.actualizar(f_crudo)
            f_kalman  = self.filtro_kalman.actualizar(ds * ESCALA_SENSOR, f_crudo)
            valor_dec  = self._valor_decision(f_crudo, f_filtrado, f_kalman)

            dist_wp = self._dist_waypoint_actual()
            vel_izq, vel_der, accion = self._seleccionar_movimiento(valores_ps, valor_dec)
            aplicar_velocidades(self.motor_izq, self.motor_der, vel_izq, vel_der)

            self._actualizar_bloqueo(avance_bloqueo, accion, vel_izq, vel_der)

            celda_actual = None
            if self.grid is not None:
                celda_actual = self.grid.mundo_a_celda(self.odometria.x, self.odometria.y)

            if self.muestra % 20 == 0:
                self._imprimir_estado(tiempo, f_crudo, f_filtrado, f_kalman, valores_ps, v_lin, v_ang, dist_wp, accion)

            self.logger.agregar({
                "t_s":                    round(tiempo, 4),
                "muestra":                self.muestra,
                "modo_decision":          self.modo,
                "pose_fuente":            "supervisor" if self.usando_supervisor else "encoders",
                "ruta_celdas":            len(self.ruta) if self.ruta else 0,
                "waypoints_total":        len(self.waypoints) if self.waypoints else 0,
                "waypoint_indice":        self.seguidor.indice if self.seguidor else -1,
                "distancia_waypoint_m":   round(dist_wp, 5) if dist_wp is not None else "",
                "grid_fila":              celda_actual[0] if celda_actual else "",
                "grid_columna":           celda_actual[1] if celda_actual else "",
                "ps0_frontal_der":        round(valores_ps[0], 3),
                "ps7_frontal_izq":        round(valores_ps[7], 3),
                "ps2_lateral_der":        round(valores_ps[2], 3),
                "ps5_lateral_izq":        round(valores_ps[5], 3),
                "frontal_crudo":          round(f_crudo, 3),
                "frontal_filtrado":       round(f_filtrado, 3),
                "frontal_kalman":         round(f_kalman, 3),
                "kalman_ganancia":        round(self.filtro_kalman.k, 5),
                "kalman_p":               round(self.filtro_kalman.p, 5),
                "encoder_izq_rad":        round(enc_izq_v, 5),
                "encoder_der_rad":        round(enc_der_v, 5),
                "avance_ds_m":            round(ds, 6),
                "velocidad_lineal_m_s":   round(v_lin, 5),
                "velocidad_angular_rad_s":round(v_ang, 5),
                "odom_x_m":               round(self.odometria.x, 5),
                "odom_y_m":               round(self.odometria.y, 5),
                "odom_theta_rad":         round(self.odometria.theta, 5),
                "valor_decision":         round(valor_dec, 3),
                "vel_motor_izq_rad_s":    round(vel_izq, 3),
                "vel_motor_der_rad_s":    round(vel_der, 3),
                "accion":                 accion,
                "bloqueo_contador":       self.bloqueo_contador,
            })

            if accion == ACCION_META_ALCANZADA:
                print()
                separador()
                print(negrita(" META ALCANZADA"))
                print(f" Tiempo total          : {tiempo:.2f} s")
                if self.grid:
                    lx = self.odometria.x
                    ly = self.odometria.y
                    mx, my = self.grid.celda_a_mundo(self.grid.meta)
                    error = math.sqrt((lx - mx) ** 2 + (ly - my) ** 2)
                    print(f" Error final de posicion: {error:.4f} m")
                separador()
                break

        self.logger.guardar_csv(self.robot, self.modo)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "dificil":
        ControladorNavegacion(
            archivo_grilla="dificil_grid.csv", 
            tipo_movimiento="8", 
            tamano_celda=0.1, 
            origen_webots=(-1.35, 1.35)
        ).ejecutar()
    else:
        ControladorNavegacion(
            archivo_grilla="facil_grid.csv", 
            tipo_movimiento="4", 
            tamano_celda=TAMANO_CELDA, 
            origen_webots=ORIGEN_WEBOTS
        ).ejecutar()