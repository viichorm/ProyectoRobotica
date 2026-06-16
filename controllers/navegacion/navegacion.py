# Dario, cuando veas eso, los comentarios de abajo dejalos comentados, porque sino en consola te va a dejar la grande en el printeo xd
from controller import Robot
import cinematica as cin

# --- Cuando P2 tenga sus modulos listos, descomentar ---
# import planner
# import filtros
# from grid_simple import GRID, ORIGIN, CELL_SIZE

robot = Robot()
cin.init(robot)
timestep = int(robot.getBasicTimeStep())

print("[main] Iniciando loop principal...")

while robot.step(timestep) != -1:
    cin.update_odometry()
    x, y, theta = cin.get_pose()

    # --- Calibracion: muestra valores de sensores en consola ---
    # Descomentar para ver que valores arrojan los sensores cerca de paredes
    # vals = cin.get_sensor_values()
    # print(f"[sensores] {[round(v,1) for v in vals]}")

    if cin.is_obstacle_ahead():
        cin.reactive_avoid()
    else:
        # Aqui va la logica de navegacion global de P2
        # cin.set_velocity(0.1, 0.0)  # avanzar recto de prueba
        pass