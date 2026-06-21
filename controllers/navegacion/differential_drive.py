"""
Inicializacion y control de los motores diferenciales del e-puck.
"""

from config import TIME_STEP


def inicializar_motores(robot):
    """
    Configura los dos motores en modo velocidad y los devuelve.
    Posicion infinita = modo velocidad continua.
    """
    motor_izq = robot.getDevice("left wheel motor")
    motor_der = robot.getDevice("right wheel motor")

    motor_izq.setPosition(float("inf"))
    motor_der.setPosition(float("inf"))
    motor_izq.setVelocity(0.0)
    motor_der.setVelocity(0.0)

    return motor_izq, motor_der


def inicializar_encoders(robot):
    """Habilita los encoders de posicion angular y los devuelve."""
    enc_izq = robot.getDevice("left wheel sensor")
    enc_der = robot.getDevice("right wheel sensor")

    enc_izq.enable(TIME_STEP)
    enc_der.enable(TIME_STEP)

    return enc_izq, enc_der


def aplicar_velocidades(motor_izq, motor_der, vel_izq, vel_der):
    """Escribe las velocidades angulares (rad/s) en los motores."""
    motor_izq.setVelocity(vel_izq)
    motor_der.setVelocity(vel_der)