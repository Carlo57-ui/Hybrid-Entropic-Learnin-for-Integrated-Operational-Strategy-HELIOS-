# controller_dwa.py

import numpy as np
import math


class Config:
    def __init__(self):
        # --- Límites físicos del robot ---
        self.max_speed = 1.0                         # Velocidad lineal máxima [m/s]
        self.min_speed = -1.0                        # Velocidad lineal mínima (puede ir en reversa)
        self.max_yaw_rate = 40.0 * math.pi / 180.0   # Velocidad angular máxima [rad/s]
        self.max_accel = 1.0                         # Aceleración máxima [m/s²]
        self.max_delta_yaw_rate = 40.0 * math.pi / 180.0  # Cambio máximo de giro por dt

        # --- Parámetros de predicción ---
        self.v_resolution = 0.01                     # Resolución de búsqueda para v
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # Resolución de búsqueda para w
        self.dt = 0.1                                # Paso de tiempo [s]
        self.predict_time = 2.0                      # Tiempo de predicción para cada trayectoria [s]

        # ---- Pesos de las funciones de costo ----
        self.to_goal_cost_gain = 0.35               # Peso para costo hacia la meta
        self.speed_cost_gain = 0.3                  # Peso para costo de velocidad
        self.obstacle_cost_gain = 0.3               # Peso para costo por obstáculos

        # ---- Dimensiones físicas del robot ----
        self.robot_radius = 2                       # Radio aproximado del robot [m]
        self.robot_width = 0.7                      # Ancho del robot [m]
        self.robot_length = 0.3                     # Largo del robot [m]

        # ---- Constante anti-trampa ----
        self.robot_stuck_flag_cons = 0.01           # Umbral para detectar si el robot se estanca


def DWAController(state):
    """
    Entrada:
        state = [x, y, teta, v, w, goal_x, goal_y, obs1_x, obs1_y, ...]
    Salida:
        v, w, conf
        donde:
            v: velocidad lineal óptima [m/s]
            w: velocidad angular óptima [rad/s]
            conf: nivel de confianza (1.0=seguro, 0.1=riesgo alto)
    """

    config = Config()

    # --- Extraer estado extendido ---
    x, y, yaw, v_prev, w_prev = state[:5]
    goal_x, goal_y = state[5:7]

    # --- Extraer lista de obstáculos ---
    obstacles = np.array([
        (state[i], state[i + 1]) for i in range(7, len(state), 2)
        if not (state[i] == 0 and state[i + 1] == 0)
    ])

    robot_state = np.array([x, y, yaw, v_prev, w_prev])


    def motion(x, u, dt):
        """Simula el movimiento del robot para un paso temporal dt."""
        x[2] += u[1] * dt                         # Actualizar ángulo (yaw o teta)
        x[0] += u[0] * math.cos(x[2]) * dt        # Actualizar posición X
        x[1] += u[0] * math.sin(x[2]) * dt        # Actualizar posición Y
        x[3] = u[0]                               # Velocidad lineal actual
        x[4] = u[1]                               # Velocidad angular actual
        return x

    def calc_dynamic_window(x, config):
        # Velocidades posibles según límites del robot
        Vs = [config.min_speed, config.max_speed,
              -config.max_yaw_rate, config.max_yaw_rate]

        # Velocidades posibles según dinámica instantánea
        Vd = [x[3] - config.max_accel * config.dt,
              x[3] + config.max_accel * config.dt,
              x[4] - config.max_delta_yaw_rate * config.dt,
              x[4] + config.max_delta_yaw_rate * config.dt]

        # Intersección de los dos rangos (física + dinámica)
        return [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
                max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    def predict_trajectory(x_init, v, y_rate):
        """
        Predice una trayectoria futura del robot aplicando las velocidades (v, y_rate)
        durante el tiempo de predicción configurado.
        """
        traj = [x_init.copy()]
        x = x_init.copy()
        time = 0
        while time <= config.predict_time:
            x = motion(x, [v, y_rate], config.dt)
            traj.append(x.copy())
            time += config.dt
        return np.array(traj)

    def calc_obstacle_cost(traj, ob):
        """
        Calcula el costo por proximidad a obstáculos.
        Penaliza trayectorias que se acerquen o colisionen.
        """
        if ob.size == 0:
            return 0
        dx = traj[:, 0][:, None] - ob[:, 0][None, :]
        dy = traj[:, 1][:, None] - ob[:, 1][None, :]
        r = np.hypot(dx, dy)                     # Distancia euclidiana
        if (r <= config.robot_radius).any():     # Colisión
            return float("Inf")
        return 1.0 / np.min(r)  

    def calc_to_goal_cost(traj):
        """
        Evalúa qué tan alineada está la trayectoria con la meta.
        Penaliza ángulos grandes entre la orientación final y la dirección a la meta.
        """
        dx = goal_x - traj[-1, 0]
        dy = goal_y - traj[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - traj[-1, 2]
        return abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

    def calc_control_and_trajectory(x, dw, ob):
        """
        Evalúa todas las combinaciones posibles (v, w) dentro de la ventana dinámica
        y elige aquella que minimiza el costo total.
        """
        min_cost = float("inf")
        best_u = [0.0, 0.0]

        for v in np.arange(dw[0], dw[1] + 1e-5, config.v_resolution):
            for y_rate in np.arange(dw[2], dw[3] + 1e-5, config.yaw_rate_resolution):
                traj = predict_trajectory(x, v, y_rate)
                cost = (
                    config.to_goal_cost_gain * calc_to_goal_cost(traj) +
                    config.speed_cost_gain * (config.max_speed - traj[-1, 3]) +
                    config.obstacle_cost_gain * calc_obstacle_cost(traj, ob)
                )
                if cost < min_cost:
                    min_cost = cost
                    best_u = [v, y_rate]
        return best_u


    # Calcular la ventana dinámica actual
    dw = calc_dynamic_window(robot_state, config)

    # Buscar las velocidades óptimas (v, w)
    u = calc_control_and_trajectory(robot_state, dw, obstacles)


    if obstacles.size > 0:
        nearest = np.min(np.hypot(obstacles[:, 0] - x, obstacles[:, 1] - y))
    else:
        nearest = 1e6

    if nearest < 0.5:
        conf = 0.1          # Muy cerca de obstáculo
    elif nearest < 2.0:
        conf = 0.5          # Distancia moderada
    else:
        conf = 1.0          # Espacio libre


    v = np.clip(u[0], config.min_speed, config.max_speed)
    w = np.clip(u[1], -config.max_yaw_rate, config.max_yaw_rate)

    return v, w, conf


    
    
    