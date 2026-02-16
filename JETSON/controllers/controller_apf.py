# controller_apf.py

# ------------------------------------------------------------
# Este controlador calcula las velocidades lineal (v) y angular (w)
# a partir de un campo de fuerzas virtuales:
#  - Una fuerza de atracción hacia el objetivo (meta)
#  - Una fuerza de repulsión ante obstáculos cercanos
#
# Devuelve también una "confianza" (conf) entre [0, 1],
# que mide la seguridad de la navegación según la distancia
# al obstáculo más cercano.
# ------------------------------------------------------------

import math
import numpy as np

class APFController:
    def __init__(self, vmax=1.0, wmax=1.0, k_att=1.0, k_rep=50.0, influence_radius=2.0):
        self.vmax = vmax                  # velocidad lineal máxima
        self.wmax = wmax                  # velocidad angular máxima
        self.k_att = k_att                # ganancia atracción
        self.k_rep = k_rep                # ganancia repulsión
        self.influence_radius = influence_radius  # distancia de influencia de los obstáculos

    def plan(self, state):
        """
        state = [x, y, theta, v_prev, w_prev, goal_x, goal_y, obst1_x, obst1_y, obst2_x, obst2_y, ...]
        Devuelve v, w, confianza
        """
        # Posición y orientación
        x, y, theta = state[0], state[1], state[2]
        goal_x, goal_y = state[5], state[6]

        # Obstáculos como lista de tuplas
        obstacles = [(state[i], state[i+1]) for i in range(7, len(state), 2)
                     if not (state[i] == 0 and state[i+1] == 0)]

        # Vector de atracción
        diff = np.array([goal_x - x, goal_y - y])        # Dirección hacia la meta
        dist_goal = np.linalg.norm(diff)                 # Distancia al objetivo
        # Si la distancia es muy pequeña, evita división por cero
        F_att = np.zeros(2) if dist_goal < 1e-6 else self.k_att * diff / dist_goal

        # Vector de repulsión
        F_rep = np.zeros(2)
        for ox, oy in obstacles:
            diff_o = np.array([x - ox, y - oy])  # Vector desde el obstáculo al robot
            d = np.linalg.norm(diff_o)           # Distancia al obstáculo

            # Solo influye si el obstáculo está dentro del radio de influencia
            if d < self.influence_radius and d > 1e-6:
                F_rep += self.k_rep * (1.0/d - 1.0/self.influence_radius) * (1.0/(d**2)) * (diff_o/d)

        # Fuerza total
        F = F_att + F_rep

        # Ángulo deseado y error
        desired_angle = math.atan2(F[1], F[0])   # Dirección del vector resultante
        angle_error = math.atan2(math.sin(desired_angle - theta), math.cos(desired_angle - theta))

        # Velocidades
        v = self.vmax * (1.0 - min(abs(angle_error)/math.pi, 1.0))
        w = self.wmax * angle_error

        # Confianza según obstáculo más cercano
        # Depende de la distancia al obstáculo más cercano
        
        if obstacles:
            nearest = min([np.linalg.norm(np.array([x, y]) - np.array([ox, oy])) for ox, oy in obstacles])
        else:
            nearest = 1e6  # No hay obstáculos cerca

        if nearest < 0.5:
            conf = 0.1  # Muy cerca → riesgo alto
        elif nearest < 2.0:
            conf = 0.5  # Moderadamente cerca → precaución
        else:
            conf = 1.0  # Lejos de obstáculos → confianza total

        return float(v), float(w), float(conf)
