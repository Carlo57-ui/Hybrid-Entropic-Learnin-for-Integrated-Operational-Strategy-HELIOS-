# controller_teb.py

# Este controlador genera una trayectoria entre el punto inicial y la meta,
# optimizándola mediante fuerzas de suavizado y repulsión de obstáculos.
# Devuelve velocidades lineal (v), angular (w) y una medida de confianza (conf).

import math
import numpy as np

class TEBController:
    """
    Controlador TEB simplificado
    Interfaz: v, w, conf = plan(state)
    """

    def __init__(self, vmax=0.5, wmax=1.0, N=20, k_obs=5.0, k_smooth=1.0, d_safe=1.0, iters=10, dt=0.1):
        self.vmax = vmax              # Velocidad lineal máxima [m/s]
        self.wmax = wmax              # Velocidad angular máxima [rad/s]
        self.N = N                    # Número de puntos de la banda elástica
        self.k_obs = k_obs            # Ganancia de repulsión frente a obstáculos
        self.k_smooth = k_smooth      # Ganancia de suavizado (mantiene continuidad en la trayectoria)
        self.d_safe = d_safe          # Distancia mínima segura a obstáculos
        self.iters = iters            # Iteraciones del proceso de optimización
        self.dt = dt                  # Paso de tiempo entre puntos consecutivos de la banda

    def generate_initial_band(self, start, goal):
        """
        Genera una trayectoria inicial (banda) lineal desde la posición inicial hasta la meta.
        Devuelve una matriz con N puntos, cada uno con (x, y, t).
        """
        xs = np.linspace(start[0], goal[0], self.N)   # Puntos equiespaciados en X
        ys = np.linspace(start[1], goal[1], self.N)   # Puntos equiespaciados en Y
        ts = np.linspace(0, self.N * self.dt, self.N) # Tiempo asociado a cada punto
        return np.vstack([xs, ys, ts]).T              # Banda con forma (N, 3)

    def optimize_band(self, band, obstacles):
        """
        Ajusta la banda inicial aplicando dos tipos de fuerzas:
        - Suavizado: mantiene una distribución homogénea entre los puntos.
        - Repulsión: evita que la banda se acerque demasiado a los obstáculos.
        """
        
        for _ in range(self.iters):
            for i in range(1, len(band)-1):
                xi, yi, _ = band[i]

                # Fuerza de suavizado: tiende a promediar entre los puntos adyacentes
                prev = band[i - 1][:2]
                nxt = band[i + 1][:2]
                smooth = (prev + nxt - 2 * np.array([xi, yi])) * self.k_smooth

                # Fuerza de repulsión: empuja la trayectoria lejos de los obstáculos

                rep = np.zeros(2)
                for ox, oy in obstacles:
                    diff = np.array([xi - ox, yi - oy])
                    dist = np.linalg.norm(diff)
                    if dist < self.d_safe and dist > 1e-6:
                        rep += self.k_obs * (1.0/dist - 1.0/self.d_safe) * diff / dist**2

                band[i,0:2] += 0.1 * (smooth + rep)
        return band

    def plan(self, state):
        """
        Función principal del controlador.
        Recibe el estado actual y devuelve:
        - v: velocidad lineal
        - w: velocidad angular
        - conf: nivel de confianza según proximidad a obstáculos

        state = [x, y, theta, v_prev, w_prev, goal_x, goal_y, obst1_x, obst1_y, obst2_x, obst2_y, ...]
        """
        
        # Extraer posición y meta
        start = np.array(state[0:2])
        goal = np.array(state[5:7])
        obstacles = [(state[i], state[i+1]) for i in range(7, len(state), 2)
                     if not (state[i]==0 and state[i+1]==0)]

        # Generar y optimizar la banda
        band = self.generate_initial_band(start, goal)
        band = self.optimize_band(band, obstacles)

        # Velocidad lineal proporcional a la distancia al siguiente punto
        next_point = band[1][:2]
        vec = next_point - start
        dist = np.linalg.norm(vec)
        v = min(self.vmax, dist/self.dt if dist > 1e-6 else 0.0)

        # Velocidad angular hacia el siguiente punto
        desired_angle = math.atan2(vec[1], vec[0])
        theta = state[2]
        angle_error = math.atan2(math.sin(desired_angle - theta), math.cos(desired_angle - theta))
        w = max(-self.wmax, min(self.wmax, angle_error/self.dt))

        # Estimar confianza según distancia al obstáculo más cercano
        if obstacles:
            nearest = min([np.linalg.norm(np.array([state[0], state[1]]) - np.array([ox, oy]))
                           for ox, oy in obstacles])
        else:
            nearest = 1e6  # Sin obstáculos cercanos

        # Regla heurística de confianza
        if nearest < 0.5:
            conf = 0.1     # Muy cerca del obstáculo → baja confianza
        elif nearest < 2.0:
            conf = 0.5     # Moderadamente cerca → confianza media
        else:
            conf = 1.0     # Lejos de obstáculos → alta confianza

        return float(v), float(w), float(conf)
