# ReplayBuffer.py

import numpy as np

class ReplayBuffer:
    def __init__(self, max_size, state_dim, action_dim):
        
        self.max_size = max_size    # Tamaño máximo del buffer (cuántas transiciones puede guardar)
        self.ptr = 0            # Puntero de inserción (posición actual en el buffer)
        self.size = 0      # Cantidad de elementos almacenados actualmente

        self.state = np.zeros((max_size, state_dim))
        self.action = np.zeros((max_size, action_dim))
        self.next_state = np.zeros((max_size, state_dim))
        self.reward = np.zeros((max_size, 1))
        self.done = np.zeros((max_size, 1))

    def add(self, state, action, reward, next_state, done):
        self.state[self.ptr] = state
        self.action[self.ptr] = action
        self.reward[self.ptr] = reward
        self.next_state[self.ptr] = next_state
        self.done[self.ptr] = done

        self.ptr = (self.ptr + 1) % self.max_size
        self.size = min(self.size + 1, self.max_size)
        
    # -------------------------------------------------------
    # Muestra aleatoriamente un lote (batch) de transiciones.
    # Este muestreo aleatorio ayuda a romper la correlación
    # entre muestras consecutivas del entorno.
    # -------------------------------------------------------

    def sample(self, batch_size):
        ind = np.random.randint(0, self.size, size=batch_size)
        return (
            self.state[ind],
            self.action[ind],
            self.reward[ind],
            self.next_state[ind],
            self.done[ind]
        )
