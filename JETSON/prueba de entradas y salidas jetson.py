# main.py

import torch
from RL.SAC import SAC
from RL.TD3 import TD3
from controllers.controller_apf import APFController
from controllers.controller_dwa import DWAController
from controllers.controller_teb import TEBController


apf = APFController()
teb = TEBController()

# ---------------- Parámetros ----------------
state_dim = 2 + 1 + 2 + 2 + 20  # q(2) + teta(1) + a_h(2) + q_meta(2) + q_obst(10*2)
action_dim = 2  # (v, w)
max_action = [1.0, 3.14]  # ejemplo: velocidad máxima y velocidad angular máxima

# ---------------- Inicializar agentes ----------------
sac_agent = SAC(state_dim=state_dim, action_dim=action_dim, max_action=max_action)
td3_agent = TD3(state_dim=state_dim, action_dim=action_dim, max_action=max_action)

# ---------------- Valores manuales de estado ----------------
q = [0, 0]          # lat, lon actual
teta = [0.5]                     # orientación en radianes
a_h = [0.0, 0.0]                 # acción previa (v, w)
q_meta = [20, 15]     # meta lat, lon

# Obstáculos detectados (menos de 10)
q_obst = [
    (0.5, 0.2),
    (1.0, -0.5),
    (0.8, 0.3),
    (0.0, 0.0),
    (0.6, -0.2),
    (0.4, 0.1)  # sólo 6 obstáculos
]

# ---------------- Rellenar con ceros hasta 10 obstáculos ----------------
max_obst = 10
while len(q_obst) < max_obst:
    q_obst.append((0.0, 0.0))

# Aplanar lista de obstáculos
q_obst_flat = [coord for obs in q_obst for coord in obs]

# ---------------- Estado extendido completo ----------------
estado_extendido = q + teta + a_h + q_meta + q_obst_flat

# ---------------- Obtener acciones ----------------
a_p_sac = sac_agent.get_action(estado_extendido)
a_p_td3 = td3_agent.get_action(estado_extendido)

a_r_dwa = DWAController(estado_extendido)
a_r_apf = apf.plan(estado_extendido)
a_r_teb = teb.plan(estado_extendido)

# ---------------- Imprimir resultados ----------------
print("Acción SAC:", a_p_sac)
print("Acción TD3:", a_p_td3)
print("Acción DWA:", a_r_dwa)
print("Acción APF:", a_r_apf)
print("Acción TEB:", a_r_teb)
