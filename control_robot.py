import sim
import time
import math
import numpy as np
import sys

# --- Parâmetros DWA e Configurações do Robô ---
# Esta classe define o comportamento do robô.
# Os valores foram ajustados com base nos parâmetros do seu script Lua original.
class Config:
    def __init__(self):
        # Parâmetros Físicos do Robô
        self.max_speed = 0.5          # [m/s] Velocidade linear máxima
        self.min_speed = -0.2         # [m/s] Velocidade de ré
        self.max_yawrate = 50.0 * math.pi / 180.0  # [rad/s] Velocidade de rotação máxima
        self.max_accel = 1.0          # [m/ss] Aceleração linear máxima
        self.max_dyawrate = 70.0 * math.pi / 180.0 # [rad/ss] Aceleração de rotação máxima
        self.robot_radius = 0.45      # [m] "Bolha de segurança" do robô

        # Parâmetros Físicos das Rodas (para controlo de motores)
        self.wheel_base = 0.5         # [m] Distância entre as rodas
        self.wheel_radius = 0.076     # [m] Raio da roda

        # Parâmetros do Algoritmo DWA
        self.dt = 0.1                 # [s] Passo de tempo da simulação
        self.predict_time = 2.0       # [s] Horizonte de previsão de trajetórias
        self.v_reso = 0.02            # [m/s] Resolução da amostragem de velocidade
        self.yawrate_reso = 0.2 * math.pi / 180.0 # [rad/s] Resolução da amostragem de rotação

        # Pesos da Função de Custo (A "Personalidade" do Robô)
        self.to_goal_cost_gain = 0.5  # Importância de chegar ao objetivo
        self.speed_cost_gain = 0.1    # Importância de manter a velocidade
        self.obstacle_cost_gain = 10.0 # Importância de desviar de obstáculos

        # Parâmetros dos Sensores (Baseado no seu script Lua: noDetectionDist=0.5)
        self.sensor_max_range = 0.5   # [m] Alcance máximo dos sensores
        
        # Ângulos dos 16 sensores (essencial para localizar obstáculos)
        self.sensor_angles_rad = np.array([
            -90.0, -75.0, -60.0, -45.0, -30.0, -15.0, 0.0, 15.0,
            15.0, 30.0, 45.0, 60.0, 75.0, 90.0, 135.0, -135.0
        ]) * math.pi / 180.0

# --- Funções do Algoritmo DWA ---

def motion(state, v, w, dt):
    """Simula o movimento do robô para uma trajetória."""
    state[0] += v * math.cos(state[2]) * dt
    state[1] += v * math.sin(state[2]) * dt
    state[2] += w * dt
    state[3] = v
    state[4] = w
    return state

def calc_dynamic_window(current_state, config):
    """Calcula a janela de velocidades alcançáveis."""
    Vs = [config.min_speed, config.max_speed, -config.max_yawrate, config.max_yawrate]
    Vd = [current_state[3] - config.max_accel * config.dt,
          current_state[3] + config.max_accel * config.dt,
          current_state[4] - config.max_dyawrate * config.dt,
          current_state[4] + config.max_dyawrate * config.dt]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
    return dw

def predict_trajectory(initial_state, v, w, config):
    """Prevê a trajetória completa para um par (v,w)."""
    state = np.array(initial_state)
    traj = [state]
    time = 0.0
    while time <= config.predict_time:
        state = motion(state, v, w, config.dt)
        traj.append(np.copy(state))
        time += config.dt
    return np.array(traj)

def calc_to_goal_cost(trajectory, goal, config):
    """Calcula o custo em relação ao objetivo."""
    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    dist_to_goal = math.hypot(dx, dy)
    angle_to_goal = math.atan2(dy, dx)
    angle_diff = abs(angle_to_goal - trajectory[-1, 2])
    return dist_to_goal + angle_diff

def calc_speed_cost(trajectory, config):
    """Calcula o custo de velocidade (incentiva a andar mais rápido)."""
    return config.max_speed - trajectory[-1, 3]

def calc_obstacle_cost(trajectory, detected_obstacles_xy, config):
    """Calcula o custo de proximidade a obstáculos."""
    if not detected_obstacles_xy:
        return 0.0
    min_dist = float('inf')
    for p in trajectory:
        for obs in detected_obstacles_xy:
            dist = math.hypot(p[0] - obs[0], p[1] - obs[1])
            if dist <= config.robot_radius:
                return float('inf') # Custo infinito por colisão
            min_dist = min(min_dist, dist)
    return 1.0 / min_dist

# --- ALTERAÇÃO CRÍTICA: Corrigida a ordem dos parâmetros ---
def dwa_control(current_state, detected_obstacles_xy, goal, config):
    """Função principal que executa o DWA e retorna a melhor velocidade."""
    dw = calc_dynamic_window(current_state, config)
    best_cost = float('inf')
    best_v, best_w = 0.0, 0.0

    for v in np.arange(dw[0], dw[1], config.v_reso):
        for w in np.arange(dw[2], dw[3], config.yawrate_reso):
            traj = predict_trajectory(current_state, v, w, config)
            
            cost_goal = config.to_goal_cost_gain * calc_to_goal_cost(traj, goal, config)
            cost_speed = config.speed_cost_gain * calc_speed_cost(traj, config)
            cost_obstacle = config.obstacle_cost_gain * calc_obstacle_cost(traj, detected_obstacles_xy, config)
            
            total_cost = cost_goal + cost_speed + cost_obstacle

            if total_cost < best_cost:
                best_cost = total_cost
                best_v, best_w = v, w
    return best_v, best_w

# --- Conexão e Inicialização ---
print("Iniciando conexão ao CoppeliaSim...")
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
if clientID == -1: sys.exit('❌ Falha ao conectar ao CoppeliaSim.')
print("✅ Conectado ao CoppeliaSim!")

config = Config()

# Obtenção dos handles dos objetos
_, left_motor = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
_, right_motor = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)
_, robot_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx', sim.simx_opmode_blocking)
if left_motor == -1 or right_motor == -1 or robot_handle == -1: sys.exit('❌ ERRO: Handles não encontrados.')
print("✅ Handles dos motores e robô obtidos.")

# Inicialização dos sensores
sensors = []
for i in range(1, 17):
    _, h = sim.simxGetObjectHandle(clientID, f'Pioneer_p3dx_ultrasonicSensor{i}', sim.simx_opmode_blocking)
    if h == -1: sys.exit(f"❌ ERRO: Sensor Pioneer_p3dx_ultrasonicSensor{i} não encontrado.")
    sensors.append(h)
    sim.simxReadProximitySensor(clientID, h, sim.simx_opmode_streaming)

# Iniciar streaming de dados da pose do robô
sim.simxGetObjectPosition(clientID, robot_handle, -1, sim.simx_opmode_streaming)
sim.simxGetObjectOrientation(clientID, robot_handle, -1, sim.simx_opmode_streaming)
time.sleep(1.0)

# --- DEFINIÇÃO DO OBJETIVO ---
# Altere estas coordenadas para definir para onde o robô deve ir.
goal = np.array([1.5, 1.8]) 
state = np.array([0.0, 0.0, 0.0, 0.0, 0.0]) # x, y, yaw, v, w

print(f"🧭 Iniciando navegação com DWA até o ponto {goal}.")
sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)

# --- Loop de Controle Principal ---
try:
    while sim.simxGetConnectionId(clientID) != -1:
        # 1. Obter estado atual (posição e orientação)
        _, pos = sim.simxGetObjectPosition(clientID, robot_handle, -1, sim.simx_opmode_buffer)
        _, ori = sim.simxGetObjectOrientation(clientID, robot_handle, -1, sim.simx_opmode_buffer)
        if pos and ori:
            state[0], state[1], state[2] = pos[0], pos[1], ori[2]
        else:
            time.sleep(config.dt)
            continue
            
        # 2. Ler sensores e construir mapa de obstáculos
        obstacles_xy = []
        for i, h in enumerate(sensors):
            _, detected, point, _, _ = sim.simxReadProximitySensor(clientID, h, sim.simx_opmode_buffer)
            if detected:
                dist = np.linalg.norm(point)
                if dist < config.sensor_max_range:
                    global_sensor_yaw = state[2] + config.sensor_angles_rad[i]
                    ox = state[0] + dist * math.cos(global_sensor_yaw)
                    oy = state[1] + dist * math.sin(global_sensor_yaw)
                    obstacles_xy.append([ox, oy])

        # 3. Verificar se chegou ao objetivo
        if math.hypot(state[0] - goal[0], state[1] - goal[1]) < 0.5:
            print("\n🎯 Objetivo alcançado!")
            break

        # 4. Executar o DWA para obter as melhores velocidades
        best_v, best_w = dwa_control(state, obstacles_xy, goal, config)
        state[3], state[4] = best_v, best_w # Atualiza o estado com as novas velocidades

        # 5. Converter velocidades (v,w) para velocidades das rodas e comandar motores
        v_right = best_v + (best_w * config.wheel_base / 2.0)
        v_left = best_v - (best_w * config.wheel_base / 2.0)
        w_right = v_right / config.wheel_radius
        w_left = v_left / config.wheel_radius
        
        sim.simxSetJointTargetVelocity(clientID, left_motor, w_left, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(clientID, right_motor, w_right, sim.simx_opmode_oneshot)

        print(f"\rPos: [{state[0]:.2f}, {state[1]:.2f}] | Vel: {best_v:.2f} m/s | Rot: {math.degrees(best_w):.1f}°/s", end="")
        time.sleep(config.dt)

except KeyboardInterrupt:
    print("\nSimulação interrompida pelo utilizador.")
finally:
    print("\n🛑 Parando robô e finalizando simulação...")
    sim.simxSetJointTargetVelocity(clientID, left_motor, 0, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, right_motor, 0, sim.simx_opmode_oneshot)
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
    sim.simxFinish(clientID)
    print('Conexão com CoppeliaSim fechada.')