import sim
import time
import math
import numpy as np
import sys

# --- Parâmetros DWA e Configurações do Robô ---
class Config:
    def __init__(self):
        self.max_speed = 0.5          # [m/s]
        self.min_speed = -0.2         # [m/s]
        self.max_yawrate = 50.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 1.0          # [m/ss]
        self.max_dyawrate = 70.0 * math.pi / 180.0 # [rad/ss]
        self.robot_radius = 0.45      # [m]
        self.wheel_base = 0.5         # [m]
        self.wheel_radius = 0.076     # [m]

        self.dt = 0.1                 # [s]
        self.predict_time = 2.0       # [s]
        self.v_reso = 0.02            # [m/s]
        self.yawrate_reso = 0.2 * math.pi / 180.0 # [rad/s]

        self.to_goal_cost_gain = 0.5
        self.speed_cost_gain = 0.1
        self.obstacle_cost_gain = 10.0

        self.sensor_max_range = 0.8   # reduzido 20%
        self.sensor_angles_rad = np.array([
            -90.0, -75.0, -60.0, -45.0, -30.0, -15.0, 0.0, 15.0,
            15.0, 30.0, 45.0, 60.0, 75.0, 90.0, 135.0, -135.0
        ]) * math.pi / 180.0

# Função de movimentação do robô com velocidade linear v e angular w
def motion(state, v, w, dt):
    state[0] += v * math.cos(state[2]) * dt
    state[1] += v * math.sin(state[2]) * dt
    state[2] += w * dt
    state[3] = v
    state[4] = w
    return state

# Calcula a janela dinâmica respeitando limites de velocidade e aceleração
def calc_dynamic_window(current_state, config):
    Vs = [config.min_speed, config.max_speed, -config.max_yawrate, config.max_yawrate]
    Vd = [current_state[3] - config.max_accel * config.dt,
          current_state[3] + config.max_accel * config.dt,
          current_state[4] - config.max_dyawrate * config.dt,
          current_state[4] + config.max_dyawrate * config.dt]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
    return dw

# Previsão da trajetória para um par (v,w)
def predict_trajectory(initial_state, v, w, config):
    state = np.array(initial_state)
    traj = [state]
    time = 0.0
    while time <= config.predict_time:
        state = motion(state, v, w, config.dt)
        traj.append(np.copy(state))
        time += config.dt
    return np.array(traj)

# Cálculo de custo em relação ao objetivo
def calc_to_goal_cost(trajectory, goal, config):
    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    dist_to_goal = math.hypot(dx, dy)
    angle_to_goal = math.atan2(dy, dx)
    angle_diff = abs(angle_to_goal - trajectory[-1, 2])
    return dist_to_goal + angle_diff

# Cálculo do custo de velocidade para preferir alta velocidade
def calc_speed_cost(trajectory, config):
    return config.max_speed - trajectory[-1, 3]

# Cálculo de custo em relação a obstáculos detectados
def calc_obstacle_cost(trajectory, detected_obstacles_xy, config):
    if not detected_obstacles_xy:
        return 0.0
    min_dist = float('inf')
    for p in trajectory:
        for obs in detected_obstacles_xy:
            dist = math.hypot(p[0] - obs[0], p[1] - obs[1])
            if dist <= config.robot_radius:
                return float('inf')  # colisão
            min_dist = min(min_dist, dist)
    return 1.0 / min_dist

# Algoritmo DWA para controle de velocidade
def dwa_control(current_state, detected_obstacles_xy, goal, config):
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

# Controle Braitenberg para recuperação quando DWA trava
def braitenberg_control(detected_obstacles_xy, state, config):
    braitenbergL = np.array([-0.2, -0.4, -0.6, -0.8, -1, -1.2, -1.4, -1.6,
                             0, 0, 0, 0, 0, 0, 0, 0])
    braitenbergR = np.array([-1.6, -1.4, -1.2, -1, -0.8, -0.6, -0.4, -0.2,
                             0, 0, 0, 0, 0, 0, 0, 0])
    v0 = 2.0
    detect = np.zeros(16)

    for i in range(16):
        min_dist = float('inf')
        sensor_angle = config.sensor_angles_rad[i]
        for ox, oy in detected_obstacles_xy:
            dx = ox - state[0]
            dy = oy - state[1]
            obs_dist = math.hypot(dx, dy)
            obs_angle = math.atan2(dy, dx) - state[2]
            obs_angle = (obs_angle + math.pi) % (2 * math.pi) - math.pi
            if abs(obs_angle - sensor_angle) < (10 * math.pi / 180):
                if obs_dist < min_dist:
                    min_dist = obs_dist

        noDetectionDist = config.sensor_max_range
        maxDetectionDist = 0.2

        if min_dist < noDetectionDist:
            if min_dist < maxDetectionDist:
                min_dist = maxDetectionDist
            detect[i] = 1 - ((min_dist - maxDetectionDist) / (noDetectionDist - maxDetectionDist))
        else:
            detect[i] = 0

    vLeft = v0 + np.dot(braitenbergL, detect)
    vRight = v0 + np.dot(braitenbergR, detect)

    v = config.wheel_radius * (vLeft + vRight) / 2.0
    w = config.wheel_radius * (vRight - vLeft) / config.wheel_base

    return v, w

# --- Inicialização conexão com CoppeliaSim ---
print("Iniciando conexão ao CoppeliaSim...")
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
if clientID == -1: sys.exit('❌ Falha ao conectar ao CoppeliaSim.')
print("✅ Conectado ao CoppeliaSim!")

config = Config()

_, left_motor = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
_, right_motor = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)
_, robot_handle = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx', sim.simx_opmode_blocking)
if left_motor == -1 or right_motor == -1 or robot_handle == -1:
    sys.exit('❌ ERRO: Handles não encontrados.')
print("✅ Handles obtidos.")

# Obtém handles dos sensores e inicializa streaming de sensores
sensors = []
for i in range(1, 17):
    _, h = sim.simxGetObjectHandle(clientID, f'Pioneer_p3dx_ultrasonicSensor{i}', sim.simx_opmode_blocking)
    if h == -1:
        sys.exit(f"❌ ERRO: Sensor Pioneer_p3dx_ultrasonicSensor{i} não encontrado.")
    sensors.append(h)
    sim.simxReadProximitySensor(clientID, h, sim.simx_opmode_streaming)

# Inicializa streaming da posição e orientação
sim.simxGetObjectPosition(clientID, robot_handle, -1, sim.simx_opmode_streaming)
sim.simxGetObjectOrientation(clientID, robot_handle, -1, sim.simx_opmode_streaming)
time.sleep(1.0)  # espera inicial para dados estabilizarem

goal = np.array([-2.15, 3])  # ponto meta
state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # x, y, yaw, v, w

print(f"🧭 Iniciando navegação com DWA até o ponto {goal}.")
sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot)

try:
    last_quadrant = None
    quadrant_timer = 0.0
    quadrant_timeout = 5.0
    t_start = time.time()
    quadrant_size = 1.0

    while sim.simxGetConnectionId(clientID) != -1:
        # Leitura posição e orientação do robô
        _, pos = sim.simxGetObjectPosition(clientID, robot_handle, -1, sim.simx_opmode_buffer)
        _, ori = sim.simxGetObjectOrientation(clientID, robot_handle, -1, sim.simx_opmode_buffer)

        # Validação: garante que pos e ori são válidos e não contém NaN
        if (not pos or not ori or
            any(math.isnan(coord) for coord in pos) or
            any(math.isnan(angle) for angle in ori)):
            time.sleep(config.dt)
            continue

        # Atualiza estado do robô com leitura válida
        state[0], state[1], state[2] = pos[0], pos[1], ori[2]

        # Detecta se o robô está preso no mesmo quadrante por muito tempo
        current_quadrant = (int(state[0] // quadrant_size), int(state[1] // quadrant_size))
        if current_quadrant == last_quadrant:
            quadrant_timer += config.dt
        else:
            quadrant_timer = 0.0
            last_quadrant = current_quadrant

        if quadrant_timer > quadrant_timeout:
            print(f"\n⚠️ Robô preso no quadrante {current_quadrant}. Resetando posição para origem.")
            state[0], state[1], state[2] = 0.0, 0.0, 0.0
            quadrant_timer = 0.0
            # Aqui poderia implementar reset na simulação ou alteração da meta, se desejado

        # Leitura sensores e cálculo das posições dos obstáculos
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

        # Impressão estado atual para monitoramento
        print(f"\nPosição: x={state[0]:.2f} m, y={state[1]:.2f} m, yaw={math.degrees(state[2]):.1f}°")
        print(f"Obstáculos detectados: {len(obstacles_xy)}")
        for i, (ox, oy) in enumerate(obstacles_xy):
            print(f"  Sensor {i+1}: ({ox:.2f}, {oy:.2f})")

        # Verifica se objetivo foi alcançado
        if math.hypot(state[0] - goal[0], state[1] - goal[1]) < 0.5:
            print("\n🎯 Objetivo final alcançado!")
            break

        # Calcula controle DWA
        best_v, best_w = dwa_control(state, obstacles_xy, goal, config)

        # Se DWA trava, ativa controle Braitenberg para recuperação
        if abs(best_v) < 1e-3 and abs(best_w) < 1e-3:
            print("⚠️ Velocidade zero detectada, ativando controle Braitenberg para recuperação.")
            best_v, best_w = braitenberg_control(obstacles_xy, state, config)

        state[3], state[4] = best_v, best_w

        # Conversão de velocidades linear e angular para velocidades angulares das rodas
        v_right = best_v + (best_w * config.wheel_base / 2.0)
        v_left = best_v - (best_w * config.wheel_base / 2.0)
        w_right = v_right / config.wheel_radius
        w_left = v_left / config.wheel_radius

        # Envio dos comandos para os motores
        sim.simxSetJointTargetVelocity(clientID, left_motor, w_left, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(clientID, right_motor, w_right, sim.simx_opmode_oneshot)

        print(f"Vel linear: {best_v:.2f} m/s | Vel angular: {math.degrees(best_w):.1f} °/s")
        print(f"Vel rodas (rad/s): Esq={w_left:.2f} | Dir={w_right:.2f}")

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
