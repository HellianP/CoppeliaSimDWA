import sim
import time
import math
import numpy as np
import sys

# --- Conexão com o CoppeliaSim ---
print("🔌 Iniciando conexão com o CoppeliaSim...")
sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
if clientID == -1:
    sys.exit("❌ Falha ao conectar ao CoppeliaSim.")
print("✅ Conectado!")

# --- Obtenção dos handles ---
_, left_motor = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', sim.simx_opmode_blocking)
_, right_motor = sim.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', sim.simx_opmode_blocking)
sensors = []
for i in range(1, 17):
    _, sensor = sim.simxGetObjectHandle(clientID, f'Pioneer_p3dx_ultrasonicSensor{i}', sim.simx_opmode_blocking)
    sensors.append(sensor)
    sim.simxReadProximitySensor(clientID, sensor, sim.simx_opmode_streaming)
print("✅ Sensores e motores prontos.")

# --- Parâmetros Braitenberg ---
noDetectionDist = 0.5
maxDetectionDist = 0.2
v0 = 2.0  # velocidade base

braitenbergL = [-0.2,-0.4,-0.6,-0.8,-1,-1.2,-1.4,-1.6, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
braitenbergR = [-1.6,-1.4,-1.2,-1,-0.8,-0.6,-0.4,-0.2, 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

# --- Loop principal ---
try:
    while sim.simxGetConnectionId(clientID) != -1:
        detect = [0.0] * 16

        for i, sensor in enumerate(sensors):
            _, detectionState, detectedPoint, _, _ = sim.simxReadProximitySensor(clientID, sensor, sim.simx_opmode_buffer)
            if detectionState:
                dist = np.linalg.norm(detectedPoint)
                if dist < noDetectionDist:
                    dist = max(dist, maxDetectionDist)
                    detect[i] = 1.0 - ((dist - maxDetectionDist) / (noDetectionDist - maxDetectionDist))

        # Cálculo das velocidades com pesos Braitenberg
        vLeft = v0
        vRight = v0
        for i in range(16):
            vLeft += braitenbergL[i] * detect[i]
            vRight += braitenbergR[i] * detect[i]

        sim.simxSetJointTargetVelocity(clientID, left_motor, vLeft, sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(clientID, right_motor, vRight, sim.simx_opmode_oneshot)

        print(f"\rL: {vLeft:.2f} | R: {vRight:.2f}", end="")
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n⛔ Encerrando simulação...")
finally:
    sim.simxSetJointTargetVelocity(clientID, left_motor, 0, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(clientID, right_motor, 0, sim.simx_opmode_oneshot)
    sim.simxFinish(clientID)
    print("✅ Finalizado com sucesso.")
