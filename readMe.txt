# Navegação Autônoma do Robô Pioneer P3DX com DWA no CoppeliaSim

🚗 **CoppeliaSim Python Remote API – DWA com Pioneer P3DX**  
Este repositório contém exemplos de integração entre o simulador CoppeliaSim (antigo V-REP) e scripts Python via Remote API, com foco na simulação de movimentação autônoma do robô Pioneer P3DX, incluindo controle por Dynamic Window Approach (DWA).

---

## 📦 Estrutura Necessária

Certifique-se de que os seguintes arquivos estejam no mesmo diretório para que os exemplos funcionem corretamente:

| Arquivo/Dependência                | Descrição                                                                                   |
|----------------------------------|---------------------------------------------------------------------------------------------|
| `sim.py`                         | Script de interface Python com a Remote API da CoppeliaSim.                                |
| `simConst.py`                    | Constantes utilizadas pela API remota do CoppeliaSim.                                      |
| `remoteApi.dll / remoteApi.so / remoteApi.dylib` | Biblioteca nativa da Remote API, dependente do sistema operacional.                 |
| `simpleTest.py`                  | Exemplo de controle simples do robô. Pode ser substituído por outro arquivo, como `control_robot.py`. |
| `*.ttt`                         | (Opcional) Arquivo de cena do CoppeliaSim com o Pioneer P3DX e sensores configurados.      |

---

## Descrição

- Algoritmo DWA para planejamento de trajetórias em tempo real.
- Controle reativo Braitenberg como fallback para recuperação em casos de bloqueio.
- Monitoramento e memória de quadrantes para evitar que o robô fique preso em regiões.
- Ajustes nos alcances dos sensores para balancear segurança e eficiência.
- Impressão detalhada no terminal sobre a posição, detecção de obstáculos, e velocidades do robô.

---

## Requisitos

- Python 3.8+
- Biblioteca [CoppeliaSim Remote API Python](https://coppeliarobotics.com/helpFiles/en/remoteApiFunctionsPython.htm)
- CoppeliaSim instalado e configurado para aceitar conexões remotas na porta 19999.

---

## Como executar

1. Inicie o CoppeliaSim e abra a cena com o robô Pioneer P3DX configurado.
2. Ative a API remota na porta 19999 (padrão).
3. No terminal, execute:
   ```bash
   python control_robot.py
