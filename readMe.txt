Assista ao vídeo no YouTube  (https://www.youtube.com/watch?v=s1JZXWI64hs&authuser=1)

🧭 Navegação Autônoma do Robô Pioneer P3DX com DWA no CoppeliaSim
🚗 Visão Geral
Este projeto demonstra a simulação de navegação autônoma do robô Pioneer P3DX no CoppeliaSim (antigo V-REP), utilizando a Remote API Python e o algoritmo Dynamic Window Approach (DWA) para planejamento de trajetórias em tempo real.
Inclui ainda um controle reativo Braitenberg para recuperação, e um sistema de memória espacial para evitar bloqueios em áreas recorrentes.

📂 Estrutura do Projeto
Certifique-se de que os seguintes arquivos estejam presentes no mesmo diretório:
Arquivo / Dependência	Descrição
sim.py	Script de interface Python com a Remote API da CoppeliaSim.
simConst.py	Constantes usadas pela Remote API.
remoteApi.dll, remoteApi.so, remoteApi.dylib	Bibliotecas nativas da Remote API (depende do sistema operacional).
control_robot.py ou simpleTest.py	Script principal para controle do robô (substituível conforme o experimento).
*.ttt (opcional)	Arquivo de cena do CoppeliaSim com o Pioneer P3DX e sensores configurados.

⚙️ Funcionalidades
✅ Navegação autônoma via Dynamic Window Approach (DWA)
🔁 Controle reativo Braitenberg como fallback
🧠 Sistema de memória de quadrantes para evitar áreas bloqueadas
🔍 Monitoramento de obstáculos com sensores e ajustes de alcance
🖥️ Impressão no terminal: posição, obstáculos e velocidade do robô

📋 Requisitos
Requisito	Versão / Observação
Python	3.8 ou superior
CoppeliaSim	Instância configurada com Remote API na porta 19999
Biblioteca Remote API Python	Remote API Functions Python

▶️ Como Executar
Clone o repositório, Abra VScode
Abra o CoppeliaSim e carregue a cena com o robô Pioneer P3DX.
Certifique-se de que a porta 19999 esteja ativada para conexões remotas.
Execute python control_robot.py
Acompanhe no terminal os dados da simulação: posição, obstáculos, comandos de movimento.

