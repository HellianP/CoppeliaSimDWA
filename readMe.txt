🚗 CoppeliaSim Python Remote API – DWA com Pioneer P3DX
Este repositório contém exemplos de integração entre o simulador CoppeliaSim (antigo V-REP) e scripts Python via Remote API, com foco na simulação de movimentação autônoma do robô Pioneer P3DX, incluindo controle por Dynamic Window Approach (DWA).

📦 Estrutura Necessária
Certifique-se de que os seguintes arquivos estejam no mesmo diretório para que os exemplos funcionem corretamente:

Arquivo/Dependência	Descrição
sim.py	Script de interface Python com a Remote API da CoppeliaSim.
simConst.py	Constantes utilizadas pela API remota do CoppeliaSim.
remoteApi.dll / remoteApi.so / remoteApi.dylib	Biblioteca nativa da Remote API, dependente do sistema operacional.
simpleTest.py	Exemplo de controle simples do robô. Pode ser substituído por outro arquivo de simulação como dwaController.py.
*.ttt	(Opcional) Arquivo de cena do CoppeliaSim com o Pioneer P3DX e sensores configurados.
