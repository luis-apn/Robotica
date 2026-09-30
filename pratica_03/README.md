# Prática 3 — Controle de Pose e Trajetória (ROS2 + Gazebo Classic)

Pacote ROS2 (`controle_trajetoria`, Python) com os nós de controle do robô diferencial da prática 02: controle de pose (contínuo e com três manobras), seguimento de missão por waypoints e seguimento de trajetória em 8 (malha aberta e realimentado com feedforward). Todos os ganhos e parâmetros são carregados de arquivos YAML e os PIDs usam a biblioteca [`simple-pid`](https://pypi.org/project/simple-pid/).

O roteiro completo da atividade está em [Prtica_robtica_03___Controle_de_trajetria.pdf](Prtica_robtica_03___Controle_de_trajetria.pdf).

## Pré-requisitos

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Classic 11 (`gazebo_ros_pkgs`)
- Pacote `model_description` da [prática 02](../pratica_02/README.md) instalado e funcionando (é ele que sobe o robô no Gazebo)

Instala as dependências necessárias (além das da prática 02):

```bash
sudo apt install ros-humble-plotjuggler-ros python3-pip -y
pip install simple-pid
```

## Estrutura do pacote

```
pratica_03/
├── controle_trajetoria/
│   ├── base_node.py         # Nó base: lê /odom, publica /cmd_vel, carrega parâmetros
│   ├── controllers.py       # Leis de controle (Python puro, sem ROS)
│   ├── utils.py             # PID (simple_pid), quatérnio ↔ yaw, wrap_angle, saturação
│   ├── odom_monitor.py      # Imprime a pose do robô no terminal
│   ├── pose_controller.py   # Exercício 1.1 — controle de pose
│   ├── mission.py           # Exercício 1.2 — missão com waypoints
│   └── trajectory_8.py      # Exercícios 2.1 e 2.2 — trajetória em 8
└── config/
    ├── odom_monitor.yaml
    ├── pose_controller.yaml
    ├── mission.yaml
    └── trajectory_8.yaml
```

| Nó | Assina | Publica |
|---|---|---|
| `odom_monitor` | `/odom` | — (só log) |
| `pose_controller` | `/odom`, `/pose_desejada` (`geometry_msgs/Pose2D`) | `/cmd_vel` |
| `mission` | `/odom` | `/cmd_vel` |
| `trajectory_8` | `/odom` | `/cmd_vel`, `/trajetoria_desejada` (`geometry_msgs/PoseStamped`) |

> Os nós de controle rodam o laço a cada mensagem de `/odom` (50 Hz) em vez de usar um timer, porque com `use_sim_time` o timer depende do `/clock`, que o Gazebo publica a apenas 10 Hz. O `dt` vem do carimbo de tempo da própria odometria.

## Instalação

Copia o pacote pra dentro do seu workspace ROS2 (junto com o `model_description` da prática 02):

```bash
cd ~/ros2_ws/src
cp -r <caminho-desse-repositorio>/pratica_03 controle_trajetoria
```

Builda o pacote:

```bash
cd ~/ros2_ws
colcon build --packages-select model_description controle_trajetoria
source install/setup.bash
```

> Os YAMLs são instalados em `~/ros2_ws/install/controle_trajetoria/share/controle_trajetoria/config/`. Se editar algum, roda o `colcon build` de novo (ou aponta o `--params-file` direto pro arquivo em `src/`).

Pra facilitar os comandos abaixo, define uma variável com o caminho dos YAMLs:

```bash
export CFG=~/ros2_ws/install/controle_trajetoria/share/controle_trajetoria/config
```

## Como rodar

### Terminal 1 — sobe a simulação (prática 02)

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch model_description my_robot_urdf.launch.py
```

Isso abre o **Gazebo** com o robô e a janela **xterm** do teleop. O teleop pode ficar aberto, mas **não aperta nenhuma tecla nele** enquanto um nó de controle estiver rodando, senão os dois brigam pelo `/cmd_vel`.

Deixa esse terminal rodando. Nos outros terminais, lembra sempre de fazer `source ~/ros2_ws/install/setup.bash` e definir o `CFG`.

### (Opcional) Monitorar a pose do robô

```bash
ros2 run controle_trajetoria odom_monitor --ros-args --params-file $CFG/odom_monitor.yaml
```

Imprime `x`, `y` e `θ` do robô 10 vezes por segundo.

---

### Exercício 1.1 — Controle de pose

**Terminal 2** — sobe o controlador:

```bash
ros2 run controle_trajetoria pose_controller --ros-args --params-file $CFG/pose_controller.yaml
```

**Terminal 3** — manda uma pose desejada (x, y em metros; θ em radianos):

```bash
ros2 topic pub --once /pose_desejada geometry_msgs/msg/Pose2D "{x: 2.0, y: 1.0, theta: 1.57}"
```

O nó loga `Pose atingida` quando chega no alvo e para de publicar. Dá pra mandar uma nova pose a qualquer momento.

O parâmetro `mode` do [pose_controller.yaml](config/pose_controller.yaml) escolhe a lei de controle:

| `mode` | Funcionamento |
|---|---|
| `continuous` | Comanda `v` e `w` ao mesmo tempo: `v = PID_v(ρ)·cos(α)`, `w = PID_w(α)`. Ao entrar na tolerância de posição, gira no lugar até a orientação final. |
| `three_maneuvers` (bônus) | Máquina de estados: **gira** até apontar pro alvo → **anda reto** (com pequena correção de rumo) → **gira** até a orientação final. |

Onde `ρ` é a distância até o alvo e `α` o desalinhamento entre o rumo do robô e a direção do alvo.

Pra trocar o modo sem editar o YAML:

```bash
ros2 run controle_trajetoria pose_controller --ros-args --params-file $CFG/pose_controller.yaml -p mode:=three_maneuvers
```

Principais parâmetros:

| Parâmetro | Descrição |
|---|---|
| `linear.kp/ki/kd` | Ganhos do PID da velocidade linear (sobre `ρ`) |
| `angular.kp/ki/kd` | Ganhos do PID da velocidade angular (sobre o erro angular) |
| `position_tolerance` | Distância (m) em que a posição conta como atingida |
| `angle_tolerance` | Erro angular (rad) em que a orientação conta como atingida |
| `v_max`, `w_max` | Saturação de `v` (m/s) e `w` (rad/s) |

> Valores reais no YAML precisam ter ponto decimal (`1.0`, não `1`), senão o ROS2 acusa erro de tipo.

---

### Exercício 1.2 — Seguimento de missão

**Terminal 2:**

```bash
ros2 run controle_trajetoria mission --ros-args --params-file $CFG/mission.yaml
```

O robô segue os waypoints em sequência e loga cada um atingido, até `Missão concluída`. Os waypoints ficam no [mission.yaml](config/mission.yaml) como dois arrays paralelos (o ROS2 não aceita lista de listas como parâmetro) — o i-ésimo waypoint é `(waypoints_x[i], waypoints_y[i])`, com no mínimo 3:

```yaml
waypoints_x: [2.0, 1.0, -1.0]
waypoints_y: [1.0, 2.5, 1.0]
waypoint_radius: 0.15         # m, raio em que o waypoint conta como atingido
```

Um waypoint é considerado atingido quando o robô entra no círculo de raio `waypoint_radius` (a orientação não importa). Os ganhos são os mesmos do controle de pose.

**(Bônus) Comparação dos controladores:** roda a missão com `mode:=continuous` e depois com `mode:=three_maneuvers`, e compara o caminho percorrido no PlotJuggler (XY de `/odom`) e o tempo total:

```bash
ros2 run controle_trajetoria mission --ros-args --params-file $CFG/mission.yaml -p mode:=three_maneuvers
```

> Entre uma execução e outra, reinicia a simulação (Terminal 1) ou reseta o mundo no Gazebo (`Ctrl+R`) pra o robô voltar à origem.

---

### Exercício 2.1 — Trajetória em 8

**Terminal 2:**

```bash
ros2 run controle_trajetoria trajectory_8 --ros-args --params-file $CFG/trajectory_8.yaml
```

A trajetória desejada (lemniscata de Gerono) é:

```
x_d = A·sin(Ωt)
y_d = B·sin(Ωt)·cos(Ωt)
```

com `A = 1`, `B = 2`, `Ω = 0,5 rad/s` carregados do [trajectory_8.yaml](config/trajectory_8.yaml). A trajetória é ancorada na posição do robô no início, então dá pra rodar de novo sem reiniciar o Gazebo. Com `align_first: true`, o robô primeiro gira no lugar até o rumo inicial da trajetória, pra comparação entre os modos ser justa.

A partir das derivadas da trajetória, o nó calcula as velocidades de referência de um uniciclo (feedforward):

```
v_d = √(ẋ² + ẏ²)
w_d = (ẋ·ÿ − ẏ·ẍ) / (ẋ² + ẏ²)
θ_d = atan2(ẏ, ẋ)
```

O parâmetro `mode` escolhe como seguir:

| `mode` | Funcionamento |
|---|---|
| `open_loop` | Malha aberta: publica só `(v_d, w_d)`, ignorando a odometria. |
| `feedback` | Realimentado + feedforward: controla um ponto `P` a uma distância `d` à frente do eixo das rodas, com `u = K_ff·Ṗ_ref + PID(P_ref − P)` (um PID por eixo), convertido em `v` e `w`. |

O ganho `k_ff` multiplica o termo feedforward: `k_ff = 0` é só realimentação, `k_ff = 1` é realimentação + feedforward completo.

Exemplos:

```bash
# Malha aberta
ros2 run controle_trajetoria trajectory_8 --ros-args --params-file $CFG/trajectory_8.yaml -p mode:=open_loop

# Só realimentação (K_ff = 0)
ros2 run controle_trajetoria trajectory_8 --ros-args --params-file $CFG/trajectory_8.yaml -p mode:=feedback -p k_ff:=0.0

# Realimentação + feedforward (K_ff = 1)
ros2 run controle_trajetoria trajectory_8 --ros-args --params-file $CFG/trajectory_8.yaml -p mode:=feedback -p k_ff:=1.0
```

Principais parâmetros:

| Parâmetro | Descrição |
|---|---|
| `A`, `B`, `Omega` | Parâmetros da trajetória em 8 |
| `laps` | Número de voltas completas (período `2π/Ω`) |
| `mode` | `open_loop` ou `feedback` |
| `k_ff` | Ganho do termo feedforward |
| `d` | Distância (m) do ponto controlado à frente do robô |
| `feedback.kp/ki/kd` | Ganhos do PID de posição (eixos x e y) |
| `align_first`, `align_tolerance`, `align.kp/ki/kd` | Alinhamento inicial com o rumo da trajetória |
| `v_max`, `w_max` | Saturação (mesmos limites do `diff_drive_controller` da prática 02) |

**(Bônus) Maior Ω:** vai aumentando o `Omega` (`-p Omega:=0.8`, `-p Omega:=1.0`, ...) até o robô não conseguir mais seguir a trajetória — o RMSE no fim da execução sobe bastante e as velocidades batem na saturação (`v_max`, `w_max`).

---

### Exercício 2.2 — Visualização no PlotJuggler

Com o `trajectory_8` rodando, abre em outro terminal:

```bash
ros2 run plotjuggler plotjuggler
```

Dentro do PlotJuggler:
1. **Streaming** → **ROS2 Topic Subscriber** → **Start**
2. Marca os tópicos `/odom` e `/trajetoria_desejada` → **OK**
3. Pra ver a trajetória no plano **XY**, seleciona duas séries e arrasta com o **botão direito** do mouse pro gráfico:
   - Desejada: `/trajetoria_desejada/pose/position/x` e `/trajetoria_desejada/pose/position/y`
   - Real: `/odom/pose/pose/position/x` e `/odom/pose/pose/position/y`
4. Pra ver no tempo, arrasta as séries normalmente (botão esquerdo)

**(Bônus) RMSE:** ao final das `laps` voltas, o nó para o robô e loga o erro médio quadrático de posição e de orientação, no formato:

```
Fim (<mode>, k_ff=<k_ff>, Ω=<Omega>): RMSE posição = <erro> m, RMSE orientação = <erro>° (<n> amostras)
```

Roda os três casos (malha aberta, `k_ff = 0` e `k_ff = 1`) com os comandos do exercício 2.1 e compara os valores.
