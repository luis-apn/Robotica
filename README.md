# Prática 2 — Controle de Movimento (ROS2 + Gazebo Classic)

Pacote ROS2 com o robô diferencial convertido de SDF para URDF, controlado via `gazebo_ros2_control` + `diff_drive_controller`, com controle teleoperado por teclado.

## Pré-requisitos

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Classic 11 (`gazebo_ros_pkgs`)

Instala as dependências necessárias:

```bash
sudo apt install ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-diff-drive-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-controller-manager \
  ros-humble-teleop-twist-keyboard \
  ros-humble-plotjuggler-ros \
  xterm -y
```

## Instalação

Clona o pacote dentro do seu workspace ROS2:

```bash
cd ~/ros2_ws/src
cp -r <caminho-desse-repositorio>/pratica_02/model_description .
```

Builda o pacote:

```bash
cd ~/ros2_ws
colcon build --packages-select model_description
source install/setup.bash
```

## Como rodar

### Terminal 1 — sobe o sistema completo

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch model_description my_robot_urdf.launch.py
```

Isso abre automaticamente:
- **Gazebo Classic**, com o robô carregado e os controllers ativos
- Uma janela **xterm** rodando o `teleop_twist_keyboard`, pronta pra uso

Deixa esse terminal rodando. Pra encerrar tudo, usa `Ctrl+C` nele.

### Controlando o robô pelo teclado

Clica na janela do **xterm** e usa:

| Tecla | Ação |
|---|---|
| `i` | Anda pra frente |
| `,` | Anda de ré |
| `j` | Gira pra esquerda |
| `l` | Gira pra direita |
| `k` | Para o robô |

### Terminal 2 (opcional) — visualizar velocidade em tempo real

```bash
source ~/ros2_ws/install/setup.bash
ros2 run plotjuggler plotjuggler
```

Dentro do PlotJuggler:
1. **Streaming** → **ROS2 Topic Subscriber** → **Start**
2. Marca o tópico `/odom` → **OK**
3. Expande `odom` → `twist` → `twist`
4. Arrasta `linear/x` e `angular/z` pro gráfico

### Controlando via terminal (sem teclado)

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}"
``
