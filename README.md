# Robótica — Práticas com ROS2 + Gazebo Classic

Repositório com as atividades práticas da disciplina de **Robótica**, desenvolvidas em **ROS2 Humble** e simuladas no **Gazebo Classic 11**. Todas as práticas usam o mesmo robô diferencial: a prática 02 monta o robô e a simulação, e as práticas seguintes adicionam os nós de controle por cima dela.

Cada prática fica numa pasta própria, com um README detalhando o funcionamento e como executar.

## Práticas

| Pasta | Tema | Conteúdo |
|---|---|---|
| [pratica_02](pratica_02/README.md) | Controle de movimento | Pacote `model_description`: robô diferencial em URDF, `gazebo_ros2_control` + `diff_drive_controller`, teleoperação por teclado e visualização da velocidade no PlotJuggler. |
| [pratica_03](pratica_03/README.md) | Controle de pose e trajetória | Pacote `controle_trajetoria`: controle de pose (contínuo e três manobras), seguimento de missão por waypoints e trajetória em 8 (malha aberta e realimentado com feedforward), com cálculo do RMSE. |

## Estrutura do repositório

```
Robotica/
├── pratica_02/
│   ├── README.md
│   └── model_description/      # Pacote ROS2 (ament_cmake): URDF, launch e config do robô
└── pratica_03/
    ├── README.md
    ├── Prtica_robtica_03___Controle_de_trajetria.pdf   # Roteiro da atividade
    ├── controle_trajetoria/    # Pacote ROS2 (ament_python): nós de controle
    └── config/                 # Parâmetros (YAML) de cada nó
```

## Ambiente

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Classic 11
- PlotJuggler (`ros-humble-plotjuggler-ros`) para visualização

As dependências específicas de cada prática estão listadas no README dela.

## Uso rápido

Copia os pacotes pro seu workspace ROS2 e builda:

```bash
cd ~/ros2_ws/src
cp -r <caminho-desse-repositorio>/pratica_02/model_description .
cp -r <caminho-desse-repositorio>/pratica_03 controle_trajetoria

cd ~/ros2_ws
colcon build
source install/setup.bash
```

Sobe a simulação (base de todas as práticas):

```bash
ros2 launch model_description my_robot_urdf.launch.py
```

A partir daí, segue as instruções do README da prática desejada.
