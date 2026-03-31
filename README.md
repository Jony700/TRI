# Assignment 1 — Wall Following Robot

Simulação Gazebo com um robot diferencial (Andino) que segue paredes usando LiDAR.

## Pré-requisitos

- **ROS 2 Jazzy**
- **Gazebo (gz-sim)**
- Pacotes: `andino_gz`, `andino_description`, `ros_gz_sim`, `ros_gz_bridge`, `nav2_common`

## Build

```bash
cd ~/tri_ws
colcon build
source install/setup.bash
```

## Correr a simulação

### Terminal 1 — Gazebo + Robot

```bash
source ~/tri_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch andino_gz assignment1.launch.py
```

> O robot aparece numa posição aleatória perto das paredes. Cada vez que lançam, a posição muda.

### Terminal 2 — Wall Follower

```bash
source ~/tri_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch wall_follower wall_follower.launch.py
```

O robot vai procurar uma parede à esquerda e segui-la automaticamente.

## Parâmetros ajustáveis

| Parâmetro | Default | Descrição |
|---|---|---|
| `desired_distance` | 1.5 m | Distância desejada à parede esquerda |
| `forward_speed` | 0.15 m/s | Velocidade linear |
| `kp` | 1.2 | Ganho proporcional |
| `kd` | 0.4 | Ganho derivativo |
| `front_obstacle_dist` | 0.8 m | Distância para parar (obstáculo à frente) |

Exemplo com parâmetros custom:
```bash
ros2 launch wall_follower wall_follower.launch.py --ros-args -p desired_distance:=1.0 -p forward_speed:=0.2
```

## Teleoperação manual (alternativa ao wall follower)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Estrutura

```
tri_ws/src/
├── andino_gz/          # Simulação Gazebo + mundo assignment1
├── andino/             # Descrição do robot (URDF)
└── wall_follower/      # Nó de wall following (este pacote)
```
