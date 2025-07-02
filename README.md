<div align="center">
<picture>
    <source srcset="https://imgur.com/5bYAzsb.png" media="(prefers-color-scheme: dark)">
    <source srcset="https://imgur.com/Os03JoE.png" media="(prefers-color-scheme: light)">
    <img src="https://imgur.com/Os03JoE.png" alt="Escudo UNAL" width="350px">
</picture>

<h3>Curso de Robótica 2025-I</h3>

<h1>Introducción a Robots Phantom Pincher X100</h1>

<h2>Guía 04 - Uso Básico de los Robots Phantom Pincher X100 en ROS 2 Humble</h2>


<h4>Pedro Fabián Cárdenas Herrera<br>
    Manuel Felipe Carranza Montenegro</h4>

</div>

<div align="justify"> 

## Introducción

Este repositorio, **PhantomX Pincher ROS 2 Control**, ofrece un paquete en Python que facilita el movimiento de los cinco servomotores AX-12A de tu PhantomX Pincher usando ROS 2 Humble. Con él podrás, de manera rápida y sencilla:

- Configurar y lanzar un nodo ROS 2 (`pincher_control`) desde terminal o VS Code.  
- Enviar posiciones objetivo a múltiples servos de forma simultánea.  
- Ajustar la velocidad de comunicación y el retardo para garantizar la correcta ejecución del movimiento.  
- Servir como base para integrar control más avanzado, como `ros2_control`, topics o acciones de trayectoria.

## Objetivos

1. **Automatizar el control** de los servomotores AX-12A del PhantomX Pincher mediante un nodo ROS 2.  
2. **Parametrizar** la conexión (puerto serie y baudios), la lista de IDs de servos y la posición objetivo.  
3. **Incluir un retardo configurable** para asegurar que los servos alcanzan la posición antes de desactivar el torque.  
4. **Ofrecer un ejemplo claro** de uso en terminal (`ros2 run …`) y configuración en VS Code.  
5. **Proporcionar un punto de partida** para futuras extensiones: integración con `ros2_control`, generación de trayectorias y feedback de estado.


> **Prerrequisitos:** 
> - Ubuntu 22.04  
> - ROS 2 Humble  
> - Haber ejecutado `source /opt/ros/humble/setup.bash` en cada terminal nueva.
> - `python3-dynamixel-sdk`. 
> - Acceso al puerto serie (`dialout`).  
> - Adaptador USB2Dynamixel conectado a `/dev/ttyUSB0`.

## Preparativos de hardware y software

### Conecta el robot

- Asegúrate de alimentar el bus de Dynamixel (12 V) y conecta el adaptador USB2Dynamixel (U2D2 o FTDI) al PC por `/dev/ttyUSB0` (o similar).
- Verifica que cada AX-12A tenga un ID único (por defecto suelen venir del 1 al 5).

### Instala dependencias ROS 2 y DynamixelSDK

Esto da las bibliotecas necesarias para la comunicación con los servos desde ROS2 o Python.

```bash
sudo apt update
sudo apt install -y ros-humble-dynamixel-sdk
sudo apt install python3-rosdep2
sudo apt install python3-serial
```

### Añadir tu usuario al grupo dialout
```bash
ls -l /dev/ttyUSB0
sudo usermod -aG dialout $USER
groups
```
> En caso de que `dialout` no aparezca dentro de `groups` se debe reiniciar la máquina.

## Crea y construye un workspace

### Crear workspace y clonar demos de simulación (opcional).

```bash
mkdir -p ~/ros2_ws/phantom_ws/src && cd ~/ros2_ws/phantom_ws/src
git clone https://github.com/snt-spacer/phantomx_pincher.git
```

### Instalar dependencias con rosdep
```bash
cd ~/phantom_ws
rosdep update
rosdep install -y -r -i \
    --from-paths src \
    --rosdistro humble
```

### Compilar

```bash
colcon build
```

### Fuente

```bash
source install/setup.bash
```

## Crear un paquete de control en Python

### Genera el paquete

```bash
cd ~/phantom_ws/src
ros2 pkg create --build-type ament_python pincher_control --dependencies rclpy dynamixel_sdk
```

### Estructura del paquete

```bash
pincher_control
├── package.xml
├── setup.py
└── pincher_control
    └── control_servo.py
```

### Editar `setup.py`

```python
entry_points={
  'console_scripts': [
    'control_servo = pincher_control.control_servo:main',
  ],
},
```

### Implementa `control_servo.py`

```python
# pincher_control/control_servo.py
import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler
import time

# Direcciones de registro en el AX-12A
ADDR_TORQUE_ENABLE    = 24
ADDR_GOAL_POSITION    = 30
ADDR_MOVING_SPEED     = 32
ADDR_TORQUE_LIMIT     = 34
ADDR_PRESENT_POSITION = 36

class PincherController(Node):
    def __init__(self):
        super().__init__('pincher_controller')

        # Parámetros
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('dxl_ids', [1, 2, 3, 4, 5])
        self.declare_parameter('goal_positions', [512, 512, 512, 512, 512])
        self.declare_parameter('moving_speed', 100)     # 0–1023 
        self.declare_parameter('torque_limit', 1000)     # 0–1023 
        self.declare_parameter('delay', 2.0)

        port_name      = self.get_parameter('port').value
        baudrate       = self.get_parameter('baudrate').value
        dxl_ids        = self.get_parameter('dxl_ids').value
        goal_positions = self.get_parameter('goal_positions').value
        moving_speed   = self.get_parameter('moving_speed').value
        torque_limit   = self.get_parameter('torque_limit').value
        delay_seconds  = self.get_parameter('delay').value

        if len(goal_positions) != len(dxl_ids):
            self.get_logger().error(
                f'La lista goal_positions ({len(goal_positions)}) '
                f'debe tener la misma longitud que dxl_ids ({len(dxl_ids)})'
            )
            rclpy.shutdown()
            return

        # Inicializar comunicación
        port   = PortHandler(port_name)
        port.openPort()
        port.setBaudRate(baudrate)
        packet = PacketHandler(1.0)

        # 1) Configurar torque_limit, velocidad y enviar posición a cada servo
        for dxl_id, goal in zip(dxl_ids, goal_positions):
            # Limitar torque
            packet.write2ByteTxRx(port, dxl_id, ADDR_TORQUE_LIMIT, torque_limit)
            # Limitar velocidad
            packet.write2ByteTxRx(port, dxl_id, ADDR_MOVING_SPEED, moving_speed)
            # Habilitar torque
            packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 1)
            # Enviar posición objetivo
            packet.write2ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, goal)
            self.get_logger().info(f'[ID {dxl_id}] → goal={goal}, speed={moving_speed}, torque_limit={torque_limit}')

        # 2) (Opcional) Leer y mostrar posición actual
        for dxl_id in dxl_ids:
            pos, _, _ = packet.read2ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
            self.get_logger().info(f'[ID {dxl_id}] posición actual={pos}')

        # 3) Esperar a que todos los servos alcancen la posición
        self.get_logger().info(f'Esperando {delay_seconds}s para completar movimiento...')
        time.sleep(delay_seconds)

        # 4) Apagar torque en todos los servos
        for dxl_id in dxl_ids:
            packet.write1ByteTxRx(port, dxl_id, ADDR_TORQUE_ENABLE, 0)

        # 5) Cerrar puerto y terminar nodo
        port.closePort()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    PincherController()
    # No es necesario spin() para un movimiento puntual
    # rclpy.spin(node)  # habilita sólo si agregas callbacks

if __name__ == '__main__':
    main()

```

### Compila y fuente

```bash
cd ~/phantom_ws
colcon build
source install/setup.bash
```

### Ejecuta el nodo con valores por defecto

```bash
ros2 run pincher_control control_servo
```


## Recursos útiles

- [Documentación oficial de ROS 2 Humble](https://docs.ros.org/en/humble/index.html)  
  Guía completa con tutoriales, ejemplos y referencia de API.

- [DynamixelSDK (Python)](https://github.com/ROBOTIS-GIT/DynamixelSDK)  
  Repositorio oficial con ejemplos y guía de uso para controlar servos Dynamixel.

- [Colcon Build](https://colcon.readthedocs.io/en/released/)  
  Herramienta de compilación de ROS 2: `colcon build`, instalación de dependencias y configuración de workspaces.

- [Configuración de permisos para dispositivos serie (udev)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html#configure-permissions)  
  Cómo añadir tu usuario a `dialout` o crear reglas udev para acceso a `/dev/ttyUSB*`.

- [Extensión ROS para Visual Studio Code](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros)  
  Integra comandos ROS 2, depuración y autocompletado en VS Code.


</div>