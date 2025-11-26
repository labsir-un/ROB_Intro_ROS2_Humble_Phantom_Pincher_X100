<div align="center">
<picture>
    <source srcset="https://imgur.com/5bYAzsb.png" media="(prefers-color-scheme: dark)">
    <source srcset="https://imgur.com/Os03JoE.png" media="(prefers-color-scheme: light)">
    <img src="https://imgur.com/Os03JoE.png" alt="Escudo UNAL" width="350px">
</picture>

<h3>Curso de Robótica 2025-II</h3>

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
import rclpy
from rclpy.node import Node
from dynamixel_sdk import PortHandler, PacketHandler
import time

# ============================================================
#  CONFIGURACIÓN: ¿QUÉ MOTORES ESTÁS USANDO?
# ============================================================
USE_XL430 = True

# ------------------------------------------------------------
# Direcciones de registro y parámetros según el tipo de motor
# ------------------------------------------------------------
if USE_XL430:
    PROTOCOL_VERSION = 2.0
    ADDR_TORQUE_ENABLE    = 64
    ADDR_GOAL_POSITION    = 116
    ADDR_MOVING_SPEED     = 112  # Profile Velocity
    ADDR_TORQUE_LIMIT     = 38
    ADDR_PRESENT_POSITION = 132
    DEFAULT_GOAL = 2048
    MAX_SPEED = 1023  # Velocidad máxima para XL430
else:
    PROTOCOL_VERSION = 1.0
    ADDR_TORQUE_ENABLE    = 24
    ADDR_GOAL_POSITION    = 30
    ADDR_MOVING_SPEED     = 32
    ADDR_TORQUE_LIMIT     = 34
    ADDR_PRESENT_POSITION = 36
    DEFAULT_GOAL = 512
    MAX_SPEED = 1023  # Velocidad máxima para otros modelos

# ============================================================
#  FUNCIONES AUXILIARES
# ============================================================

def write_goal_position(packet, port, dxl_id, position):
    if USE_XL430:
        return packet.write4ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, int(position))
    else:
        return packet.write2ByteTxRx(port, dxl_id, ADDR_GOAL_POSITION, int(position))

def write_moving_speed(packet, port, dxl_id, speed):
    if USE_XL430:
        return packet.write4ByteTxRx(port, dxl_id, ADDR_MOVING_SPEED, int(speed))
    else:
        return packet.write2ByteTxRx(port, dxl_id, ADDR_MOVING_SPEED, int(speed))

def read_present_position(packet, port, dxl_id):
    if USE_XL430:
        return packet.read4ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)
    else:
        return packet.read2ByteTxRx(port, dxl_id, ADDR_PRESENT_POSITION)

# ============================================================
#  NODO ROS2 CON PUBLICACIÓN PARA RViz
# ============================================================

class PincherController(Node):
    def __init__(self):
        super().__init__('pincher_controller')

        # Parámetros
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('dxl_ids', [1, 2, 3, 4, 5])
        self.declare_parameter('goal_positions', [DEFAULT_GOAL] * 5)
        self.declare_parameter('moving_speed', 100)
        self.declare_parameter('torque_limit', 800)

        # Obtener parámetros
        port_name = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.dxl_ids = self.get_parameter('dxl_ids').value
        goal_positions = self.get_parameter('goal_positions').value
        moving_speed = int(self.get_parameter('moving_speed').value)
        torque_limit = int(self.get_parameter('torque_limit').value)

        # Inicializar comunicación
        self.port = PortHandler(port_name)
        if not self.port.openPort():
            self.get_logger().error(f'No se pudo abrir el puerto {port_name}')
            rclpy.shutdown()
            return

        if not self.port.setBaudRate(baudrate):
            self.get_logger().error(f'No se pudo configurar baudrate={baudrate}')
            self.port.closePort()
            rclpy.shutdown()
            return

        self.packet = PacketHandler(PROTOCOL_VERSION)
        
        # Estado de emergencia
        self.emergency_stop_activated = False
        
        # Configuración inicial de los motores
        self.initialize_motors(goal_positions, moving_speed, torque_limit)
        
        # Posiciones actuales de las articulaciones (en radianes)
        self.current_joint_positions = [0.0] * 5  # Para 5 articulaciones
        
        # Mapeo de IDs de motor a nombres de articulaciones
        self.joint_names = ['waist', 'shoulder', 'elbow', 'wrist', 'gripper']

        self.joint_sign = {
            1:  1,   
            2: -1,   
            3: -1,   
            4: -1,   
            5:  1,   
        }

    def initialize_motors(self, goal_positions, moving_speed, torque_limit):
        """Configuración inicial de todos los motores"""
        for dxl_id, goal in zip(self.dxl_ids, goal_positions):
            try:
                # Habilitar torque
                result, error = self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 1)
                if result != 0:
                    self.get_logger().error(f'Error habilitando torque en motor {dxl_id}: {error}')
                    continue
                
                # Configurar velocidad
                self.update_speed_single_motor(dxl_id, moving_speed)
                
                # Mover a posición inicial
                write_goal_position(self.packet, self.port, dxl_id, goal)
                
                # Actualizar posición de articulación
                joint_index = self.dxl_ids.index(dxl_id)
                angle = self.dxl_to_radians(goal)
                angle *= self.joint_sign.get(dxl_id, 1)
                self.current_joint_positions[joint_index] = angle
                
                self.get_logger().info(f'Motor {dxl_id} configurado correctamente')
                
            except Exception as e:
                self.get_logger().error(f'Error configurando motor {dxl_id}: {str(e)}')

    def dxl_to_radians(self, dxl_value):
        """Convierte valor Dynamixel (0-4095) a radianes (-pi a pi)"""
        return (dxl_value - 2048) * (2.618 / 2048.0)

    def radians_to_dxl(self, radians):
        """Convierte radianes a valor Dynamixel (0-4095)"""
        return int(radians * (2048.0 / 2.618) + 2048)

    def move_motor(self, motor_id, position):
        """Mueve un motor a la posición especificada solo si no hay emergencia y velocidad > 0"""
        if self.emergency_stop_activated:
            self.get_logger().warning(f'No se puede mover motor {motor_id}: Parada de emergencia activada')
            return
            
        try:
            result, error = write_goal_position(self.packet, self.port, motor_id, position)
            if result == 0:
                self.get_logger().info(f'[Motor {motor_id}] Moviendo a {position}')
                
                # Actualizar posición de articulación
                joint_index = self.dxl_ids.index(motor_id)
                angle = self.dxl_to_radians(position)
                angle *= self.joint_sign.get(motor_id, 1)
                self.current_joint_positions[joint_index] = angle
                
            else:
                self.get_logger().error(f'Error moviendo motor {motor_id}: {error}')
        except Exception as e:
            self.get_logger().error(f'Excepción moviendo motor {motor_id}: {str(e)}')

    def update_speed_single_motor(self, motor_id, speed):
        """Actualiza la velocidad de un motor individual"""
        try:
            result, error = write_moving_speed(self.packet, self.port, motor_id, speed)
            return result == 0
        except Exception as e:
            self.get_logger().error(f'Error actualizando velocidad motor {motor_id}: {str(e)}')
            return False

    def update_speed(self, speed):
        """Actualiza la velocidad de movimiento en todos los motores"""
        if self.emergency_stop_activated:
            self.get_logger().warning('No se puede actualizar velocidad: Parada de emergencia activada')
            return
            
        success_count = 0
        for motor_id in self.dxl_ids:
            if self.update_speed_single_motor(motor_id, speed):
                success_count += 1
        
        if success_count == len(self.dxl_ids):
            self.get_logger().info(f'Velocidad actualizada a {speed} en todos los motores')
        else:
            self.get_logger().warning(f'Velocidad actualizada a {speed} en {success_count}/{len(self.dxl_ids)} motores')

    def home_all_motors(self):
        """Mueve todos los motores a la posición home (DEFAULT_GOAL)"""
        if self.emergency_stop_activated:
            self.reactivate_torque()
            
        for motor_id in self.dxl_ids:
            self.move_motor(motor_id, DEFAULT_GOAL)
        self.get_logger().info('Todos los motores movidos a posición HOME')
        
        # Agregar un pequeño retraso para asegurar que los motores lleguen a HOME antes de cerrar el puerto
        time.sleep(2)  # Espera 2 segundos, ajusta el tiempo si es necesario

    def emergency_stop(self):
        """Parada de emergencia - desactiva el torque de todos los motores"""
        self.emergency_stop_activated = True
        for dxl_id in self.dxl_ids:
            try:
                self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 0)
                self.get_logger().warning(f'Torque desactivado en motor {dxl_id} (EMERGENCY STOP)')
            except Exception as e:
                self.get_logger().error(f'Error en parada de emergencia motor {dxl_id}: {str(e)}')

    def reactivate_torque(self):
        """Reactivar el torque después de una parada de emergencia"""
        self.emergency_stop_activated = False
        for dxl_id in self.dxl_ids:
            try:
                self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 1)
                self.get_logger().info(f'Torque reactivado en motor {dxl_id}')
            except Exception as e:
                self.get_logger().error(f'Error reactivando torque en motor {dxl_id}: {str(e)}')

    def close(self):
        """Apaga el torque y cierra el puerto"""
        for dxl_id in self.dxl_ids:
            try:
                self.packet.write1ByteTxRx(self.port, dxl_id, ADDR_TORQUE_ENABLE, 0)
            except:
                pass
        self.port.closePort()

# ============================================================
#  MAIN
# ============================================================

def main(args=None):
    rclpy.init(args=args)

    # Crear el nodo controlador
    controller = PincherController()

    try:
        # Mover todos los motores a la posición HOME
        controller.home_all_motors()
    except KeyboardInterrupt:
        pass
    finally:
        # Cerrar todo ordenadamente
        controller.close()
        controller.destroy_node()
        rclpy.shutdown()

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
