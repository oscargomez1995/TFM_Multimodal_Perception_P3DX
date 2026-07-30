#!/usr/bin/env python3

import math
from enum import Enum, auto
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class State(Enum):
    WAITING_FOR_POSE = auto()
    PREPARING = auto()
    MOVING_LINEAR = auto()
    ROTATING = auto()
    PAUSED = auto()
    FINISHED = auto()
    ERROR = auto()


class MotionController(Node):
    """
    Controlador experimental para Pioneer P3-DX.

    Modos disponibles:
      - linear: avance o retroceso una distancia determinada.
      - rotation: giro sobre sí mismo un ángulo determinado.
      - square: trayectoria cuadrada de cuatro lados.
      - wall: avance, pausa frente a una pared y retroceso.

    Entradas:
      - /pose    [nav_msgs/msg/Odometry]

    Salidas:
      - /cmd_vel [geometry_msgs/msg/Twist]
    """

    def __init__(self) -> None:
        super().__init__('motion_controller')

        # Parámetros del experimento
        self.declare_parameter('mode', 'linear')
        self.declare_parameter('distance', 1.0)
        self.declare_parameter('angle_deg', 90.0)
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.25)
        self.declare_parameter('pause_seconds', 5.0)
        self.declare_parameter('start_delay', 3.0)
        self.declare_parameter('timeout_seconds', 120.0)
        self.declare_parameter('pose_topic', '/pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.mode = str(self.get_parameter('mode').value).lower()
        self.target_distance = float(self.get_parameter('distance').value)
        self.target_angle_deg = float(self.get_parameter('angle_deg').value)
        self.linear_speed = abs(
            float(self.get_parameter('linear_speed').value)
        )
        self.angular_speed = abs(
            float(self.get_parameter('angular_speed').value)
        )
        self.pause_seconds = float(
            self.get_parameter('pause_seconds').value
        )
        self.start_delay = float(
            self.get_parameter('start_delay').value
        )
        self.timeout_seconds = float(
            self.get_parameter('timeout_seconds').value
        )
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.cmd_vel_topic = str(
            self.get_parameter('cmd_vel_topic').value
        )

        self.publisher = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        self.subscription = self.create_subscription(
            Odometry,
            self.pose_topic,
            self.pose_callback,
            10
        )

        # El control se ejecuta a 20 Hz.
        self.timer = self.create_timer(0.05, self.control_loop)

        self.state = State.WAITING_FOR_POSE
        self.pose_received = False

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.start_x = 0.0
        self.start_y = 0.0
        self.previous_yaw = 0.0
        self.accumulated_angle = 0.0

        self.square_side = 0
        self.next_action: Optional[str] = None

        self.state_start_time = self.get_clock().now()
        self.experiment_start_time = self.get_clock().now()

        self.validate_parameters()

        self.get_logger().info('Controlador experimental iniciado.')
        self.get_logger().info(f'Modo seleccionado: {self.mode}')
        self.get_logger().info(f'Tópico de pose: {self.pose_topic}')
        self.get_logger().info(
            f'Tópico de velocidad: {self.cmd_vel_topic}'
        )
        self.get_logger().info(
            'Esperando el primer mensaje de odometría...'
        )

    def validate_parameters(self) -> None:
        valid_modes = {'linear', 'rotation', 'square', 'wall'}

        if self.mode not in valid_modes:
            raise ValueError(
                f'Modo no válido: {self.mode}. '
                f'Use uno de: {sorted(valid_modes)}'
            )

        if self.target_distance == 0.0 and self.mode != 'rotation':
            raise ValueError('La distancia no puede ser cero.')

        if self.linear_speed <= 0.0:
            raise ValueError('linear_speed debe ser mayor que cero.')

        if self.angular_speed <= 0.0:
            raise ValueError('angular_speed debe ser mayor que cero.')

        if self.target_angle_deg == 0.0 and self.mode == 'rotation':
            raise ValueError('angle_deg no puede ser cero.')

        if self.start_delay < 0.0:
            raise ValueError('start_delay no puede ser negativo.')

        if self.timeout_seconds <= 0.0:
            raise ValueError('timeout_seconds debe ser mayor que cero.')

    @staticmethod
    def quaternion_to_yaw(
        x: float,
        y: float,
        z: float,
        w: float
    ) -> float:
        """Convierte un cuaternión a ángulo yaw en radianes."""
        sin_yaw = 2.0 * (w * z + x * y)
        cos_yaw = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(sin_yaw, cos_yaw)

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normaliza un ángulo al intervalo [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def pose_callback(self, msg: Odometry) -> None:
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        orientation = msg.pose.pose.orientation
        new_yaw = self.quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )

        if self.pose_received and self.state == State.ROTATING:
            delta_yaw = self.normalize_angle(
                new_yaw - self.previous_yaw
            )
            self.accumulated_angle += delta_yaw

        self.current_yaw = new_yaw
        self.previous_yaw = new_yaw

        if not self.pose_received:
            self.pose_received = True
            self.state = State.PREPARING
            self.state_start_time = self.get_clock().now()
            self.experiment_start_time = self.get_clock().now()

            self.get_logger().info(
                'Odometría recibida correctamente.'
            )
            self.get_logger().info(
                f'Posición inicial: x={self.current_x:.3f} m, '
                f'y={self.current_y:.3f} m'
            )
            self.get_logger().info(
                f'Inicio automático en {self.start_delay:.1f} s.'
            )

    def elapsed_in_state(self) -> float:
        duration = self.get_clock().now() - self.state_start_time
        return duration.nanoseconds / 1e9

    def elapsed_experiment(self) -> float:
        duration = self.get_clock().now() - self.experiment_start_time
        return duration.nanoseconds / 1e9

    def publish_velocity(
        self,
        linear_x: float = 0.0,
        angular_z: float = 0.0
    ) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)

    def stop_robot(self) -> None:
        """Publica varios comandos cero para asegurar la detención."""
        stop_msg = Twist()

        for _ in range(5):
            self.publisher.publish(stop_msg)

    def begin_linear(self, distance: float) -> None:
        self.target_distance = distance
        self.start_x = self.current_x
        self.start_y = self.current_y
        self.state = State.MOVING_LINEAR
        self.state_start_time = self.get_clock().now()

        direction = 'avance' if distance > 0.0 else 'retroceso'

        self.get_logger().info(
            f'Iniciando {direction}: '
            f'{abs(distance):.2f} m a {self.linear_speed:.2f} m/s.'
        )

    def begin_rotation(self, angle_deg: float) -> None:
        self.target_angle_deg = angle_deg
        self.accumulated_angle = 0.0
        self.previous_yaw = self.current_yaw
        self.state = State.ROTATING
        self.state_start_time = self.get_clock().now()

        direction = 'izquierda' if angle_deg > 0.0 else 'derecha'

        self.get_logger().info(
            f'Iniciando giro hacia la {direction}: '
            f'{abs(angle_deg):.1f} grados a '
            f'{self.angular_speed:.2f} rad/s.'
        )

    def begin_pause(
        self,
        duration: float,
        next_action: Optional[str]
    ) -> None:
        self.stop_robot()
        self.pause_seconds = duration
        self.next_action = next_action
        self.state = State.PAUSED
        self.state_start_time = self.get_clock().now()

        self.get_logger().info(
            f'Robot detenido durante {duration:.1f} s.'
        )

    def distance_travelled(self) -> float:
        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        return math.hypot(dx, dy)

    def execute_initial_action(self) -> None:
        if self.mode == 'linear':
            self.begin_linear(self.target_distance)

        elif self.mode == 'rotation':
            self.begin_rotation(self.target_angle_deg)

        elif self.mode == 'square':
            self.square_side = 1
            self.begin_linear(abs(self.target_distance))

        elif self.mode == 'wall':
            self.begin_linear(abs(self.target_distance))

    def control_linear(self) -> None:
        travelled = self.distance_travelled()
        target = abs(self.target_distance)

        if travelled >= target:
            self.stop_robot()

            self.get_logger().info(
                f'Distancia alcanzada: {travelled:.3f} m.'
            )

            if self.mode == 'square':
                self.begin_pause(1.0, 'square_rotate')

            elif self.mode == 'wall' and self.target_distance > 0.0:
                self.begin_pause(
                    self.pause_seconds,
                    'wall_return'
                )

            else:
                self.finish_experiment()

            return

        direction = 1.0 if self.target_distance > 0.0 else -1.0
        self.publish_velocity(
            linear_x=direction * self.linear_speed
        )

    def control_rotation(self) -> None:
        target_rad = math.radians(abs(self.target_angle_deg))
        turned_rad = abs(self.accumulated_angle)

        if turned_rad >= target_rad:
            self.stop_robot()

            self.get_logger().info(
                f'Ángulo alcanzado: '
                f'{math.degrees(turned_rad):.2f} grados.'
            )

            if self.mode == 'square':
                if self.square_side >= 4:
                    self.finish_experiment()
                else:
                    self.square_side += 1
                    self.begin_pause(1.0, 'square_linear')
            else:
                self.finish_experiment()

            return

        direction = 1.0 if self.target_angle_deg > 0.0 else -1.0
        self.publish_velocity(
            angular_z=direction * self.angular_speed
        )

    def control_pause(self) -> None:
        self.stop_robot()

        if self.elapsed_in_state() < self.pause_seconds:
            return

        action = self.next_action
        self.next_action = None

        if action == 'square_rotate':
            self.begin_rotation(90.0)

        elif action == 'square_linear':
            self.get_logger().info(
                f'Iniciando lado {self.square_side} de 4.'
            )
            self.begin_linear(abs(self.target_distance))

        elif action == 'wall_return':
            self.get_logger().info(
                'Iniciando regreso al punto de partida.'
            )
            self.begin_linear(-abs(self.target_distance))

        else:
            self.finish_experiment()

    def finish_experiment(self) -> None:
        self.stop_robot()
        self.state = State.FINISHED

        self.get_logger().info('Experimento finalizado.')
        self.get_logger().info(
            f'Tiempo total: {self.elapsed_experiment():.2f} s.'
        )
        self.get_logger().info(
            f'Posición final: x={self.current_x:.3f} m, '
            f'y={self.current_y:.3f} m, '
            f'yaw={math.degrees(self.current_yaw):.2f} grados.'
        )

    def abort_experiment(self, reason: str) -> None:
        self.stop_robot()
        self.state = State.ERROR
        self.get_logger().error(reason)

    def control_loop(self) -> None:
        if self.state == State.WAITING_FOR_POSE:
            self.stop_robot()
            return

        if self.state in {
            State.FINISHED,
            State.ERROR
        }:
            self.stop_robot()
            return

        if self.elapsed_experiment() > self.timeout_seconds:
            self.abort_experiment(
                'Tiempo máximo excedido. Robot detenido.'
            )
            return

        if self.state == State.PREPARING:
            self.stop_robot()

            if self.elapsed_in_state() >= self.start_delay:
                self.execute_initial_action()

        elif self.state == State.MOVING_LINEAR:
            self.control_linear()

        elif self.state == State.ROTATING:
            self.control_rotation()

        elif self.state == State.PAUSED:
            self.control_pause()

    def destroy_node(self) -> bool:
        self.get_logger().info(
            'Deteniendo el robot antes de cerrar el nodo.'
        )
        self.stop_robot()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[MotionController] = None

    try:
        node = MotionController()
        rclpy.spin(node)

    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().warning(
                'Interrupción solicitada mediante Ctrl+C.'
            )

    except Exception as exc:
        if node is not None:
            node.get_logger().error(
                f'Error durante el experimento: {exc}'
            )
        else:
            print(f'Error al iniciar el nodo: {exc}')

    finally:
        if rclpy.ok():
            if node is not None:
                node.destroy_node()

            rclpy.shutdown()


if __name__ == '__main__':
    main()
