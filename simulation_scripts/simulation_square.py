#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class SquareController(Node):

    def __init__(self):
        super().__init__('simulation_square_controller')

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

    def send_command(self, linear_x, angular_z, duration):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z

        start = time.monotonic()

        while time.monotonic() - start < duration:
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)

        self.stop()

    def stop(self):
        msg = Twist()

        # Publicamos varias veces la parada para asegurar que Gazebo la reciba.
        for _ in range(10):
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.05)


def main():
    rclpy.init()

    controller = SquareController()

    linear_speed = 0.30       # m/s
    linear_time = 4.0         # s

    angular_speed = 0.50      # rad/s
    rotation_time = 3.1416    # s ≈ 90 grados

    print("La prueba comenzará en 3 segundos...")
    time.sleep(3)

    for side in range(4):

        print(f"Lado {side + 1}/4")
        controller.send_command(
            linear_speed,
            0.0,
            linear_time
        )

        print("Pausa")
        time.sleep(1)

        print(f"Giro {side + 1}/4")
        controller.send_command(
            0.0,
            angular_speed,
            rotation_time
        )

        print("Pausa")
        time.sleep(1)

    controller.stop()

    print("Prueba finalizada.")

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
