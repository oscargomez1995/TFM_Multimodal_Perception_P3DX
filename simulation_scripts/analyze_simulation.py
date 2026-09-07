#!/usr/bin/env python3

import os
import sys
import math

import numpy as np
import matplotlib.pyplot as plt

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def read_bag(bag_path):

    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id='mcap'
    )

    converter_options = rosbag2_py.ConverterOptions('', '')

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {
        topic.name: topic.type
        for topic in reader.get_all_topics_and_types()
    }

    odom = []

    while reader.has_next():

        topic, data, timestamp = reader.read_next()

        if topic != '/odom':
            continue

        msg = deserialize_message(
            data,
            get_message(topic_types[topic])
        )

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        yaw = quaternion_to_yaw(
            q.x,
            q.y,
            q.z,
            q.w
        )

        odom.append([
            timestamp * 1e-9,
            p.x,
            p.y,
            yaw
        ])

    return np.asarray(odom)


def main():

    if len(sys.argv) != 2:
        print("Uso: python3 analyze_simulation.py <rosbag>")
        sys.exit(1)

    bag_path = sys.argv[1]

    name = os.path.basename(
        os.path.normpath(bag_path)
    )

    output_dir = os.path.join(
        os.path.dirname(bag_path),
        name + "_analysis"
    )

    os.makedirs(output_dir, exist_ok=True)

    odom = read_bag(bag_path)

    if len(odom) == 0:
        print("No se encontraron datos /odom")
        sys.exit(1)

    # Tiempo relativo
    odom[:, 0] -= odom[0, 0]

    # Posición relativa
    x0 = odom[0, 1]
    y0 = odom[0, 2]

    odom[:, 1] -= x0
    odom[:, 2] -= y0

    # Orientación continua
    odom[:, 3] = np.unwrap(odom[:, 3])
    odom[:, 3] -= odom[0, 3]

    dx = np.diff(odom[:, 1])
    dy = np.diff(odom[:, 2])

    path_length = np.sum(
        np.sqrt(dx**2 + dy**2)
    )

    displacement = math.hypot(
        odom[-1, 1],
        odom[-1, 2]
    )

    yaw_deg = math.degrees(
        odom[-1, 3]
    )

    print("\n==============================================")
    print("RESULTADOS DE SIMULACIÓN")
    print("==============================================")

    print(f"Muestras /odom:        {len(odom)}")
    print(f"Duración analizada:    {odom[-1,0]:.3f} s")
    print(f"Longitud trayectoria:  {path_length:.6f} m")
    print(f"Desplazamiento final:  {displacement:.6f} m")
    print(f"Cambio angular final:  {yaw_deg:.6f} deg")

    # Trayectoria XY
    plt.figure(figsize=(7, 6))

    plt.plot(
        odom[:,1],
        odom[:,2]
    )

    plt.scatter(
        odom[0,1],
        odom[0,2],
        label='Inicio'
    )

    plt.scatter(
        odom[-1,1],
        odom[-1,2],
        label='Final'
    )

    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Trayectoria simulada del Pioneer P3-DX')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()

    plt.tight_layout()

    plt.savefig(
        os.path.join(
            output_dir,
            'simulation_trajectory_xy.png'
        ),
        dpi=300
    )

    plt.close()

    # Posición temporal
    plt.figure(figsize=(8,5))

    plt.plot(
        odom[:,0],
        odom[:,1],
        label='X'
    )

    plt.plot(
        odom[:,0],
        odom[:,2],
        label='Y'
    )

    plt.xlabel('Tiempo [s]')
    plt.ylabel('Posición relativa [m]')
    plt.title('Posición durante la prueba de simulación')
    plt.grid(True)
    plt.legend()

    plt.tight_layout()

    plt.savefig(
        os.path.join(
            output_dir,
            'simulation_position_time.png'
        ),
        dpi=300
    )

    plt.close()

    # Yaw
    plt.figure(figsize=(8,5))

    plt.plot(
        odom[:,0],
        np.degrees(odom[:,3])
    )

    plt.xlabel('Tiempo [s]')
    plt.ylabel('Yaw relativo [°]')
    plt.title('Orientación durante la prueba de simulación')
    plt.grid(True)

    plt.tight_layout()

    plt.savefig(
        os.path.join(
            output_dir,
            'simulation_yaw.png'
        ),
        dpi=300
    )

    plt.close()

    print(f"\nResultados guardados en:\n{output_dir}")


if __name__ == '__main__':
    main()
