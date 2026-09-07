#!/usr/bin/env python3

import os
import sys
import math
import csv

import numpy as np
import matplotlib.pyplot as plt

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def quaternion_to_yaw(x, y, z, w):
    """Convierte un cuaternión ROS a yaw [rad]."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def read_bag(bag_path):
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id='sqlite3'
    )

    converter_options = rosbag2_py.ConverterOptions('', '')

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {
        topic.name: topic.type
        for topic in reader.get_all_topics_and_types()
    }

    pose = []
    filtered = []
    imu = []

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic not in ['/pose', '/odometry/filtered', '/imu/data_raw']:
            continue

        msg_type = get_message(topic_types[topic])
        msg = deserialize_message(data, msg_type)

        t = timestamp * 1e-9

        if topic in ['/pose', '/odometry/filtered']:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            tw = msg.twist.twist

            yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

            row = [
                t,
                p.x,
                p.y,
                yaw,
                tw.linear.x,
                tw.linear.y,
                tw.angular.z
            ]

            if topic == '/pose':
                pose.append(row)
            else:
                filtered.append(row)

        elif topic == '/imu/data_raw':
            imu.append([
                t,
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ])

    return (
        np.asarray(pose),
        np.asarray(filtered),
        np.asarray(imu)
    )


def normalize_time(data, t0):
    result = data.copy()
    result[:, 0] -= t0
    return result


def save_csv(path, header, data):
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(data)


def main():
    if len(sys.argv) != 2:
        print("Uso: python3 scripts/analyze_static.py <ruta_rosbag>")
        sys.exit(1)

    bag_path = sys.argv[1]

    if not os.path.isdir(bag_path):
        print(f"No existe: {bag_path}")
        sys.exit(1)

    bag_name = os.path.basename(os.path.normpath(bag_path))

    output_dir = os.path.join(
        'analysis_results',
        bag_name
    )

    os.makedirs(output_dir, exist_ok=True)

    print(f"Leyendo: {bag_path}")

    pose, filtered, imu = read_bag(bag_path)

    if len(pose) == 0 or len(filtered) == 0 or len(imu) == 0:
        print("ERROR: faltan datos requeridos.")
        print("pose:", len(pose))
        print("filtered:", len(filtered))
        print("imu:", len(imu))
        sys.exit(1)

    # Tiempo común
    t0 = min(pose[0, 0], filtered[0, 0], imu[0, 0])

    pose = normalize_time(pose, t0)
    filtered = normalize_time(filtered, t0)
    imu = normalize_time(imu, t0)

    # Desenvolver yaw
    pose[:, 3] = np.unwrap(pose[:, 3])
    filtered[:, 3] = np.unwrap(filtered[:, 3])

    # Posición relativa al inicio
    pose_rel = pose.copy()
    filtered_rel = filtered.copy()

    pose_rel[:, 1] -= pose_rel[0, 1]
    pose_rel[:, 2] -= pose_rel[0, 2]

    filtered_rel[:, 1] -= filtered_rel[0, 1]
    filtered_rel[:, 2] -= filtered_rel[0, 2]

    pose_rel[:, 3] -= pose_rel[0, 3]
    filtered_rel[:, 3] -= filtered_rel[0, 3]

    # Guardar CSV
    save_csv(
        os.path.join(output_dir, 'pose.csv'),
        ['time_s', 'x_m', 'y_m', 'yaw_rad', 'vx_m_s', 'vy_m_s', 'wz_rad_s'],
        pose_rel
    )

    save_csv(
        os.path.join(output_dir, 'odometry_filtered.csv'),
        ['time_s', 'x_m', 'y_m', 'yaw_rad', 'vx_m_s', 'vy_m_s', 'wz_rad_s'],
        filtered_rel
    )

    save_csv(
        os.path.join(output_dir, 'imu.csv'),
        ['time_s', 'wx_rad_s', 'wy_rad_s', 'wz_rad_s',
         'ax_m_s2', 'ay_m_s2', 'az_m_s2'],
        imu
    )

    # Estadísticas IMU
    imu_labels = [
        'wx [rad/s]',
        'wy [rad/s]',
        'wz [rad/s]',
        'ax [m/s²]',
        'ay [m/s²]',
        'az [m/s²]'
    ]

    print("\n==============================================")
    print("RESULTADOS DE LA PRUEBA ESTÁTICA")
    print("==============================================")

    print(f"\nMuestras /pose:              {len(pose)}")
    print(f"Muestras /odometry/filtered: {len(filtered)}")
    print(f"Muestras IMU:                {len(imu)}")

    print("\nIMU — media y desviación estándar")

    for i, label in enumerate(imu_labels, start=1):
        mean = np.mean(imu[:, i])
        std = np.std(imu[:, i])
        print(f"{label:15s}: media = {mean: .6f}, std = {std: .6f}")

    # Desplazamiento total aparente entre inicio y fin
    pose_drift = math.hypot(
        pose_rel[-1, 1],
        pose_rel[-1, 2]
    )

    filtered_drift = math.hypot(
        filtered_rel[-1, 1],
        filtered_rel[-1, 2]
    )

    pose_max = np.max(
        np.sqrt(pose_rel[:, 1] ** 2 + pose_rel[:, 2] ** 2)
    )

    filtered_max = np.max(
        np.sqrt(filtered_rel[:, 1] ** 2 + filtered_rel[:, 2] ** 2)
    )

    print("\nDesplazamiento aparente con robot inmóvil")

    print(
        f"/pose final:              {pose_drift:.6f} m"
    )

    print(
        f"/odometry/filtered final: {filtered_drift:.6f} m"
    )

    print(
        f"/pose máximo:              {pose_max:.6f} m"
    )

    print(
        f"/odometry/filtered máximo: {filtered_max:.6f} m"
    )

    print("\nCambio angular aparente")

    print(
        f"/pose:              {math.degrees(pose_rel[-1,3]):.6f} deg"
    )

    print(
        f"/odometry/filtered: {math.degrees(filtered_rel[-1,3]):.6f} deg"
    )

    # FIGURA 1: trayectoria estática
    plt.figure(figsize=(7, 6))

    plt.plot(
        pose_rel[:, 1],
        pose_rel[:, 2],
        label='/pose'
    )

    plt.plot(
        filtered_rel[:, 1],
        filtered_rel[:, 2],
        label='/odometry/filtered'
    )

    plt.xlabel('Desplazamiento X [m]')
    plt.ylabel('Desplazamiento Y [m]')
    plt.title('Prueba estática: desplazamiento aparente')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        os.path.join(output_dir, 'static_xy.png'),
        dpi=300
    )

    plt.close()

    # FIGURA 2: yaw
    plt.figure(figsize=(8, 5))

    plt.plot(
        pose_rel[:, 0],
        np.degrees(pose_rel[:, 3]),
        label='/pose'
    )

    plt.plot(
        filtered_rel[:, 0],
        np.degrees(filtered_rel[:, 3]),
        label='/odometry/filtered'
    )

    plt.xlabel('Tiempo [s]')
    plt.ylabel('Cambio de yaw [°]')
    plt.title('Prueba estática: evolución de la orientación')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        os.path.join(output_dir, 'static_yaw.png'),
        dpi=300
    )

    plt.close()

    # FIGURA 3: velocidad angular IMU Z
    plt.figure(figsize=(8, 5))

    plt.plot(
        imu[:, 0],
        imu[:, 3]
    )

    plt.axhline(
        np.mean(imu[:, 3]),
        linestyle='--',
        label='Media'
    )

    plt.xlabel('Tiempo [s]')
    plt.ylabel('Velocidad angular Z [rad/s]')
    plt.title('Prueba estática: velocidad angular medida por la IMU')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        os.path.join(output_dir, 'static_imu_wz.png'),
        dpi=300
    )

    plt.close()

    # FIGURA 4: aceleraciones IMU X/Y
    plt.figure(figsize=(8, 5))

    plt.plot(
        imu[:, 0],
        imu[:, 4],
        label='ax'
    )

    plt.plot(
        imu[:, 0],
        imu[:, 5],
        label='ay'
    )

    plt.xlabel('Tiempo [s]')
    plt.ylabel('Aceleración [m/s²]')
    plt.title('Prueba estática: aceleraciones lineales de la IMU')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.savefig(
        os.path.join(output_dir, 'static_imu_acceleration.png'),
        dpi=300
    )

    plt.close()

    print(f"\nResultados guardados en: {output_dir}")

    print("\nArchivos generados:")
    for filename in sorted(os.listdir(output_dir)):
        print(" -", filename)


if __name__ == '__main__':
    main()
