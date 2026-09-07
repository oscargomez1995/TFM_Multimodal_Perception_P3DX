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
        x.name: x.type
        for x in reader.get_all_topics_and_types()
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
                msg.angular_velocity.z,
                msg.linear_acceleration.x,
                msg.linear_acceleration.y
            ])

    return np.asarray(pose), np.asarray(filtered), np.asarray(imu)


def save_csv(path, header, data):
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(data)


def path_length(data):
    dx = np.diff(data[:, 1])
    dy = np.diff(data[:, 2])
    return np.sum(np.sqrt(dx**2 + dy**2))


def main():
    if len(sys.argv) != 2:
        print("Uso: python3 scripts/analyze_motion.py <ruta_rosbag>")
        sys.exit(1)

    bag_path = sys.argv[1]
    bag_name = os.path.basename(os.path.normpath(bag_path))

    output_dir = os.path.join('analysis_results', bag_name)
    os.makedirs(output_dir, exist_ok=True)

    print(f"Leyendo: {bag_path}")

    pose, filtered, imu = read_bag(bag_path)

    if len(pose) == 0 or len(filtered) == 0 or len(imu) == 0:
        print("ERROR: faltan datos.")
        print("pose:", len(pose))
        print("filtered:", len(filtered))
        print("imu:", len(imu))
        sys.exit(1)

    t0 = min(pose[0,0], filtered[0,0], imu[0,0])

    pose[:,0] -= t0
    filtered[:,0] -= t0
    imu[:,0] -= t0

    pose[:,3] = np.unwrap(pose[:,3])
    filtered[:,3] = np.unwrap(filtered[:,3])

    # Normalización espacial respecto al inicio de cada fuente
    pose[:,1] -= pose[0,1]
    pose[:,2] -= pose[0,2]
    pose[:,3] -= pose[0,3]

    filtered[:,1] -= filtered[0,1]
    filtered[:,2] -= filtered[0,2]
    filtered[:,3] -= filtered[0,3]

    save_csv(
        os.path.join(output_dir, 'pose.csv'),
        ['time_s','x_m','y_m','yaw_rad','vx_m_s','vy_m_s','wz_rad_s'],
        pose
    )

    save_csv(
        os.path.join(output_dir, 'odometry_filtered.csv'),
        ['time_s','x_m','y_m','yaw_rad','vx_m_s','vy_m_s','wz_rad_s'],
        filtered
    )

    save_csv(
        os.path.join(output_dir, 'imu.csv'),
        ['time_s','wz_rad_s','ax_m_s2','ay_m_s2'],
        imu
    )

    pose_disp = math.hypot(pose[-1,1], pose[-1,2])
    filtered_disp = math.hypot(filtered[-1,1], filtered[-1,2])

    pose_path = path_length(pose)
    filtered_path = path_length(filtered)

    pose_yaw = math.degrees(pose[-1,3])
    filtered_yaw = math.degrees(filtered[-1,3])

    final_diff = math.hypot(
        pose[-1,1] - filtered[-1,1],
        pose[-1,2] - filtered[-1,2]
    )

    print("\n==============================================")
    print(f"RESULTADOS: {bag_name}")
    print("==============================================")

    print(f"Muestras /pose:              {len(pose)}")
    print(f"Muestras /odometry/filtered: {len(filtered)}")
    print(f"Muestras IMU:                {len(imu)}")

    print("\nDesplazamiento neto")
    print(f"/pose:              {pose_disp:.6f} m")
    print(f"/odometry/filtered: {filtered_disp:.6f} m")

    print("\nLongitud de trayectoria")
    print(f"/pose:              {pose_path:.6f} m")
    print(f"/odometry/filtered: {filtered_path:.6f} m")

    print("\nCambio de orientación")
    print(f"/pose:              {pose_yaw:.6f} deg")
    print(f"/odometry/filtered: {filtered_yaw:.6f} deg")

    print("\nDiferencia final pose vs EKF")
    print(f"{final_diff:.6f} m")

    print("\nIMU wz")
    print(f"media: {np.mean(imu[:,1]):.6f} rad/s")
    print(f"std:   {np.std(imu[:,1]):.6f} rad/s")
    print(f"min:   {np.min(imu[:,1]):.6f} rad/s")
    print(f"max:   {np.max(imu[:,1]):.6f} rad/s")

    # Trayectoria XY
    plt.figure(figsize=(7,6))
    plt.plot(pose[:,1], pose[:,2], label='/pose')
    plt.plot(filtered[:,1], filtered[:,2], label='/odometry/filtered')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title(f'Trayectoria XY - {bag_name}')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(
        os.path.join(output_dir, 'trajectory_xy.png'),
        dpi=300
    )
    plt.close()

    # X e Y
    plt.figure(figsize=(8,5))
    plt.plot(pose[:,0], pose[:,1], label='X /pose')
    plt.plot(pose[:,0], pose[:,2], label='Y /pose')
    plt.plot(filtered[:,0], filtered[:,1], '--', label='X EKF')
    plt.plot(filtered[:,0], filtered[:,2], '--', label='Y EKF')
    plt.xlabel('Tiempo [s]')
    plt.ylabel('Posición relativa [m]')
    plt.title(f'Posición en el tiempo - {bag_name}')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(
        os.path.join(output_dir, 'position_time.png'),
        dpi=300
    )
    plt.close()

    # Yaw
    plt.figure(figsize=(8,5))
    plt.plot(pose[:,0], np.degrees(pose[:,3]), label='/pose')
    plt.plot(filtered[:,0], np.degrees(filtered[:,3]), label='/odometry/filtered')
    plt.xlabel('Tiempo [s]')
    plt.ylabel('Yaw relativo [°]')
    plt.title(f'Orientación - {bag_name}')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(
        os.path.join(output_dir, 'yaw_time.png'),
        dpi=300
    )
    plt.close()

    # Velocidad angular IMU
    plt.figure(figsize=(8,5))
    plt.plot(imu[:,0], imu[:,1])
    plt.xlabel('Tiempo [s]')
    plt.ylabel('Velocidad angular Z [rad/s]')
    plt.title(f'IMU: velocidad angular Z - {bag_name}')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(
        os.path.join(output_dir, 'imu_wz.png'),
        dpi=300
    )
    plt.close()

    print(f"\nResultados guardados en: {output_dir}")


if __name__ == '__main__':
    main()
