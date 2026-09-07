#!/usr/bin/env python3

import sys
import math
import numpy as np
import rosbag2_py

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def yaw(q):
    return math.atan2(
        2.0 * (q.w*q.z + q.x*q.y),
        1.0 - 2.0 * (q.y*q.y + q.z*q.z)
    )


def read(bag):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', '')
    )

    types = {
        x.name: x.type
        for x in reader.get_all_topics_and_types()
    }

    result = {
        '/pose': [],
        '/odometry/filtered': []
    }

    while reader.has_next():
        topic, data, timestamp = reader.read_next()

        if topic not in result:
            continue

        msg = deserialize_message(
            data,
            get_message(types[topic])
        )

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        tw = msg.twist.twist

        result[topic].append([
            timestamp * 1e-9,
            p.x,
            p.y,
            yaw(q),
            tw.linear.x,
            tw.linear.y,
            tw.angular.z
        ])

    return (
        np.asarray(result['/pose']),
        np.asarray(result['/odometry/filtered'])
    )


bag = sys.argv[1]
pose, ekf = read(bag)

diffs = []

# Para cada muestra del EKF buscamos la muestra /pose temporalmente más cercana
for e in ekf:
    idx = np.argmin(np.abs(pose[:,0] - e[0]))
    p = pose[idx]

    diffs.append([
        abs(p[0] - e[0]),
        abs(p[1] - e[1]),
        abs(p[2] - e[2]),
        abs(math.atan2(
            math.sin(p[3]-e[3]),
            math.cos(p[3]-e[3])
        )),
        abs(p[4] - e[4]),
        abs(p[5] - e[5]),
        abs(p[6] - e[6])
    ])

d = np.asarray(diffs)

print("Muestras pose:", len(pose))
print("Muestras EKF :", len(ekf))

print("\nDiferencias máximas, usando muestra temporal más cercana")
print(f"Δt    : {np.max(d[:,0]):.9f} s")
print(f"Δx    : {np.max(d[:,1]):.12f} m")
print(f"Δy    : {np.max(d[:,2]):.12f} m")
print(f"Δyaw  : {math.degrees(np.max(d[:,3])):.12f} deg")
print(f"Δvx   : {np.max(d[:,4]):.12f} m/s")
print(f"Δvy   : {np.max(d[:,5]):.12f} m/s")
print(f"Δwz   : {np.max(d[:,6]):.12f} rad/s")

print("\nDiferencias medias")
print(f"Δx    : {np.mean(d[:,1]):.12f} m")
print(f"Δy    : {np.mean(d[:,2]):.12f} m")
print(f"Δyaw  : {math.degrees(np.mean(d[:,3])):.12f} deg")
