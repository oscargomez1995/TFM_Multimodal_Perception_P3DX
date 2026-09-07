#!/usr/bin/env python3

import os
import math
import pandas as pd
import numpy as np

BASE = "analysis_results"

bags = [
    "static_01",
    "linear_01",
    "linear_02",
    "rotation_left_90_01",
    "rotation_right_90_01",
    "square_01",
    "manual_navigation_01",
    "manual_navigation_02",
]

rows = []

for bag in bags:
    folder = os.path.join(BASE, bag)

    pose_path = os.path.join(folder, "pose.csv")
    ekf_path = os.path.join(folder, "odometry_filtered.csv")
    imu_path = os.path.join(folder, "imu.csv")

    if not all(os.path.exists(p) for p in [pose_path, ekf_path, imu_path]):
        print(f"Faltan archivos en {bag}")
        continue

    pose = pd.read_csv(pose_path)
    ekf = pd.read_csv(ekf_path)
    imu = pd.read_csv(imu_path)

    duration = max(
        pose["time_s"].max(),
        ekf["time_s"].max(),
        imu["time_s"].max()
    )

    # Pose
    dx = pose["x_m"].iloc[-1]
    dy = pose["y_m"].iloc[-1]
    displacement = math.hypot(dx, dy)

    path_length = np.sum(
        np.sqrt(
            np.diff(pose["x_m"])**2 +
            np.diff(pose["y_m"])**2
        )
    )

    yaw_deg = math.degrees(pose["yaw_rad"].iloc[-1])

    # EKF
    ekf_dx = ekf["x_m"].iloc[-1]
    ekf_dy = ekf["y_m"].iloc[-1]
    ekf_displacement = math.hypot(ekf_dx, ekf_dy)
    ekf_yaw_deg = math.degrees(ekf["yaw_rad"].iloc[-1])

    # Diferencia final
    final_diff = math.hypot(
        dx - ekf_dx,
        dy - ekf_dy
    )

    # IMU
    wz_mean = imu["wz_rad_s"].mean()
    wz_std = imu["wz_rad_s"].std()
    wz_min = imu["wz_rad_s"].min()
    wz_max = imu["wz_rad_s"].max()

    rows.append({
        "prueba": bag,
        "duracion_s": duration,
        "trayectoria_m": path_length,
        "desplazamiento_final_m": displacement,
        "yaw_final_deg": yaw_deg,
        "ekf_desplazamiento_final_m": ekf_displacement,
        "ekf_yaw_final_deg": ekf_yaw_deg,
        "diferencia_final_pose_ekf_m": final_diff,
        "imu_wz_media_rad_s": wz_mean,
        "imu_wz_std_rad_s": wz_std,
        "imu_wz_min_rad_s": wz_min,
        "imu_wz_max_rad_s": wz_max,
    })

df = pd.DataFrame(rows)

output_csv = os.path.join(BASE, "experimental_summary.csv")
df.to_csv(output_csv, index=False)

print("\n==============================================")
print("RESUMEN EXPERIMENTAL")
print("==============================================\n")

print(
    df[
        [
            "prueba",
            "duracion_s",
            "trayectoria_m",
            "desplazamiento_final_m",
            "yaw_final_deg",
            "diferencia_final_pose_ekf_m"
        ]
    ].to_string(index=False)
)

print("\nResumen IMU wz\n")

print(
    df[
        [
            "prueba",
            "imu_wz_media_rad_s",
            "imu_wz_std_rad_s",
            "imu_wz_min_rad_s",
            "imu_wz_max_rad_s",
        ]
    ].to_string(index=False)
)

print(f"\nCSV generado: {output_csv}")
